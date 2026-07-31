// SPDX-License-Identifier: MIT
// Read-only ROS image-sequence source configured entirely with ROS parameters.

#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include <m3t/body.h>

#include "m3t_ros2/body_factory.hpp"
#include "m3t_ros2/ground_truth_publisher.hpp"

namespace fs = std::filesystem;

namespace {

std::string FormatFrameName(const std::string &pattern, int index) {
  std::vector<char> buffer(pattern.size() + 64, '\0');
  const int written =
      std::snprintf(buffer.data(), buffer.size(), pattern.c_str(), index);
  if (written < 0 || static_cast<size_t>(written) >= buffer.size()) {
    throw std::runtime_error("invalid or too-long image pattern: " + pattern);
  }
  return std::string{buffer.data()};
}

sensor_msgs::msg::CameraInfo MakeCameraInfo(
    const std::vector<double> &values, const std::string &frame_id) {
  if (values.size() != 6) {
    throw std::runtime_error(
        "camera_intrinsics must be [fu, fv, ppu, ppv, width, height]");
  }
  sensor_msgs::msg::CameraInfo info;
  info.header.frame_id = frame_id;
  info.width = static_cast<uint32_t>(values[4]);
  info.height = static_cast<uint32_t>(values[5]);
  info.distortion_model = "plumb_bob";
  info.d = {0, 0, 0, 0, 0};
  info.k = {values[0], 0, values[2], 0, values[1], values[3], 0, 0, 1};
  info.p = {values[0], 0, values[2], 0, 0, values[1],
            values[3], 0, 0, 0, 1, 0};
  return info;
}

std::vector<m3t::Transform3fA> ParseGtPoses(
    const std::vector<double> &flat_values) {
  if (flat_values.size() % 16 != 0) {
    throw std::runtime_error(
        "gt_poses must contain 16 row-major values per frame");
  }
  std::vector<m3t::Transform3fA> poses;
  poses.reserve(flat_values.size() / 16);
  for (size_t offset = 0; offset < flat_values.size(); offset += 16) {
    std::vector<double> values{flat_values.begin() + offset,
                               flat_values.begin() + offset + 16};
    poses.push_back(
        m3t_ros2::TransformFromRowMajor(values, "gt_poses"));
  }
  return poses;
}

}  // namespace

class ImagePublisherNode : public rclcpp::Node {
 public:
  ImagePublisherNode() : Node{"m3t_image_publisher"} {
    sequence_dir_ = fs::path{
        declare_parameter<std::string>("sequence_dir", std::string{})};
    color_pattern_ =
        declare_parameter<std::string>("color_pattern", "frame%04d.png");
    depth_pattern_ =
        declare_parameter<std::string>("depth_pattern", "depth%04d.png");
    start_index_ = declare_parameter<int>("start_index", 0);
    frame_index_ = start_index_;
    n_frames_ = declare_parameter<int>("n_frames", 0);
    publish_rate_ = declare_parameter<double>("publish_rate", 30.0);
    gt_publish_rate_ =
        declare_parameter<double>("gt_publish_rate", 60.0);
    loop_ = declare_parameter<bool>("loop", true);
    publish_gt_ = declare_parameter<bool>("publish_gt", true);
    world_frame_ = declare_parameter<std::string>("world_frame", "camera");
    camera_frame_ =
        declare_parameter<std::string>("camera_frame", world_frame_);
    gt_frame_ = declare_parameter<std::string>("gt_frame", "object_gt");
    mesh_resource_ =
        declare_parameter<std::string>("mesh_resource", std::string{});
    mesh_scale_ = declare_parameter<double>("mesh_scale", 1.0);
    const auto texture_path =
        declare_parameter<std::string>("texture_path", std::string{});
    mesh_embedded_ = declare_parameter<bool>(
        "mesh_use_embedded_materials", !texture_path.empty());
    const auto intrinsics = declare_parameter<std::vector<double>>(
        "camera_intrinsics",
        {698.128, 698.617, 478.459, 274.426, 960.0, 540.0});
    gt_poses_ = ParseGtPoses(
        declare_parameter<std::vector<double>>(
            "gt_poses", std::vector<double>{}));

    const auto color_topic = declare_parameter<std::string>(
        "color_topic", "/camera/color/image_raw");
    const auto depth_topic = declare_parameter<std::string>(
        "depth_topic", "/camera/depth/image_raw");
    const auto color_info_topic = declare_parameter<std::string>(
        "color_info_topic", "/camera/color/camera_info");
    const auto depth_info_topic = declare_parameter<std::string>(
        "depth_info_topic", "/camera/depth/camera_info");
    const auto gt_pose_topic = declare_parameter<std::string>(
        "gt_pose_topic", "/m3t/pose_gt");
    const auto gt_marker_topic = declare_parameter<std::string>(
        "gt_marker_topic", "/m3t/marker_gt");

    if (sequence_dir_.empty() || !fs::is_directory(sequence_dir_)) {
      throw std::runtime_error(
          "sequence_dir must be an existing read-only image directory");
    }
    if (publish_rate_ <= 0.0 || gt_publish_rate_ <= 0.0 ||
        start_index_ < 0 || n_frames_ < 0) {
      throw std::runtime_error(
          "publish rates must be positive; "
          "frame indices must be non-negative");
    }

    body_ = m3t_ros2::DeclareAndCreateBody(this);
    if (!body_->SetUp()) {
      throw std::runtime_error("failed to set up object geometry");
    }
    color_info_ = MakeCameraInfo(intrinsics, camera_frame_);
    depth_info_ = MakeCameraInfo(intrinsics, camera_frame_);

    auto qos = rclcpp::SensorDataQoS();
    pub_color_ =
        create_publisher<sensor_msgs::msg::Image>(color_topic, qos);
    pub_depth_ =
        create_publisher<sensor_msgs::msg::Image>(depth_topic, qos);
    pub_color_info_ =
        create_publisher<sensor_msgs::msg::CameraInfo>(color_info_topic, qos);
    pub_depth_info_ =
        create_publisher<sensor_msgs::msg::CameraInfo>(depth_info_topic, qos);
    if (publish_gt_) {
      m3t_ros2::GroundTruthPublisherConfig gt_config;
      gt_config.world_frame = world_frame_;
      gt_config.gt_frame = gt_frame_;
      gt_config.pose_topic = gt_pose_topic;
      gt_config.marker_topic = gt_marker_topic;
      gt_config.mesh_resource = mesh_resource_;
      gt_config.mesh_scale = static_cast<float>(mesh_scale_);
      gt_config.mesh_use_embedded_materials = mesh_embedded_;
      gt_config.publish_rate = gt_publish_rate_;
      gt_config.geometry2body_pose = body_->geometry2body_pose();
      gt_publisher_ =
          std::make_unique<m3t_ros2::GroundTruthPublisher>(this, gt_config);
    }

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>{1.0 / publish_rate_});
    timer_ = create_wall_timer(period, [this]() { PublishNextFrame(); });
    RCLCPP_INFO(get_logger(),
                "read-only sequence source: %s at %.1f Hz; "
                "GT poses=%zu GT rate=%.1f Hz",
                sequence_dir_.c_str(), publish_rate_, gt_poses_.size(),
                gt_publish_rate_);
  }

 private:
  bool LoadFrame(cv::Mat *color, cv::Mat *depth) const {
    const fs::path color_path =
        sequence_dir_ / FormatFrameName(color_pattern_, frame_index_);
    *color = cv::imread(color_path.string(), cv::IMREAD_COLOR);
    if (color->empty()) return false;

    const fs::path depth_path =
        sequence_dir_ / FormatFrameName(depth_pattern_, frame_index_);
    *depth = cv::imread(depth_path.string(), cv::IMREAD_UNCHANGED);
    if (!depth->empty() && depth->type() != CV_16UC1) {
      throw std::runtime_error("depth image is not 16UC1: " +
                               depth_path.string());
    }
    return true;
  }

  void PublishNextFrame() {
    if (n_frames_ > 0 && frame_index_ >= start_index_ + n_frames_) {
      if (loop_) {
        frame_index_ = start_index_;
      } else {
        timer_->cancel();
        return;
      }
    }

    cv::Mat color;
    cv::Mat depth;
    if (!LoadFrame(&color, &depth)) {
      if (loop_ && frame_index_ != start_index_) {
        frame_index_ = start_index_;
        if (!LoadFrame(&color, &depth)) {
          RCLCPP_ERROR(get_logger(), "cannot load first sequence frame");
          timer_->cancel();
          return;
        }
      } else {
        RCLCPP_INFO(get_logger(), "sequence complete");
        timer_->cancel();
        return;
      }
    }

    const int relative_index = frame_index_ - start_index_;
    const rclcpp::Time stamp = now();
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = camera_frame_;
    color_info_.header.stamp = stamp;
    depth_info_.header.stamp = stamp;

    if (publish_gt_ && relative_index >= 0 &&
        static_cast<size_t>(relative_index) < gt_poses_.size()) {
      gt_publisher_->Update(stamp, gt_poses_[relative_index]);
    }
    pub_color_info_->publish(color_info_);
    if (!depth.empty()) {
      depth_info_.header.stamp = stamp;
      pub_depth_info_->publish(depth_info_);
      pub_depth_->publish(
          *cv_bridge::CvImage(header, "16UC1", depth).toImageMsg());
    }
    pub_color_->publish(
        *cv_bridge::CvImage(header, "bgr8", color).toImageMsg());
    ++frame_index_;
  }

  fs::path sequence_dir_;
  std::string color_pattern_;
  std::string depth_pattern_;
  std::string world_frame_;
  std::string camera_frame_;
  std::string gt_frame_;
  std::string mesh_resource_;
  int start_index_{0};
  int frame_index_{0};
  int n_frames_{0};
  double publish_rate_{30.0};
  double gt_publish_rate_{60.0};
  bool loop_{true};
  bool publish_gt_{true};
  double mesh_scale_{1.0};
  bool mesh_embedded_{false};
  std::vector<m3t::Transform3fA> gt_poses_;
  std::shared_ptr<m3t::Body> body_;
  sensor_msgs::msg::CameraInfo color_info_;
  sensor_msgs::msg::CameraInfo depth_info_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_color_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_color_info_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_depth_info_;
  std::unique_ptr<m3t_ros2::GroundTruthPublisher> gt_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ImagePublisherNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("m3t_image_publisher"), "%s",
                 error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}