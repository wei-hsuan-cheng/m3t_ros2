// SPDX-License-Identifier: MIT
// Online synthetic RGB-D source for M3T.
//
// This is the ROS-native replacement for M3T/examples/generate_orbit_sequence:
// frames, CameraInfo, and ground truth are published directly and no sequence
// files are created in the source tree.

#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

#include <m3t/body.h>
#include <m3t/normal_renderer.h>
#include <m3t/renderer_geometry.h>

#include "m3t_ros2/body_factory.hpp"
#include "m3t_ros2/ground_truth_publisher.hpp"
#include "m3t_ros2/textured_renderer.hpp"

namespace {

constexpr float kPi = 3.14159265358979323846f;

sensor_msgs::msg::CameraInfo MakeCameraInfo(const m3t::Intrinsics &intrinsics,
                                            const std::string &frame_id) {
  sensor_msgs::msg::CameraInfo info;
  info.header.frame_id = frame_id;
  info.width = intrinsics.width;
  info.height = intrinsics.height;
  info.distortion_model = "plumb_bob";
  info.d = {0, 0, 0, 0, 0};
  info.k = {intrinsics.fu, 0, intrinsics.ppu, 0, intrinsics.fv,
            intrinsics.ppv, 0, 0, 1};
  info.p = {intrinsics.fu, 0, intrinsics.ppu, 0, 0, intrinsics.fv,
            intrinsics.ppv, 0, 0, 0, 1, 0};
  return info;
}

}  // namespace

class SyntheticSourceNode : public rclcpp::Node {
 public:
  SyntheticSourceNode() : Node{"m3t_synthetic_source"}, rng_{12345} {
    publish_rate_ = declare_parameter<double>("publish_rate", 30.0);
    gt_publish_rate_ =
        declare_parameter<double>("gt_publish_rate", 60.0);
    n_frames_ = declare_parameter<int>("n_frames", 240);
    loop_ = declare_parameter<bool>("loop", true);
    depth_noise_ = declare_parameter<double>("depth_noise", 0.0);
    distortion_ = declare_parameter<double>("distortion", 0.0);
    depth_scale_ = declare_parameter<double>("depth_scale", 0.001);
    spin_turns_ = declare_parameter<double>("spin_turns", 1.0);
    nod_degrees_ = declare_parameter<double>("nod_degrees", 25.0);
    motion_mode_ =
        declare_parameter<std::string>("motion_mode", "orbit");
    const auto gt_initial_pose_values =
        declare_parameter<std::vector<double>>(
            "gt_initial_pose", std::vector<double>{});
    const auto translation_amplitude_values =
        declare_parameter<std::vector<double>>(
            "translation_amplitude", std::vector<double>{});
    const auto translation_sine_amplitude_values =
        declare_parameter<std::vector<double>>(
            "translation_amplitude_m", {0.0, 0.0, 0.0});
    const auto translation_sine_frequency_values =
        declare_parameter<std::vector<double>>(
            "translation_frequency_hz", {0.0, 0.0, 0.0});
    const auto translation_sine_phase_values =
        declare_parameter<std::vector<double>>(
            "translation_phase_deg", {0.0, 0.0, 0.0});
    const auto rotation_sine_amplitude_values =
        declare_parameter<std::vector<double>>(
            "rotation_amplitude_deg", {0.0, 0.0, 0.0});
    const auto rotation_sine_frequency_values =
        declare_parameter<std::vector<double>>(
            "rotation_frequency_hz", {0.0, 0.0, 0.0});
    const auto rotation_sine_phase_values =
        declare_parameter<std::vector<double>>(
            "rotation_phase_deg", {0.0, 0.0, 0.0});
    translation_frame_ =
        declare_parameter<std::string>("translation_frame", "world");
    rotation_frame_ =
        declare_parameter<std::string>("rotation_frame", "body");
    rotation_pivot_ =
        declare_parameter<std::string>("rotation_pivot", "geometry_center");
    world_frame_ = declare_parameter<std::string>("world_frame", "camera");
    camera_frame_ =
        declare_parameter<std::string>("camera_frame", world_frame_);
    gt_frame_ = declare_parameter<std::string>("gt_frame", "object_gt");
    mesh_resource_ =
        declare_parameter<std::string>("mesh_resource", std::string{});
    mesh_scale_ = declare_parameter<double>("mesh_scale", 1.0);
    texture_path_ =
        declare_parameter<std::string>("texture_path", std::string{});
    mesh_embedded_ = declare_parameter<bool>(
        "mesh_use_embedded_materials", !texture_path_.empty());
    const auto camera_intrinsics =
        declare_parameter<std::vector<double>>(
            "camera_intrinsics",
            {698.128, 698.617, 478.459, 274.426, 960.0, 540.0});
    if (camera_intrinsics.size() != 6 || camera_intrinsics[0] <= 0.0 ||
        camera_intrinsics[1] <= 0.0 || camera_intrinsics[4] <= 0.0 ||
        camera_intrinsics[5] <= 0.0) {
      throw std::runtime_error(
          "camera_intrinsics must be positive "
          "[fu, fv, ppu, ppv, width, height]");
    }
    intrinsics_ = {
        static_cast<float>(camera_intrinsics[0]),
        static_cast<float>(camera_intrinsics[1]),
        static_cast<float>(camera_intrinsics[2]),
        static_cast<float>(camera_intrinsics[3]),
        static_cast<int>(camera_intrinsics[4]),
        static_cast<int>(camera_intrinsics[5])};

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

    if (motion_mode_ != "orbit" && motion_mode_ != "static" &&
        motion_mode_ != "six_dof_sine") {
      throw std::runtime_error(
          "motion_mode must be orbit, six_dof_sine, or static");
    }
    if (translation_frame_ != "world" && translation_frame_ != "body") {
      throw std::runtime_error(
          "translation_frame must be world or body");
    }
    if (rotation_frame_ != "world" && rotation_frame_ != "body") {
      throw std::runtime_error("rotation_frame must be world or body");
    }
    if (rotation_pivot_ != "geometry_center" &&
        rotation_pivot_ != "body_origin") {
      throw std::runtime_error(
          "rotation_pivot must be geometry_center or body_origin");
    }
    if (!gt_initial_pose_values.empty()) {
      gt_initial_pose_ =
          m3t_ros2::TransformFromPose(gt_initial_pose_values,
                                      "gt_initial_pose");
      has_gt_initial_pose_ = true;
    }
    if (!translation_amplitude_values.empty()) {
      if (translation_amplitude_values.size() != 3) {
        throw std::runtime_error(
            "translation_amplitude must contain [x, y, z] in meters");
      }
      translation_amplitude_ = Eigen::Vector3f{
          static_cast<float>(translation_amplitude_values[0]),
          static_cast<float>(translation_amplitude_values[1]),
          static_cast<float>(translation_amplitude_values[2])};
      if (!translation_amplitude_.allFinite()) {
        throw std::runtime_error(
            "translation_amplitude must contain finite values");
      }
      has_translation_amplitude_ = true;
    }
    translation_sine_amplitude_m_ = ParseVector3(
        translation_sine_amplitude_values, "translation_amplitude_m");
    translation_sine_frequency_hz_ = ParseVector3(
        translation_sine_frequency_values, "translation_frequency_hz");
    translation_sine_phase_rad_ =
        ParseVector3(translation_sine_phase_values, "translation_phase_deg") *
        (kPi / 180.0f);
    rotation_sine_amplitude_rad_ =
        ParseVector3(rotation_sine_amplitude_values, "rotation_amplitude_deg") *
        (kPi / 180.0f);
    rotation_sine_frequency_hz_ = ParseVector3(
        rotation_sine_frequency_values, "rotation_frequency_hz");
    rotation_sine_phase_rad_ =
        ParseVector3(rotation_sine_phase_values, "rotation_phase_deg") *
        (kPi / 180.0f);
    if ((translation_sine_frequency_hz_.array() < 0.0f).any() ||
        (rotation_sine_frequency_hz_.array() < 0.0f).any()) {
      throw std::runtime_error(
          "translation_frequency_hz and rotation_frequency_hz "
          "must be non-negative");
    }
    if (publish_rate_ <= 0.0 || gt_publish_rate_ <= 0.0 ||
        n_frames_ <= 1 || depth_scale_ <= 0.0 ||
        !std::isfinite(spin_turns_) || !std::isfinite(nod_degrees_)) {
      throw std::runtime_error(
          "publish rates and depth_scale must be positive; "
          "n_frames must be > 1; motion values must be finite");
    }

    auto qos = rclcpp::SensorDataQoS();
    pub_color_ =
        create_publisher<sensor_msgs::msg::Image>(color_topic, qos);
    pub_depth_ =
        create_publisher<sensor_msgs::msg::Image>(depth_topic, qos);
    pub_color_info_ =
        create_publisher<sensor_msgs::msg::CameraInfo>(color_info_topic, qos);
    pub_depth_info_ =
        create_publisher<sensor_msgs::msg::CameraInfo>(depth_info_topic, qos);
    SetUpRenderer();
    color_info_ = MakeCameraInfo(intrinsics_, camera_frame_);
    depth_info_ = MakeCameraInfo(intrinsics_, camera_frame_);

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

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>{1.0 / publish_rate_});
    timer_ = create_wall_timer(period, [this]() { PublishFrame(); });
    RCLCPP_INFO(get_logger(),
                "online synthetic RGB-D | body=%s frames=%d rate=%.1f Hz "
                "GT=%.1f Hz motion=%s appearance=%s depth_noise=%.4f "
                "distortion=%.3f",
                body_->name().c_str(), n_frames_, publish_rate_,
                gt_publish_rate_, motion_mode_.c_str(),
                textured_renderer_ ? "texture" : "surface-normal",
                depth_noise_, distortion_);
  }

 private:
  static Eigen::Vector3f ParseVector3(
      const std::vector<double> &values, const std::string &parameter_name) {
    if (values.size() != 3) {
      throw std::runtime_error(
          parameter_name + " must contain exactly [x, y, z] or "
          "[roll, pitch, yaw]");
    }
    Eigen::Vector3f result{
        static_cast<float>(values[0]),
        static_cast<float>(values[1]),
        static_cast<float>(values[2])};
    if (!result.allFinite()) {
      throw std::runtime_error(
          parameter_name + " must contain finite values");
    }
    return result;
  }

  void SetUpRenderer() {
    body_ = m3t_ros2::DeclareAndCreateBody(this, true);
    renderer_geometry_ =
        std::make_shared<m3t::RendererGeometry>("synthetic_renderer_geometry");
    renderer_geometry_->AddBody(body_);
    renderer_ = std::make_shared<m3t::FullNormalRenderer>(
        "synthetic_renderer", renderer_geometry_,
        m3t::Transform3fA::Identity(), intrinsics_, 0.1f, 5.0f);
    if (!body_->SetUp() || !renderer_geometry_->SetUp() ||
        !renderer_->SetUp()) {
      throw std::runtime_error("failed to set up synthetic M3T renderer");
    }
    if (!texture_path_.empty()) {
      textured_renderer_ =
          std::make_shared<m3t_ros2::FullTexturedRenderer>(
              "synthetic_texture_renderer", renderer_geometry_, body_,
              texture_path_, m3t::Transform3fA::Identity(), intrinsics_,
              0.1f, 5.0f);
      if (!textured_renderer_->SetUp()) {
        throw std::runtime_error(
            "failed to set up synthetic texture renderer");
      }
    }
    if (body_->vertices().empty()) {
      throw std::runtime_error("body mesh contains no vertices");
    }

    Eigen::Vector3f lower = body_->vertices().front();
    Eigen::Vector3f upper = lower;
    for (const auto &vertex : body_->vertices()) {
      lower = lower.cwiseMin(vertex);
      upper = upper.cwiseMax(vertex);
    }
    mesh_center_ = 0.5f * (lower + upper);
    mesh_center_in_body_ =
        body_->geometry2body_pose() * mesh_center_;
    const float diagonal = (upper - lower).norm();
    viewing_distance_ =
        intrinsics_.fu * diagonal / (0.45f * intrinsics_.height);
    if (!has_translation_amplitude_) {
      translation_amplitude_ =
          viewing_distance_ * Eigen::Vector3f{0.15f, 0.10f, 0.10f};
    }
    if (!has_gt_initial_pose_) {
      gt_initial_pose_ = m3t::Transform3fA::Identity();
      const float initial_y =
          motion_mode_ == "six_dof_sine"
              ? 0.0f
              : translation_amplitude_.y();
      gt_initial_pose_.translation() =
          Eigen::Vector3f{0.0f, initial_y, viewing_distance_} -
          mesh_center_in_body_;
    }

    depth_lut_.resize(65536);
    for (int value = 0; value < 65536; ++value) {
      depth_lut_[value] = renderer_->Depth(static_cast<ushort>(value));
    }

    if (distortion_ != 0.0) {
      map_x_.create(intrinsics_.height, intrinsics_.width, CV_32F);
      map_y_.create(intrinsics_.height, intrinsics_.width, CV_32F);
      for (int y = 0; y < intrinsics_.height; ++y) {
        for (int x = 0; x < intrinsics_.width; ++x) {
          const float xn = (x - intrinsics_.ppu) / intrinsics_.fu;
          const float yn = (y - intrinsics_.ppv) / intrinsics_.fv;
          const float scale =
              1.0f - static_cast<float>(distortion_) * (xn * xn + yn * yn);
          map_x_.at<float>(y, x) =
              xn * scale * intrinsics_.fu + intrinsics_.ppu;
          map_y_.at<float>(y, x) =
              yn * scale * intrinsics_.fv + intrinsics_.ppv;
        }
      }
    }

    RCLCPP_INFO(get_logger(),
                "mesh diagonal %.4f m, synthetic viewing distance %.4f m",
                diagonal, viewing_distance_);
  }

  m3t::Transform3fA OrbitPoseForFrame(std::uint64_t frame) const {
    const double phase =
        2.0 * static_cast<double>(kPi) * static_cast<double>(frame) /
        static_cast<double>(n_frames_);
    const float spin =
        static_cast<float>(std::remainder(
            phase * spin_turns_, 2.0 * static_cast<double>(kPi)));
    const float nod =
        static_cast<float>(nod_degrees_) * kPi / 180.0f *
        static_cast<float>(std::sin(phase));
    const float sin_phase = static_cast<float>(std::sin(phase));
    const float cos_phase = static_cast<float>(std::cos(phase));
    const Eigen::Matrix3f motion_rotation =
        (Eigen::AngleAxisf(nod, Eigen::Vector3f::UnitX()) *
         Eigen::AngleAxisf(spin, Eigen::Vector3f::UnitZ()))
            .toRotationMatrix();
    const Eigen::Vector3f initial_center =
        gt_initial_pose_ * mesh_center_in_body_;
    const Eigen::Vector3f translation_offset{
        translation_amplitude_.x() * sin_phase,
        translation_amplitude_.y() * (cos_phase - 1.0f),
        translation_amplitude_.z() * sin_phase};

    m3t::Transform3fA pose{m3t::Transform3fA::Identity()};
    pose.linear() = gt_initial_pose_.rotation() * motion_rotation;
    pose.translation() =
        initial_center + translation_offset -
        pose.rotation() * mesh_center_in_body_;
    return pose;
  }

  m3t::Transform3fA SixDofSinePoseForFrame(std::uint64_t frame) const {
    const double time =
        static_cast<double>(frame) / publish_rate_;
    Eigen::Vector3f translation_offset = Eigen::Vector3f::Zero();
    Eigen::Vector3f rpy = Eigen::Vector3f::Zero();
    for (int axis = 0; axis < 3; ++axis) {
      const double translation_argument =
          2.0 * static_cast<double>(kPi) *
              static_cast<double>(translation_sine_frequency_hz_[axis]) *
              time +
          static_cast<double>(translation_sine_phase_rad_[axis]);
      translation_offset[axis] =
          translation_sine_amplitude_m_[axis] *
          static_cast<float>(
              std::sin(translation_argument) -
              std::sin(
                  static_cast<double>(translation_sine_phase_rad_[axis])));

      const double rotation_argument =
          2.0 * static_cast<double>(kPi) *
              static_cast<double>(rotation_sine_frequency_hz_[axis]) * time +
          static_cast<double>(rotation_sine_phase_rad_[axis]);
      rpy[axis] =
          rotation_sine_amplitude_rad_[axis] *
          static_cast<float>(
              std::sin(rotation_argument) -
              std::sin(static_cast<double>(rotation_sine_phase_rad_[axis])));
    }

    if (translation_frame_ == "body") {
      translation_offset =
          gt_initial_pose_.rotation() * translation_offset;
    }

    // ZYX convention: R_delta = Rz(yaw) * Ry(pitch) * Rx(roll).
    const Eigen::Matrix3f delta_rotation =
        (Eigen::AngleAxisf(rpy.z(), Eigen::Vector3f::UnitZ()) *
         Eigen::AngleAxisf(rpy.y(), Eigen::Vector3f::UnitY()) *
         Eigen::AngleAxisf(rpy.x(), Eigen::Vector3f::UnitX()))
            .toRotationMatrix();
    const Eigen::Matrix3f rotation =
        rotation_frame_ == "body"
            ? gt_initial_pose_.rotation() * delta_rotation
            : delta_rotation * gt_initial_pose_.rotation();

    m3t::Transform3fA pose{m3t::Transform3fA::Identity()};
    pose.linear() = rotation;
    if (rotation_pivot_ == "geometry_center") {
      const Eigen::Vector3f initial_center =
          gt_initial_pose_ * mesh_center_in_body_;
      pose.translation() =
          initial_center + translation_offset -
          rotation * mesh_center_in_body_;
    } else {
      pose.translation() =
          gt_initial_pose_.translation() + translation_offset;
    }
    return pose;
  }

  m3t::Transform3fA PoseForFrame(std::uint64_t frame) const {
    if (motion_mode_ == "static") return gt_initial_pose_;
    if (motion_mode_ == "six_dof_sine") {
      return SixDofSinePoseForFrame(frame);
    }
    return OrbitPoseForFrame(frame);
  }

  void PublishFrame() {
    if (!loop_ &&
        frame_index_ >= static_cast<std::uint64_t>(n_frames_)) {
      timer_->cancel();
      RCLCPP_INFO(get_logger(), "synthetic sequence complete");
      return;
    }

    const m3t::Transform3fA pose = PoseForFrame(frame_index_);
    body_->set_body2world_pose(pose);
    if (!renderer_->StartRendering() || !renderer_->FetchNormalImage() ||
        !renderer_->FetchDepthImage() ||
        (textured_renderer_ &&
         (!textured_renderer_->StartRendering() ||
          !textured_renderer_->FetchColorImage()))) {
      RCLCPP_ERROR(
          get_logger(), "rendering failed at frame %llu",
          static_cast<unsigned long long>(frame_index_));
      return;
    }

    cv::Mat color;
    if (textured_renderer_) {
      color = textured_renderer_->color_image();
    } else {
      cv::cvtColor(renderer_->normal_image(), color, cv::COLOR_BGRA2BGR);
    }
    const cv::Mat &normal = renderer_->normal_image();
    const cv::Mat &raw_depth = renderer_->depth_image();
    cv::Mat depth(intrinsics_.height, intrinsics_.width, CV_16U,
                  cv::Scalar{0});
    for (int y = 0; y < intrinsics_.height; ++y) {
      const auto *normal_row = normal.ptr<cv::Vec4b>(y);
      const auto *raw_depth_row = raw_depth.ptr<ushort>(y);
      auto *depth_row = depth.ptr<ushort>(y);
      for (int x = 0; x < intrinsics_.width; ++x) {
        if (normal_row[x][3] == 0) continue;
        float z = depth_lut_[raw_depth_row[x]];
        if (depth_noise_ > 0.0) {
          z += static_cast<float>(depth_noise_) * z * z * gaussian_(rng_);
        }
        const int scaled =
            static_cast<int>(std::lround(z / depth_scale_));
        depth_row[x] = static_cast<ushort>(
            std::min(std::max(scaled, 0), 65535));
      }
    }

    if (distortion_ != 0.0) {
      cv::Mat distorted_color;
      cv::Mat distorted_depth;
      cv::remap(color, distorted_color, map_x_, map_y_, cv::INTER_LINEAR,
                cv::BORDER_CONSTANT, cv::Scalar{0, 0, 0});
      cv::remap(depth, distorted_depth, map_x_, map_y_, cv::INTER_NEAREST,
                cv::BORDER_CONSTANT, cv::Scalar{0});
      color = distorted_color;
      depth = distorted_depth;
    }

    const rclcpp::Time stamp = now();
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = camera_frame_;
    color_info_.header.stamp = stamp;
    depth_info_.header.stamp = stamp;

    // Publish GT and depth before color.  The tracker treats color as the frame
    // trigger, so this ordering also gives non-synchronizing subscribers the
    // newest depth/pose before the corresponding color image arrives.
    gt_publisher_->Update(stamp, pose);
    pub_color_info_->publish(color_info_);
    pub_depth_info_->publish(depth_info_);
    pub_depth_->publish(
        *cv_bridge::CvImage(header, "16UC1", depth).toImageMsg());
    pub_color_->publish(
        *cv_bridge::CvImage(header, "bgr8", color).toImageMsg());
    ++frame_index_;
  }

  std::string world_frame_;
  std::string camera_frame_;
  std::string gt_frame_;
  std::string mesh_resource_;
  std::string texture_path_;
  std::string motion_mode_{"orbit"};
  std::string translation_frame_{"world"};
  std::string rotation_frame_{"body"};
  std::string rotation_pivot_{"geometry_center"};
  double publish_rate_{30.0};
  double gt_publish_rate_{60.0};
  int n_frames_{240};
  bool loop_{true};
  double depth_noise_{0.0};
  double distortion_{0.0};
  double depth_scale_{0.001};
  double spin_turns_{1.0};
  double nod_degrees_{25.0};
  double mesh_scale_{1.0};
  bool mesh_embedded_{false};
  bool has_gt_initial_pose_{false};
  bool has_translation_amplitude_{false};
  std::uint64_t frame_index_{0};

  m3t::Intrinsics intrinsics_{};
  Eigen::Vector3f mesh_center_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f mesh_center_in_body_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f translation_amplitude_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f translation_sine_amplitude_m_{
      Eigen::Vector3f::Zero()};
  Eigen::Vector3f translation_sine_frequency_hz_{
      Eigen::Vector3f::Zero()};
  Eigen::Vector3f translation_sine_phase_rad_{
      Eigen::Vector3f::Zero()};
  Eigen::Vector3f rotation_sine_amplitude_rad_{
      Eigen::Vector3f::Zero()};
  Eigen::Vector3f rotation_sine_frequency_hz_{
      Eigen::Vector3f::Zero()};
  Eigen::Vector3f rotation_sine_phase_rad_{
      Eigen::Vector3f::Zero()};
  m3t::Transform3fA gt_initial_pose_{m3t::Transform3fA::Identity()};
  float viewing_distance_{0.5f};
  std::shared_ptr<m3t::Body> body_;
  std::shared_ptr<m3t::RendererGeometry> renderer_geometry_;
  std::shared_ptr<m3t::FullNormalRenderer> renderer_;
  std::shared_ptr<m3t_ros2::FullTexturedRenderer> textured_renderer_;
  std::vector<float> depth_lut_;
  cv::Mat map_x_;
  cv::Mat map_y_;
  std::mt19937 rng_;
  std::normal_distribution<float> gaussian_{0.0f, 1.0f};

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
    rclcpp::spin(std::make_shared<SyntheticSourceNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("m3t_synthetic_source"), "%s",
                 error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}