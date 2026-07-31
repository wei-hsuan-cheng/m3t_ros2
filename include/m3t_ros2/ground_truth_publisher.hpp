// SPDX-License-Identifier: MIT

#ifndef M3T_ROS2_GROUND_TRUTH_PUBLISHER_HPP_
#define M3T_ROS2_GROUND_TRUTH_PUBLISHER_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <Eigen/Geometry>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <m3t/common.h>

namespace m3t_ros2 {

struct GroundTruthPublisherConfig {
  std::string world_frame{"camera"};
  std::string gt_frame{"object_gt"};
  std::string pose_topic{"/m3t/pose_gt"};
  std::string marker_topic{"/m3t/marker_gt"};
  std::string mesh_resource;
  float mesh_scale{1.0f};
  bool mesh_use_embedded_materials{false};
  double publish_rate{60.0};
  m3t::Transform3fA geometry2body_pose{m3t::Transform3fA::Identity()};
};

// Keeps the latest image-aligned GT pose available as a dynamic TF and mesh
// marker even when rendering or image I/O delays the next source frame.
class GroundTruthPublisher {
 public:
  GroundTruthPublisher(rclcpp::Node *node,
                       GroundTruthPublisherConfig config)
      : node_{node}, config_{std::move(config)} {
    if (config_.publish_rate <= 0.0) {
      throw std::runtime_error("gt_publish_rate must be positive");
    }

    pose_publisher_ =
        node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            config_.pose_topic, 10);
    const auto marker_qos =
        rclcpp::QoS{rclcpp::KeepLast(1)}.reliable().transient_local();
    marker_publisher_ =
        node_->create_publisher<visualization_msgs::msg::Marker>(
            config_.marker_topic, marker_qos);
    tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    publish_thread_ = std::thread{[this]() { PublishLoop(); }};
  }

  ~GroundTruthPublisher() {
    {
      std::lock_guard<std::mutex> lock{wait_mutex_};
      running_ = false;
    }
    wait_condition_.notify_all();
    if (publish_thread_.joinable()) publish_thread_.join();
  }

  GroundTruthPublisher(const GroundTruthPublisher &) = delete;
  GroundTruthPublisher &operator=(const GroundTruthPublisher &) = delete;

  // The pose topic retains the exact image timestamp. TF and the marker are
  // also sent immediately, then refreshed at gt_publish_rate using the latest
  // pose and the current ROS time.
  void Update(const rclcpp::Time &stamp,
              const m3t::Transform3fA &body2world_pose) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = config_.world_frame;
    pose.pose = ToPose(body2world_pose);
    pose_publisher_->publish(pose);

    {
      std::lock_guard<std::mutex> lock{pose_mutex_};
      latest_body2world_pose_ = body2world_pose;
      has_pose_ = true;
    }
    PublishVisualization(stamp, body2world_pose);
  }

 private:
  static geometry_msgs::msg::Pose ToPose(
      const m3t::Transform3fA &transform) {
    geometry_msgs::msg::Pose pose;
    const Eigen::Vector3f translation = transform.translation();
    const Eigen::Quaternionf rotation{transform.rotation()};
    pose.position.x = translation.x();
    pose.position.y = translation.y();
    pose.position.z = translation.z();
    pose.orientation.x = rotation.x();
    pose.orientation.y = rotation.y();
    pose.orientation.z = rotation.z();
    pose.orientation.w = rotation.w();
    return pose;
  }

  void PublishLoop() {
    const auto period = std::chrono::duration<double>{
        1.0 / config_.publish_rate};
    std::unique_lock<std::mutex> wait_lock{wait_mutex_};
    while (running_) {
      if (wait_condition_.wait_for(
              wait_lock, period, [this]() { return !running_; })) {
        break;
      }
      wait_lock.unlock();
      if (!rclcpp::ok()) {
        wait_lock.lock();
        break;
      }
      try {
        PublishLatestVisualization();
      } catch (const std::exception &error) {
        if (rclcpp::ok()) {
          RCLCPP_ERROR(node_->get_logger(),
                       "stopping GT publisher: %s", error.what());
        }
        wait_lock.lock();
        break;
      }
      wait_lock.lock();
    }
  }

  void PublishLatestVisualization() {
    m3t::Transform3fA body2world_pose;
    {
      std::lock_guard<std::mutex> lock{pose_mutex_};
      if (!has_pose_) return;
      body2world_pose = latest_body2world_pose_;
    }
    PublishVisualization(node_->now(), body2world_pose);
  }

  void PublishVisualization(
      const rclcpp::Time &stamp,
      const m3t::Transform3fA &body2world_pose) {
    std::lock_guard<std::mutex> lock{publication_mutex_};

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = config_.world_frame;
    transform.child_frame_id = config_.gt_frame;
    const auto body_pose = ToPose(body2world_pose);
    transform.transform.translation.x = body_pose.position.x;
    transform.transform.translation.y = body_pose.position.y;
    transform.transform.translation.z = body_pose.position.z;
    transform.transform.rotation = body_pose.orientation;
    tf_broadcaster_->sendTransform(transform);

    visualization_msgs::msg::Marker marker;
    marker.header = transform.header;
    marker.ns = "gt";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.mesh_resource = config_.mesh_resource;
    marker.mesh_use_embedded_materials =
        config_.mesh_use_embedded_materials;
    marker.pose =
        ToPose(body2world_pose * config_.geometry2body_pose);
    marker.scale.x = marker.scale.y = marker.scale.z = config_.mesh_scale;
    if (config_.mesh_use_embedded_materials) {
      // All-zero color asks RViz to preserve the OBJ/MTL appearance.
      marker.color.r = marker.color.g = marker.color.b = marker.color.a = 0.0f;
    } else {
      marker.color.r = 0.55f;
      marker.color.g = 0.55f;
      marker.color.b = 0.55f;
      marker.color.a = 0.7f;
    }
    marker_publisher_->publish(marker);
  }

  rclcpp::Node *node_;
  GroundTruthPublisherConfig config_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      pose_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      marker_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::mutex pose_mutex_;
  m3t::Transform3fA latest_body2world_pose_{
      m3t::Transform3fA::Identity()};
  bool has_pose_{false};

  std::mutex publication_mutex_;
  std::mutex wait_mutex_;
  std::condition_variable wait_condition_;
  bool running_{true};
  std::thread publish_thread_;
};

}  // namespace m3t_ros2

#endif  // M3T_ROS2_GROUND_TRUTH_PUBLISHER_HPP_