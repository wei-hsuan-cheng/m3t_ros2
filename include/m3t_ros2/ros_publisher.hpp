// SPDX-License-Identifier: MIT
// RosPublisher: immutable-snapshot ROS output, called only by the dedicated
// publisher thread. The tracking thread is never blocked by ROS serialization
// or keypoint extraction. Optional image publishers are created only when
// requested; TF and the estimate marker remain independent of image output.

#ifndef M3T_ROS2_ROS_PUBLISHER_HPP_
#define M3T_ROS2_ROS_PUBLISHER_HPP_

#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/features2d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <m3t/body.h>
#include <m3t/common.h>

namespace m3t_ros2 {

struct RosPublisherConfig {
  std::string world_frame{"camera"};
  std::string mesh_resource;
  float mesh_scale{1.0f};
  bool mesh_use_embedded_materials{false};
  bool publish_color{false};
  bool publish_depth{false};
  bool publish_overlay{false};
  bool publish_gt{false};
  bool publish_keypoints{false};
};

// One consistent frame handed from the tracking thread to the publisher.
struct Snapshot {
  bool valid{false};
  bool has_depth{false};
  bool has_gt{false};
  cv_bridge::CvImageConstPtr color_owner, depth_owner;
  cv::Mat color, depth, overlay;
  m3t::Transform3fA body2world_est, geometry2world_est;
  m3t::Transform3fA body2world_gt, geometry2world_gt;
};

class RosPublisher {
 public:
  RosPublisher(rclcpp::Node *node, const RosPublisherConfig &cfg,
               std::shared_ptr<const m3t::Body> body)
      : node_{node}, cfg_{cfg}, body_{std::move(body)} {
    auto qos = rclcpp::SensorDataQoS();
    if (cfg_.publish_color)
      pub_color_ = node_->create_publisher<sensor_msgs::msg::Image>(
          "~/color/image_raw", qos);
    if (cfg_.publish_depth)
      pub_depth_ = node_->create_publisher<sensor_msgs::msg::Image>(
          "~/depth/image_raw", qos);
    if (cfg_.publish_overlay)
      pub_overlay_ = node_->create_publisher<sensor_msgs::msg::Image>(
          "~/overlay/image", qos);
    if (cfg_.publish_keypoints) {
      pub_keypoints_ = node_->create_publisher<sensor_msgs::msg::Image>(
          "~/keypoints/image", qos);
      orb_ = cv::ORB::create(500);
    }
    pub_marker_est_ = node_->create_publisher<visualization_msgs::msg::Marker>("~/marker_est", 1);
    if (cfg_.publish_gt)
      pub_marker_gt_ = node_->create_publisher<visualization_msgs::msg::Marker>(
          "~/marker_gt", 1);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  }

  void Publish(const Snapshot &s, bool publish_images) {
    if (!s.valid) return;
    const rclcpp::Time stamp = node_->now();
    std_msgs::msg::Header h;
    h.stamp = stamp;
    h.frame_id = cfg_.world_frame;

    if (publish_images) {
      if (cfg_.publish_color && !s.color.empty())
        pub_color_->publish(
            *cv_bridge::CvImage(h, "bgr8", s.color).toImageMsg());
      if (cfg_.publish_depth && s.has_depth && !s.depth.empty())
        pub_depth_->publish(
            *cv_bridge::CvImage(h, "16UC1", s.depth).toImageMsg());
      if (cfg_.publish_overlay && !s.overlay.empty())
        pub_overlay_->publish(
            *cv_bridge::CvImage(h, "bgr8", s.overlay).toImageMsg());
      if (cfg_.publish_keypoints && !s.color.empty()) {
        std::vector<cv::KeyPoint> kp;
        orb_->detect(s.color, kp);
        cv::Mat kimg;
        cv::drawKeypoints(s.color, kp, kimg, cv::Scalar(0, 255, 0));
        pub_keypoints_->publish(
            *cv_bridge::CvImage(h, "bgr8", kimg).toImageMsg());
      }
    }

    BroadcastTf(stamp, "object_est", s.body2world_est);
    PublishEstimateMarker(s, stamp);
    if (cfg_.publish_gt && s.has_gt) {
      BroadcastTf(stamp, "object_gt", s.body2world_gt);
      PublishMeshResourceMarker(
          pub_marker_gt_, stamp, "gt", s.geometry2world_gt,
          cfg_.mesh_use_embedded_materials, 0.55f, 0.55f, 0.55f, 0.7f);
    }
  }

 private:
  static constexpr auto kNormalMarkerPeriod =
      std::chrono::milliseconds{100};

  static geometry_msgs::msg::Pose ToPose(const m3t::Transform3fA &t) {
    geometry_msgs::msg::Pose p;
    const Eigen::Vector3f tr = t.translation();
    const Eigen::Quaternionf q{t.rotation()};
    p.position.x = tr.x(); p.position.y = tr.y(); p.position.z = tr.z();
    p.orientation.x = q.x(); p.orientation.y = q.y();
    p.orientation.z = q.z(); p.orientation.w = q.w();
    return p;
  }

  void BroadcastTf(const rclcpp::Time &stamp, const std::string &child,
                   const m3t::Transform3fA &t) {
    geometry_msgs::msg::TransformStamped m;
    m.header.stamp = stamp;
    m.header.frame_id = cfg_.world_frame;
    m.child_frame_id = child;
    const auto p = ToPose(t);
    m.transform.translation.x = p.position.x;
    m.transform.translation.y = p.position.y;
    m.transform.translation.z = p.position.z;
    m.transform.rotation = p.orientation;
    tf_->sendTransform(m);
  }

  bool PrepareNormalMarkerGeometry() {
    if (normal_marker_geometry_ready_) return true;
    if (!body_ || !body_->set_up()) return false;

    const auto &vertices = body_->vertices();
    const auto &triangles = body_->mesh_indices();
    normal_marker_points_.clear();
    normal_marker_normals_.clear();
    normal_marker_points_.reserve(3 * triangles.size());
    normal_marker_normals_.reserve(3 * triangles.size());

    for (const auto &triangle : triangles) {
      if (triangle[0] < 0 || triangle[1] < 0 || triangle[2] < 0 ||
          static_cast<size_t>(triangle[0]) >= vertices.size() ||
          static_cast<size_t>(triangle[1]) >= vertices.size() ||
          static_cast<size_t>(triangle[2]) >= vertices.size()) {
        continue;
      }
      const Eigen::Vector3f &p0 = vertices[triangle[0]];
      const Eigen::Vector3f &p1 = vertices[triangle[1]];
      const Eigen::Vector3f &p2 = vertices[triangle[2]];
      Eigen::Vector3f normal = (p2 - p1).cross(p0 - p1);
      const float squared_norm = normal.squaredNorm();
      if (squared_norm <= 1.0e-16f) continue;
      normal /= std::sqrt(squared_norm);

      for (const Eigen::Vector3f *point : {&p0, &p1, &p2}) {
        geometry_msgs::msg::Point message_point;
        message_point.x = point->x();
        message_point.y = point->y();
        message_point.z = point->z();
        normal_marker_points_.push_back(message_point);
        normal_marker_normals_.push_back(normal);
      }
    }

    normal_marker_geometry_ready_ = !normal_marker_points_.empty();
    if (!normal_marker_geometry_ready_) {
      RCLCPP_WARN_ONCE(
          node_->get_logger(),
          "could not build estimate normal marker; using solid mesh");
    }
    return normal_marker_geometry_ready_;
  }

  void PublishEstimateMarker(const Snapshot &snapshot,
                             const rclcpp::Time &stamp) {
    const auto now = std::chrono::steady_clock::now();
    if (normal_marker_published_ &&
        now - last_normal_marker_publish_ < kNormalMarkerPeriod) {
      return;
    }
    if (!PrepareNormalMarkerGeometry()) {
      PublishMeshResourceMarker(
          pub_marker_est_, stamp, "est", snapshot.geometry2world_est, false,
          1.0f, 0.1f, 0.1f, 0.9f);
      return;
    }

    visualization_msgs::msg::Marker marker;
    // TF carries the high-rate body pose. The large TRIANGLE_LIST only needs
    // republishing when its camera-frame normal colors are refreshed.
    marker.header.frame_id = "object_est";
    marker.header.stamp = stamp;
    marker.ns = "est_normal";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.frame_locked = true;
    marker.pose = ToPose(
        snapshot.body2world_est.inverse() * snapshot.geometry2world_est);
    marker.scale.x = marker.scale.y = marker.scale.z = cfg_.mesh_scale;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 0.9f;
    marker.points = normal_marker_points_;
    marker.colors.resize(normal_marker_normals_.size());

    const Eigen::Matrix3f geometry2world_rotation =
        snapshot.geometry2world_est.rotation();
    for (size_t i = 0; i < normal_marker_normals_.size(); ++i) {
      const Eigen::Vector3f normal =
          geometry2world_rotation * normal_marker_normals_[i];
      auto &color = marker.colors[i];
      // Match M3T NormalRendererCore:
      // FragColor = vec4(0.5 - 0.5 * Normal, 1.0).zyxw.
      color.r = std::clamp(0.5f - 0.5f * normal.z(), 0.0f, 1.0f);
      color.g = std::clamp(0.5f - 0.5f * normal.y(), 0.0f, 1.0f);
      color.b = std::clamp(0.5f - 0.5f * normal.x(), 0.0f, 1.0f);
      color.a = 0.9f;
    }

    pub_marker_est_->publish(marker);
    normal_marker_published_ = true;
    last_normal_marker_publish_ = now;
  }

  void PublishMeshResourceMarker(
      const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub,
      const rclcpp::Time &stamp, const std::string &ns,
      const m3t::Transform3fA &pose, bool use_embedded_materials,
      float r, float g, float b, float a) {
    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = cfg_.world_frame;
    mk.header.stamp = stamp;
    mk.ns = ns;
    mk.id = 0;
    mk.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    mk.action = visualization_msgs::msg::Marker::ADD;
    mk.mesh_resource = cfg_.mesh_resource;
    mk.mesh_use_embedded_materials = use_embedded_materials;
    mk.pose = ToPose(pose);
    mk.scale.x = mk.scale.y = mk.scale.z = cfg_.mesh_scale;
    if (use_embedded_materials) {
      // RViz uses all-zero marker color as the sentinel for the mesh's
      // original material colors instead of multiplying them by a tint.
      mk.color.r = mk.color.g = mk.color.b = mk.color.a = 0.0f;
    } else {
      mk.color.r = r;
      mk.color.g = g;
      mk.color.b = b;
      mk.color.a = a;
    }
    pub->publish(mk);
  }

  rclcpp::Node *node_;
  RosPublisherConfig cfg_;
  std::shared_ptr<const m3t::Body> body_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_color_, pub_depth_,
      pub_overlay_, pub_keypoints_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_est_,
      pub_marker_gt_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  cv::Ptr<cv::ORB> orb_;
  std::vector<geometry_msgs::msg::Point> normal_marker_points_;
  std::vector<Eigen::Vector3f> normal_marker_normals_;
  bool normal_marker_geometry_ready_{false};
  bool normal_marker_published_{false};
  std::chrono::steady_clock::time_point last_normal_marker_publish_{};
};

}  // namespace m3t_ros2

#endif  // M3T_ROS2_ROS_PUBLISHER_HPP_