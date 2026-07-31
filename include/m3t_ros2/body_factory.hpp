// SPDX-License-Identifier: MIT
// Construct an M3T Body from ROS parameters instead of an M3T-specific YAML.

#ifndef M3T_ROS2_BODY_FACTORY_HPP_
#define M3T_ROS2_BODY_FACTORY_HPP_

#include <rclcpp/rclcpp.hpp>

#include <m3t/body.h>

#include <Eigen/Geometry>

#include <cmath>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace m3t_ros2 {

inline m3t::Transform3fA TransformFromRowMajor(
    const std::vector<double> &values, const std::string &parameter_name) {
  if (values.size() != 16) {
    throw std::runtime_error(parameter_name +
                             " must contain 16 row-major values");
  }
  Eigen::Matrix4f matrix;
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) {
      matrix(row, column) =
          static_cast<float>(values[static_cast<size_t>(row * 4 + column)]);
    }
  }
  return m3t::Transform3fA{matrix};
}

inline m3t::Transform3fA TransformFromPose(
    const std::vector<double> &values, const std::string &parameter_name) {
  if (values.size() != 6 && values.size() != 7) {
    throw std::runtime_error(
        parameter_name +
        " must be [x, y, z, roll, pitch, yaw] (radians) or "
        "[x, y, z, qx, qy, qz, qw]");
  }
  for (const double value : values) {
    if (!std::isfinite(value)) {
      throw std::runtime_error(parameter_name +
                               " must contain only finite values");
    }
  }

  m3t::Transform3fA transform{m3t::Transform3fA::Identity()};
  transform.translation() =
      Eigen::Vector3f{static_cast<float>(values[0]),
                      static_cast<float>(values[1]),
                      static_cast<float>(values[2])};
  if (!transform.translation().allFinite()) {
    throw std::runtime_error(parameter_name +
                             " translation is outside float range");
  }

  if (values.size() == 6) {
    const float roll = static_cast<float>(values[3]);
    const float pitch = static_cast<float>(values[4]);
    const float yaw = static_cast<float>(values[5]);
    if (!std::isfinite(roll) || !std::isfinite(pitch) ||
        !std::isfinite(yaw)) {
      throw std::runtime_error(parameter_name +
                               " RPY angles are outside float range");
    }
    transform.linear() =
        (Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
         Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
         Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()))
            .toRotationMatrix();
    return transform;
  }

  Eigen::Quaternionf quaternion{
      static_cast<float>(values[6]), static_cast<float>(values[3]),
      static_cast<float>(values[4]), static_cast<float>(values[5])};
  if (!quaternion.coeffs().allFinite()) {
    throw std::runtime_error(parameter_name +
                             " quaternion is outside float range");
  }
  if (quaternion.squaredNorm() <= 1.0e-12f) {
    throw std::runtime_error(parameter_name +
                             " quaternion must have non-zero norm");
  }
  quaternion.normalize();
  transform.linear() = quaternion.toRotationMatrix();
  return transform;
}

inline std::shared_ptr<m3t::Body> DeclareAndCreateBody(
    rclcpp::Node *node, bool force_disable_culling = false) {
  const auto object_name =
      node->declare_parameter<std::string>("object_name", "object");
  const auto geometry_path =
      node->declare_parameter<std::string>("geometry_path", "");
  const float geometry_unit = static_cast<float>(
      node->declare_parameter<double>("geometry_unit_in_meter", 1.0));
  const bool counterclockwise =
      node->declare_parameter<bool>("geometry_counterclockwise", true);
  bool enable_culling =
      node->declare_parameter<bool>("geometry_enable_culling", false);
  const auto geometry2body_values = node->declare_parameter<std::vector<double>>(
      "geometry2body_pose",
      {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
       0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0});
  const int64_t body_id = node->declare_parameter<int64_t>("body_id", 1);
  const int64_t region_id = node->declare_parameter<int64_t>("region_id", 1);

  if (geometry_path.empty()) {
    throw std::runtime_error("geometry_path is required");
  }
  if (!std::filesystem::exists(geometry_path)) {
    throw std::runtime_error("geometry_path does not exist: " + geometry_path);
  }
  if (body_id < 1 || body_id > 255 || region_id < 1 || region_id > 255) {
    throw std::runtime_error("body_id and region_id must be within [1, 255]");
  }
  if (force_disable_culling) enable_culling = false;

  auto body = std::make_shared<m3t::Body>(
      object_name, geometry_path, geometry_unit, counterclockwise,
      enable_culling,
      TransformFromRowMajor(geometry2body_values, "geometry2body_pose"));
  body->set_body_id(static_cast<uchar>(body_id));
  body->set_region_id(static_cast<uchar>(region_id));
  return body;
}

}  // namespace m3t_ros2

#endif  // M3T_ROS2_BODY_FACTORY_HPP_