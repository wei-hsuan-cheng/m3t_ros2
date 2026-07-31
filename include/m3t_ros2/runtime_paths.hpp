// SPDX-License-Identifier: MIT
// Runtime path helpers for generated M3T data.

#ifndef M3T_ROS2_RUNTIME_PATHS_HPP_
#define M3T_ROS2_RUNTIME_PATHS_HPP_

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

namespace m3t_ros2 {

namespace fs = std::filesystem;

inline fs::path DefaultRuntimeRoot() {
  if (const char *ros_home = std::getenv("ROS_HOME");
      ros_home && ros_home[0] != '\0') {
    return fs::path{ros_home} / "m3t";
  }
  if (const char *home = std::getenv("HOME"); home && home[0] != '\0') {
    return fs::path{home} / ".ros" / "m3t";
  }
  return fs::temp_directory_path() / "m3t";
}

inline std::string SanitizePathComponent(std::string value) {
  for (char &c : value) {
    const bool valid = (c >= 'a' && c <= 'z') ||
                       (c >= 'A' && c <= 'Z') ||
                       (c >= '0' && c <= '9') || c == '-' || c == '_';
    if (!valid) c = '_';
  }
  return value.empty() ? "object" : value;
}

inline bool EnsureWritableDirectory(const fs::path &directory,
                                    std::string *error) {
  std::error_code ec;
  fs::create_directories(directory, ec);
  if (ec) {
    if (error) {
      *error = "cannot create " + directory.string() + ": " + ec.message();
    }
    return false;
  }

  const auto suffix =
      std::chrono::steady_clock::now().time_since_epoch().count();
  const fs::path probe =
      directory / (".m3t_write_test_" + std::to_string(suffix));
  std::ofstream stream{probe};
  if (!stream.is_open()) {
    if (error) *error = "directory is not writable: " + directory.string();
    return false;
  }
  stream << "ok\n";
  stream.close();
  fs::remove(probe, ec);
  return true;
}

}  // namespace m3t_ros2

#endif  // M3T_ROS2_RUNTIME_PATHS_HPP_
