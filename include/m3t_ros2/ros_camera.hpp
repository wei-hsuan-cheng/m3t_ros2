// SPDX-License-Identifier: MIT
// M3T Camera implementations backed by ROS subscriptions. They let the tracker
// consume images from topics (a real camera node, a bag, or the sequence
// publisher) with no code change in the tracker — only topic names differ.
// The latest cv_bridge owner is stored under a mutex. UpdateImage() retains
// that owner for the complete tracking step and exposes its cv::Mat to M3T.
// This keeps zero-copy ROS image views alive without copying pixel buffers.

#ifndef M3T_ROS2_ROS_CAMERA_HPP_
#define M3T_ROS2_ROS_CAMERA_HPP_

#include <atomic>
#include <cv_bridge/cv_bridge.h>
#include <cstdint>
#include <mutex>
#include <opencv2/core.hpp>

#include <m3t/camera.h>

namespace m3t_ros2 {

class RosColorCamera : public m3t::ColorCamera {
 public:
  explicit RosColorCamera(const std::string &name) : m3t::ColorCamera(name) {}
  bool SetUp() override { set_up_ = true; return true; }
  bool UpdateImage(bool) override {
    std::lock_guard<std::mutex> lk{m_};
    if (!latest_owner_ || latest_owner_->image.empty()) return false;
    current_owner_ = latest_owner_;
    image_ = current_owner_->image;
    return true;
  }
  bool CanAccept(const cv_bridge::CvImageConstPtr &image) const {
    if (!image || image->image.empty()) return false;
    if (!has_intrinsics_.load(std::memory_order_acquire)) return true;
    return image->image.cols == intrinsics_.width &&
           image->image.rows == intrinsics_.height;
  }
  bool SetLatest(cv_bridge::CvImageConstPtr image) {
    if (!CanAccept(image)) return false;
    {
      std::lock_guard<std::mutex> lk{m_};
      latest_owner_ = std::move(image);
    }
    seq_.fetch_add(1, std::memory_order_release);
    return true;
  }
  // Intrinsics are latched from the first CameraInfo message. Camera drivers
  // normally republish identical calibration; latching avoids writing M3T's
  // camera state while the tracking worker is reading it.
  void SetIntrinsics(const m3t::Intrinsics &in) {
    if (has_intrinsics_.load(std::memory_order_acquire)) return;
    std::lock_guard<std::mutex> lk{intrinsics_mutex_};
    if (has_intrinsics_.load(std::memory_order_relaxed)) return;
    intrinsics_ = in;
    has_intrinsics_.store(true, std::memory_order_release);
  }
  bool HasImage() {
    std::lock_guard<std::mutex> lk{m_};
    return latest_owner_ && !latest_owner_->image.empty();
  }
  bool Ready() {
    if (!HasIntrinsics()) return false;
    std::lock_guard<std::mutex> lk{m_};
    return latest_owner_ && CanAccept(latest_owner_);
  }
  bool HasIntrinsics() const {
    return has_intrinsics_.load(std::memory_order_acquire);
  }
  uint64_t seq() const {
    return seq_.load(std::memory_order_acquire);
  }
  cv_bridge::CvImageConstPtr CurrentOwner() {
    std::lock_guard<std::mutex> lk{m_};
    return current_owner_;
  }

 private:
  std::mutex m_;
  std::mutex intrinsics_mutex_;
  cv_bridge::CvImageConstPtr latest_owner_;
  cv_bridge::CvImageConstPtr current_owner_;
  std::atomic<uint64_t> seq_{0};
  std::atomic<bool> has_intrinsics_{false};
};

class RosDepthCamera : public m3t::DepthCamera {
 public:
  explicit RosDepthCamera(const std::string &name) : m3t::DepthCamera(name) {}
  bool SetUp() override { set_up_ = true; return true; }
  bool UpdateImage(bool) override {
    std::lock_guard<std::mutex> lk{m_};
    if (!latest_owner_ || latest_owner_->image.empty()) return false;
    current_owner_ = latest_owner_;
    image_ = current_owner_->image;
    return true;
  }
  bool CanAccept(const cv_bridge::CvImageConstPtr &image) const {
    if (!image || image->image.empty()) return false;
    if (!has_intrinsics_.load(std::memory_order_acquire)) return true;
    return image->image.cols == intrinsics_.width &&
           image->image.rows == intrinsics_.height;
  }
  bool SetLatest(cv_bridge::CvImageConstPtr image) {
    if (!CanAccept(image)) return false;
    std::lock_guard<std::mutex> lk{m_};
    latest_owner_ = std::move(image);
    return true;
  }
  void SetIntrinsics(const m3t::Intrinsics &in) {
    if (has_intrinsics_.load(std::memory_order_acquire)) return;
    std::lock_guard<std::mutex> lk{intrinsics_mutex_};
    if (has_intrinsics_.load(std::memory_order_relaxed)) return;
    intrinsics_ = in;
    has_intrinsics_.store(true, std::memory_order_release);
  }
  void SetDepthScale(float s) { depth_scale_ = s; }
  bool HasImage() {
    std::lock_guard<std::mutex> lk{m_};
    return latest_owner_ && !latest_owner_->image.empty();
  }
  bool Ready() {
    if (!HasIntrinsics()) return false;
    std::lock_guard<std::mutex> lk{m_};
    return latest_owner_ && CanAccept(latest_owner_);
  }
  bool HasIntrinsics() const {
    return has_intrinsics_.load(std::memory_order_acquire);
  }
  cv_bridge::CvImageConstPtr CurrentOwner() {
    std::lock_guard<std::mutex> lk{m_};
    return current_owner_;
  }

 private:
  std::mutex m_;
  std::mutex intrinsics_mutex_;
  cv_bridge::CvImageConstPtr latest_owner_;
  cv_bridge::CvImageConstPtr current_owner_;
  std::atomic<bool> has_intrinsics_{false};
};

}  // namespace m3t_ros2

#endif  // M3T_ROS2_ROS_CAMERA_HPP_
