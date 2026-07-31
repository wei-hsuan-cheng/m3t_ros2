// SPDX-License-Identifier: MIT
// m3t_ros2 tracker node.
//
// The ROS executor owns only lightweight subscription and service callbacks.
// A high-priority compute thread exclusively owns all mutable M3T state. A
// lower-priority publisher thread consumes immutable snapshots, so TF, marker,
// image serialization, and keypoint visualization never block tracking.

#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#ifdef __linux__
#include <cerrno>
#include <cstring>
#include <pthread.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

#include <opencv2/imgproc.hpp>

#include <m3t/body.h>
#include <m3t/depth_modality.h>
#include <m3t/depth_model.h>
#include <m3t/link.h>
#include <m3t/normal_renderer.h>
#include <m3t/optimizer.h>
#include <m3t/region_modality.h>
#include <m3t/region_model.h>
#include <m3t/renderer_geometry.h>
#include <m3t/silhouette_renderer.h>
#include <m3t/static_detector.h>
#include <m3t/texture_modality.h>
#include <m3t/tracker.h>

#include "m3t_ros2/body_factory.hpp"
#include "m3t_ros2/ros_camera.hpp"
#include "m3t_ros2/ros_publisher.hpp"
#include "m3t_ros2/runtime_paths.hpp"

namespace fs = std::filesystem;
using clk = std::chrono::steady_clock;
using dsec = std::chrono::duration<double>;

namespace {

bool HasOption(const std::string &list, const std::string &option) {
  return ("," + list + ",").find("," + option + ",") != std::string::npos;
}

bool HasOnlyOptions(const std::string &list,
                    const std::vector<std::string> &allowed) {
  if (list.empty() || list == "none") return true;
  size_t begin = 0;
  while (begin <= list.size()) {
    const size_t end = list.find(',', begin);
    const auto option = list.substr(begin, end - begin);
    if (option.empty() ||
        std::find(allowed.begin(), allowed.end(), option) == allowed.end()) {
      return false;
    }
    if (end == std::string::npos) break;
    begin = end + 1;
  }
  return true;
}

m3t::Intrinsics FromInfo(const sensor_msgs::msg::CameraInfo &info) {
  return {static_cast<float>(info.k[0]), static_cast<float>(info.k[4]),
          static_cast<float>(info.k[2]), static_cast<float>(info.k[5]),
          static_cast<int>(info.width), static_cast<int>(info.height)};
}

bool HasValidIntrinsics(const sensor_msgs::msg::CameraInfo &info) {
  return info.width > 0 && info.height > 0 && info.k[0] > 0.0 &&
         info.k[4] > 0.0;
}

m3t::Transform3fA TfToTransform(
    const geometry_msgs::msg::Transform &transform) {
  m3t::Transform3fA result{m3t::Transform3fA::Identity()};
  result.translation() =
      Eigen::Vector3f(transform.translation.x, transform.translation.y,
                      transform.translation.z);
  result.linear() =
      Eigen::Quaternionf(transform.rotation.w, transform.rotation.x,
                         transform.rotation.y, transform.rotation.z)
          .toRotationMatrix();
  return result;
}

float RotationDistanceDeg(const Eigen::Matrix3f &a,
                          const Eigen::Matrix3f &b) {
  const float cosine = 0.5f * ((a.transpose() * b).trace() - 1.0f);
  return std::acos(std::clamp(cosine, -1.0f, 1.0f)) * 180.0f /
         static_cast<float>(M_PI);
}

std::vector<Eigen::Matrix3f> ParseRotationSymmetries(
    const std::vector<double> &values) {
  if (values.size() % 9 != 0) {
    throw std::invalid_argument(
        "rotation_symmetries must contain row-major 3x3 matrices");
  }
  std::vector<Eigen::Matrix3f> symmetries{Eigen::Matrix3f::Identity()};
  for (size_t offset = 0; offset < values.size(); offset += 9) {
    Eigen::Matrix3f symmetry;
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        symmetry(row, column) =
            static_cast<float>(values[offset + 3 * row + column]);
      }
    }
    const float orthogonality_error =
        (symmetry.transpose() * symmetry - Eigen::Matrix3f::Identity()).norm();
    if (!symmetry.allFinite() || orthogonality_error > 1.0e-4f ||
        std::abs(symmetry.determinant() - 1.0f) > 1.0e-4f) {
      throw std::invalid_argument(
          "rotation_symmetries entries must be valid rotation matrices");
    }
    symmetries.push_back(symmetry);
  }
  return symmetries;
}

float SymmetricRotationErrorDeg(
    const Eigen::Matrix3f &estimate, const Eigen::Matrix3f &reference,
    const std::vector<Eigen::Matrix3f> &symmetries) {
  float best_error = 180.0f;
  for (const auto &symmetry : symmetries) {
    best_error = std::min(
        best_error,
        RotationDistanceDeg(estimate * symmetry, reference));
  }
  return best_error;
}

m3t::Transform3fA ClosestSymmetricPose(
    const m3t::Transform3fA &pose, const m3t::Transform3fA &reference,
    const std::vector<Eigen::Matrix3f> &symmetries) {
  m3t::Transform3fA closest = pose;
  float best_error =
      RotationDistanceDeg(pose.rotation(), reference.rotation());
  for (const auto &symmetry : symmetries) {
    m3t::Transform3fA candidate = pose;
    candidate.linear() = pose.rotation() * symmetry;
    const float error =
        RotationDistanceDeg(candidate.rotation(), reference.rotation());
    if (error < best_error) {
      best_error = error;
      closest = candidate;
    }
  }
  return closest;
}

cv::Mat Composite(const cv::Mat &color, const cv::Mat &normal) {
  cv::Mat normal_bgr;
  cv::Mat overlay = color.clone();
  cv::cvtColor(normal, normal_bgr, cv::COLOR_BGRA2BGR);
  for (int row = 0; row < overlay.rows; ++row) {
    const auto *normal_row = normal_bgr.ptr<cv::Vec3b>(row);
    auto *overlay_row = overlay.ptr<cv::Vec3b>(row);
    for (int column = 0; column < overlay.cols; ++column) {
      const auto &n = normal_row[column];
      if (!(n[0] || n[1] || n[2])) continue;
      auto &pixel = overlay_row[column];
      for (int channel = 0; channel < 3; ++channel) {
        pixel[channel] = cv::saturate_cast<uchar>(
            0.5 * pixel[channel] + 0.5 * n[channel]);
      }
    }
  }
  return overlay;
}

void ConfigureCurrentThread(const char *name, int nice_value,
                            const rclcpp::Logger &logger) {
#ifdef __linux__
  const int name_result = pthread_setname_np(pthread_self(), name);
  if (name_result != 0) {
    RCLCPP_WARN(logger, "could not name thread '%s': %s", name,
                std::strerror(name_result));
  }
  const auto thread_id =
      static_cast<id_t>(::syscall(SYS_gettid));
  if (nice_value != 0 &&
      ::setpriority(PRIO_PROCESS, thread_id, nice_value) != 0) {
    RCLCPP_WARN(logger, "could not set thread '%s' nice=%d: %s", name,
                nice_value, std::strerror(errno));
  }
#else
  (void)name;
  (void)nice_value;
  (void)logger;
#endif
}

}  // namespace

class M3TTrackerNode final : public rclcpp::Node {
 public:
  M3TTrackerNode() : Node{"m3t_tracker_node"} {
    DeclareAndValidateParameters();
    CreateTrackingGraph();
    CreateRosInterfaces();

    RCLCPP_INFO(
        get_logger(),
        "waiting for %s camera data; tracking setup runs on tracker thread",
        use_depth_ ? "synchronized RGB-D" : "RGB");
    try {
      tracker_thread_ =
          std::thread{&M3TTrackerNode::TrackerThreadMain, this};
      publisher_thread_ =
          std::thread{&M3TTrackerNode::PublisherThreadMain, this};
    } catch (...) {
      Stop();
      throw;
    }
  }

  ~M3TTrackerNode() override { Stop(); }

  void Stop() {
    running_.store(false, std::memory_order_release);
    frame_cv_.notify_all();
    publisher_cv_.notify_all();
    if (tracker_thread_.joinable()) tracker_thread_.join();
    if (publisher_thread_.joinable()) publisher_thread_.join();
  }

 private:
  void DeclareAndValidateParameters() {
    modalities_ =
        declare_parameter<std::string>("modalities", "region,depth");
    model_cache_dir_ =
        declare_parameter<std::string>("model_cache_dir", "");
    const auto initial_pose_values =
        declare_parameter<std::vector<double>>(
            "initial_pose",
            {0.0, 0.0, 0.5, 0.0, 0.0, 0.0});
    const auto rotation_symmetry_values =
        declare_parameter<std::vector<double>>(
            "rotation_symmetries", std::vector<double>{});

    track_rate_ = declare_parameter<double>("track_rate", 0.0);
    publish_rate_ = declare_parameter<double>("publish_rate", 60.0);
    log_period_ = declare_parameter<double>("log_period", 2.0);
    event_driven_ = declare_parameter<bool>("event_driven", true);
    adaptive_iterations_ =
        declare_parameter<bool>("adaptive_iterations", true);
    min_corr_iterations_ =
        declare_parameter<int>("min_corr_iterations", 2);
    max_corr_iterations_ =
        declare_parameter<int>("max_corr_iterations", 7);
    n_update_iterations_ =
        declare_parameter<int>("n_update_iterations", 2);
    convergence_translation_threshold_ = declare_parameter<double>(
        "convergence_translation_threshold", 0.0001);
    convergence_rotation_threshold_deg_ = declare_parameter<double>(
        "convergence_rotation_threshold_deg", 0.05);
    convergence_required_rounds_ =
        declare_parameter<int>("convergence_required_rounds", 2);
    publisher_thread_nice_ =
        declare_parameter<int>("publisher_thread_nice", 5);

    const auto image_outputs =
        declare_parameter<std::string>("image_outputs", "none");
    depth_scale_ =
        static_cast<float>(declare_parameter<double>("depth_scale", 0.001));
    const double sync_tolerance =
        declare_parameter<double>("sync_tolerance", 0.02);
    color_topic_ = declare_parameter<std::string>(
        "color_topic", "/camera/color/image_raw");
    depth_topic_ = declare_parameter<std::string>(
        "depth_topic", "/camera/depth/image_raw");
    color_info_topic_ = declare_parameter<std::string>(
        "color_info_topic", "/camera/color/camera_info");
    depth_info_topic_ = declare_parameter<std::string>(
        "depth_info_topic", "/camera/depth/camera_info");
    gt_frame_ = declare_parameter<std::string>("gt_frame", "object_gt");
    lost_threshold_ =
        declare_parameter<double>("lost_threshold", 0.05);
    lost_rotation_threshold_ =
        declare_parameter<double>("lost_rotation_threshold", 45.0);
    use_gt_initial_pose_ =
        declare_parameter<bool>("use_gt_initial_pose", true);

    if (!HasOnlyOptions(image_outputs, {"overlay", "keypoints"})) {
      throw std::invalid_argument(
          "image_outputs must be none, overlay, keypoints, or "
          "overlay,keypoints");
    }
    publish_overlay_ = HasOption(image_outputs, "overlay");
    publish_keypoints_ = HasOption(image_outputs, "keypoints");

    if (min_corr_iterations_ < 1 ||
        max_corr_iterations_ < min_corr_iterations_ ||
        n_update_iterations_ < 1 || convergence_required_rounds_ < 1 ||
        convergence_required_rounds_ > max_corr_iterations_ ||
        convergence_translation_threshold_ < 0.0 ||
        convergence_rotation_threshold_deg_ < 0.0 ||
        track_rate_ < 0.0 || publish_rate_ <= 0.0 ||
        log_period_ <= 0.0 || sync_tolerance < 0.0 ||
        publisher_thread_nice_ < 0 || publisher_thread_nice_ > 19) {
      throw std::invalid_argument(
          "invalid tracker parameters: require 1 <= min_corr <= max_corr, "
          "1 <= required_rounds <= max_corr, n_update >= 1, non-negative "
          "convergence thresholds/track_rate/sync_tolerance, positive "
          "publish_rate/log_period, and publisher_thread_nice in [0, 19]");
    }
    if (!event_driven_) {
      RCLCPP_WARN(
          get_logger(),
          "event_driven=false repeatedly updates stateful modalities with the "
          "same image; use only for short compute benchmarks");
    }

    sync_tolerance_ns_ =
        static_cast<int64_t>(sync_tolerance * 1.0e9);
    initial_pose_ =
        m3t_ros2::TransformFromPose(initial_pose_values, "initial_pose");
    detector_initial_pose_ = initial_pose_;
    rotation_symmetries_ =
        ParseRotationSymmetries(rotation_symmetry_values);

    publisher_config_.world_frame =
        declare_parameter<std::string>("world_frame", "camera");
    publisher_config_.mesh_resource =
        declare_parameter<std::string>("mesh_resource", "");
    publisher_config_.mesh_scale =
        declare_parameter<double>("mesh_scale", 1.0);
    publisher_config_.mesh_use_embedded_materials =
        declare_parameter<bool>("mesh_use_embedded_materials", false);
    publisher_config_.publish_overlay = publish_overlay_;
    publisher_config_.publish_keypoints = publish_keypoints_;
    publisher_config_.publish_color = false;
    publisher_config_.publish_depth = false;
    publisher_config_.publish_gt = false;
  }

  void CreateTrackingGraph() {
    body_ = m3t_ros2::DeclareAndCreateBody(this);
    const std::string object_name = body_->name();
    if (model_cache_dir_.empty()) {
      model_cache_dir_ =
          (m3t_ros2::DefaultRuntimeRoot() / "cache" /
           m3t_ros2::SanitizePathComponent(object_name))
              .string();
    }
    std::string cache_error;
    if (!m3t_ros2::EnsureWritableDirectory(
            model_cache_dir_, &cache_error)) {
      throw std::runtime_error("model cache error: " + cache_error);
    }

    use_region_ = HasOption(modalities_, "region");
    use_depth_ = HasOption(modalities_, "depth");
    use_texture_ = HasOption(modalities_, "texture");
    if (!use_region_ && !use_depth_ && !use_texture_) {
      throw std::invalid_argument(
          "modalities must contain region, depth, and/or texture");
    }
    RCLCPP_INFO(get_logger(), "model cache: %s",
                fs::absolute(model_cache_dir_).lexically_normal().c_str());

    color_camera_ =
        std::make_shared<m3t_ros2::RosColorCamera>("color_camera");
    color_camera_->SetUp();
    if (use_depth_) {
      depth_camera_ =
          std::make_shared<m3t_ros2::RosDepthCamera>("depth_camera");
      depth_camera_->SetDepthScale(depth_scale_);
      depth_camera_->SetUp();
    }

    link_ = std::make_shared<m3t::Link>("link", body_);
    if (use_region_) {
      auto region_model = std::make_shared<m3t::RegionModel>(
          "region_model", body_,
          fs::path{model_cache_dir_} / "region_model.bin");
      link_->AddModality(std::make_shared<m3t::RegionModality>(
          "region_modality", body_, color_camera_, region_model));
    }
    if (use_depth_) {
      auto depth_model = std::make_shared<m3t::DepthModel>(
          "depth_model", body_,
          fs::path{model_cache_dir_} / "depth_model.bin");
      link_->AddModality(std::make_shared<m3t::DepthModality>(
          "depth_modality", body_, depth_camera_, depth_model));
    }
    if (use_texture_) {
      auto renderer_geometry =
          std::make_shared<m3t::RendererGeometry>("rg_texture");
      renderer_geometry->AddBody(body_);
      auto silhouette_renderer =
          std::make_shared<m3t::FocusedSilhouetteRenderer>(
              "silhouette_renderer", renderer_geometry, color_camera_);
      silhouette_renderer->AddReferencedBody(body_);
      link_->AddModality(std::make_shared<m3t::TextureModality>(
          "texture_modality", body_, color_camera_,
          silhouette_renderer));
    }

    optimizer_ = std::make_shared<m3t::Optimizer>("optimizer", link_);
    tracker_ = std::make_shared<m3t::Tracker>(
        "tracker", max_corr_iterations_, n_update_iterations_, false, true);
    tracker_->AddOptimizer(optimizer_);
    detector_ = use_gt_initial_pose_
                    ? std::make_shared<m3t::StaticDetector>(
                          "detector", optimizer_,
                          m3t::Transform3fA::Identity(), true)
                    : std::make_shared<m3t::StaticDetector>(
                          "detector", optimizer_, initial_pose_, true);
    tracker_->AddDetector(detector_);

    if (publish_overlay_) {
      overlay_geometry_ =
          std::make_shared<m3t::RendererGeometry>("rg_overlay");
      overlay_geometry_->AddBody(body_);
      overlay_renderer_ =
          std::make_shared<m3t::FullNormalRenderer>(
              "overlay_renderer", overlay_geometry_, color_camera_,
              0.01f, 10.0f);
    }
  }

  void CreateRosInterfaces() {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(
        *tf_buffer_, this, false);

    const auto qos = rclcpp::SensorDataQoS();
    color_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        color_topic_, qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
          OnColorImage(std::move(message));
        });
    color_info_subscription_ =
        create_subscription<sensor_msgs::msg::CameraInfo>(
            color_info_topic_, qos,
            [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
              OnColorInfo(std::move(message));
            });
    if (use_depth_) {
      depth_subscription_ = create_subscription<sensor_msgs::msg::Image>(
          depth_topic_, qos,
          [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
            OnDepthImage(std::move(message));
          });
      depth_info_subscription_ =
          create_subscription<sensor_msgs::msg::CameraInfo>(
              depth_info_topic_, qos,
              [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
                OnDepthInfo(std::move(message));
              });
    }
    redetect_service_ = create_service<std_srvs::srv::Trigger>(
        "~/redetect",
        [this](std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          redetect_requested_.store(true, std::memory_order_release);
          response->success = true;
          response->message = "re-detect queued";
        });

    ros_publisher_ = std::make_unique<m3t_ros2::RosPublisher>(
        this, publisher_config_, body_);
  }

  void OnColorImage(sensor_msgs::msg::Image::ConstSharedPtr message) {
    cv_bridge::CvImageConstPtr image;
    try {
      image = cv_bridge::toCvShare(message, "bgr8");
    } catch (const cv_bridge::Exception &error) {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "RGB conversion failed: %s", error.what());
      return;
    }

    if (!use_depth_) {
      bool accepted = false;
      {
        std::lock_guard<std::mutex> wait_lock{frame_wait_mutex_};
        accepted = color_camera_->SetLatest(std::move(image));
      }
      if (!accepted) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "ignoring RGB image whose dimensions do not match CameraInfo");
        return;
      }
      frame_cv_.notify_one();
      return;
    }

    std::lock_guard<std::mutex> input_lock{input_sync_mutex_};
    pending_color_ = std::move(image);
    pending_color_stamp_ =
        rclcpp::Time{message->header.stamp}.nanoseconds();
    CommitSynchronizedFrameLocked();
  }

  void OnDepthImage(sensor_msgs::msg::Image::ConstSharedPtr message) {
    cv_bridge::CvImageConstPtr image;
    try {
      image = cv_bridge::toCvShare(message, "16UC1");
    } catch (const cv_bridge::Exception &error) {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "depth conversion failed: %s", error.what());
      return;
    }

    std::lock_guard<std::mutex> input_lock{input_sync_mutex_};
    pending_depth_ = std::move(image);
    pending_depth_stamp_ =
        rclcpp::Time{message->header.stamp}.nanoseconds();
    CommitSynchronizedFrameLocked();
  }

  void CommitSynchronizedFrameLocked() {
    if (!pending_color_ || !pending_depth_) return;
    const int64_t delta =
        std::llabs(pending_color_stamp_ - pending_depth_stamp_);
    if (delta > sync_tolerance_ns_) {
      if (pending_color_stamp_ < pending_depth_stamp_) {
        pending_color_.reset();
      } else {
        pending_depth_.reset();
      }
      return;
    }

    if (!color_camera_->CanAccept(pending_color_) ||
        !depth_camera_->CanAccept(pending_depth_)) {
      pending_color_.reset();
      pending_depth_.reset();
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "ignoring RGB-D pair whose dimensions do not match CameraInfo");
      return;
    }
    {
      std::lock_guard<std::mutex> wait_lock{frame_wait_mutex_};
      depth_camera_->SetLatest(std::move(pending_depth_));
      color_camera_->SetLatest(std::move(pending_color_));
    }
    frame_cv_.notify_one();
  }

  void OnColorInfo(
      sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
    if (!HasValidIntrinsics(*message)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "ignoring invalid RGB CameraInfo");
      return;
    }
    {
      std::lock_guard<std::mutex> wait_lock{frame_wait_mutex_};
      color_camera_->SetIntrinsics(FromInfo(*message));
    }
    frame_cv_.notify_one();
  }

  void OnDepthInfo(
      sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
    if (!HasValidIntrinsics(*message)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "ignoring invalid depth CameraInfo");
      return;
    }
    {
      std::lock_guard<std::mutex> wait_lock{frame_wait_mutex_};
      depth_camera_->SetIntrinsics(FromInfo(*message));
    }
    frame_cv_.notify_one();
  }

  bool LookupGroundTruth(m3t::Transform3fA *pose) const {
    try {
      *pose = TfToTransform(
          tf_buffer_
              ->lookupTransform(publisher_config_.world_frame, gt_frame_,
                                tf2::TimePointZero)
              .transform);
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  bool InputReady() const {
    if (!color_camera_->Ready()) return false;
    if (use_depth_ && !depth_camera_->Ready()) return false;
    if (!use_gt_initial_pose_) return true;
    m3t::Transform3fA pose;
    return LookupGroundTruth(&pose);
  }

  bool WaitForInitialInput() {
    while (running_.load(std::memory_order_acquire) && rclcpp::ok()) {
      if (InputReady()) return true;
      std::unique_lock<std::mutex> lock{frame_wait_mutex_};
      frame_cv_.wait_for(lock, std::chrono::milliseconds{50});
    }
    return false;
  }

  bool WaitForNewFrame(uint64_t last_sequence) {
    std::unique_lock<std::mutex> lock{frame_wait_mutex_};
    frame_cv_.wait(lock, [this, last_sequence]() {
      return !running_.load(std::memory_order_acquire) || !rclcpp::ok() ||
             color_camera_->seq() != last_sequence;
    });
    return running_.load(std::memory_order_acquire) && rclcpp::ok();
  }

  bool ExecuteAdaptiveTrackingStep(int iteration, int *corr_rounds) {
    *corr_rounds = 0;
    int converged_rounds = 0;
    for (int corr_iteration = 0;
         corr_iteration < max_corr_iterations_; ++corr_iteration) {
      const auto pose_before_round = body_->body2world_pose();
      const int corr_save_idx =
          iteration * max_corr_iterations_ + corr_iteration;
      if (!tracker_->CalculateCorrespondences(iteration, corr_iteration) ||
          !tracker_->VisualizeCorrespondences(corr_save_idx)) {
        return false;
      }

      for (int update_iteration = 0;
           update_iteration < n_update_iterations_; ++update_iteration) {
        const int update_save_idx =
            corr_save_idx * n_update_iterations_ + update_iteration;
        if (!tracker_->CalculateGradientAndHessian(
                iteration, corr_iteration, update_iteration) ||
            !tracker_->CalculateOptimization(
                iteration, corr_iteration, update_iteration) ||
            !tracker_->VisualizeOptimization(update_save_idx) ||
            !body_->body2world_pose().matrix().allFinite()) {
          return false;
        }
      }

      body_->set_body2world_pose(ClosestSymmetricPose(
          body_->body2world_pose(), pose_before_round,
          rotation_symmetries_));
      ++(*corr_rounds);
      const auto pose_after_round = body_->body2world_pose();
      const double translation_delta =
          (pose_after_round.translation() -
           pose_before_round.translation()).norm();
      const double rotation_delta = RotationDistanceDeg(
          pose_after_round.rotation(), pose_before_round.rotation());
      if (translation_delta <= convergence_translation_threshold_ &&
          rotation_delta <= convergence_rotation_threshold_deg_) {
        ++converged_rounds;
      } else {
        converged_rounds = 0;
      }
      if (adaptive_iterations_ &&
          *corr_rounds >= min_corr_iterations_ &&
          converged_rounds >= convergence_required_rounds_) {
        break;
      }
    }
    return tracker_->CalculateResults(iteration) &&
           tracker_->VisualizeResults(iteration);
  }

  void StoreSnapshot() {
    auto snapshot = std::make_shared<m3t_ros2::Snapshot>();
    if (publish_overlay_ &&
        overlay_renderer_->StartRendering() &&
        overlay_renderer_->FetchNormalImage()) {
      snapshot->overlay = Composite(
          color_camera_->image(), overlay_renderer_->normal_image());
    }
    if (publish_keypoints_) {
      snapshot->color_owner = color_camera_->CurrentOwner();
      if (snapshot->color_owner) {
        snapshot->color = snapshot->color_owner->image;
      }
    }
    snapshot->body2world_est = body_->body2world_pose();
    snapshot->geometry2world_est = body_->geometry2world_pose();
    snapshot->valid = true;
    std::shared_ptr<const m3t_ros2::Snapshot> immutable = snapshot;
    std::atomic_store_explicit(
        &latest_snapshot_, std::move(immutable),
        std::memory_order_release);
  }

  void FailWorker(const char *worker, const char *message) noexcept {
    RCLCPP_FATAL(get_logger(), "%s thread failed: %s", worker, message);
    running_.store(false, std::memory_order_release);
    frame_cv_.notify_all();
    publisher_cv_.notify_all();
    try {
      if (rclcpp::ok()) rclcpp::shutdown();
    } catch (...) {
    }
  }

  void TrackerThreadMain() noexcept {
    try {
      TrackerLoop();
    } catch (const std::exception &error) {
      FailWorker("tracker", error.what());
    } catch (...) {
      FailWorker("tracker", "unknown exception");
    }
  }

  void TrackerLoop() {
    ConfigureCurrentThread("m3t_tracker", 0, get_logger());
    if (!WaitForInitialInput()) return;

    RCLCPP_INFO(
        get_logger(),
        "got first frames; setting up tracker on compute thread "
        "(region=%d depth=%d texture=%d)",
        use_region_, use_depth_, use_texture_);
    if (!tracker_->SetUp() ||
        (publish_overlay_ &&
         (!overlay_geometry_->SetUp() || !overlay_renderer_->SetUp()))) {
      RCLCPP_FATAL(get_logger(), "M3T setup failed");
      running_.store(false, std::memory_order_release);
      publisher_cv_.notify_all();
      rclcpp::shutdown();
      return;
    }

    if (use_gt_initial_pose_) {
      if (!LookupGroundTruth(&detector_initial_pose_)) {
        RCLCPP_FATAL(
            get_logger(),
            "ground-truth TF disappeared before one-time initialization");
        running_.store(false, std::memory_order_release);
        publisher_cv_.notify_all();
        rclcpp::shutdown();
        return;
      }
      detector_->set_link2world_pose(detector_initial_pose_);
    }
    tracker_->ExecuteDetection(false);

    RCLCPP_INFO(
        get_logger(),
        "tracking | mode=%s track_rate=%.0f (0=max) publish_rate=%.0f "
        "image_outputs=%s corr=%d..%d updates=%d adaptive=%d "
        "convergence=%.3fmm/%.3fdeg x%d",
        event_driven_ ? "new-frame" : "free-run", track_rate_,
        publish_rate_,
        publish_overlay_ || publish_keypoints_ ? "enabled" : "none",
        min_corr_iterations_, max_corr_iterations_,
        n_update_iterations_, adaptive_iterations_,
        1.0e3 * convergence_translation_threshold_,
        convergence_rotation_threshold_deg_,
        convergence_required_rounds_);

    auto last_log = clk::now();
    auto last_snapshot = clk::now();
    auto window_start = clk::now();
    int solve_count = 0;
    int loop_count = 0;
    int error_count = 0;
    int lost_count = 0;
    int corr_round_sum = 0;
    int corr_round_max = 0;
    double solve_sum = 0.0;
    double solve_min = 1.0e9;
    double solve_max = 0.0;
    double position_error_sum = 0.0;
    double position_error_max = 0.0;
    double rotation_error_sum = 0.0;
    double rotation_error_max = 0.0;
    const dsec track_period{
        track_rate_ > 0.0 ? 1.0 / track_rate_ : 0.0};
    const dsec snapshot_period{1.0 / publish_rate_};
    uint64_t last_sequence = 0;

    for (int iteration = 0;
         running_.load(std::memory_order_acquire) && rclcpp::ok();
         ++iteration) {
      if (redetect_requested_.exchange(
              false, std::memory_order_acq_rel)) {
        // Reuse the startup detector pose. In GT initialization mode, live GT
        // must never be fed back into the estimate after initialization.
        detector_->set_link2world_pose(detector_initial_pose_);
        tracker_->ExecuteDetection(false);
      }
      if (event_driven_ && !WaitForNewFrame(last_sequence)) break;
      if (event_driven_) last_sequence = color_camera_->seq();

      const auto iteration_start = clk::now();
      if (!tracker_->UpdateCameras(iteration)) {
        if (!event_driven_) {
          std::this_thread::sleep_for(std::chrono::milliseconds{1});
        }
        continue;
      }
      const auto pose_before_update = body_->body2world_pose();
      if (!tracker_->UpdateSubscribers(iteration) ||
          !tracker_->CalculateConsistentPoses() ||
          !tracker_->ExecuteDetectingStep(iteration) ||
          !tracker_->ExecuteStartingStep(iteration)) {
        body_->set_body2world_pose(pose_before_update);
        redetect_requested_.store(true, std::memory_order_release);
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "M3T frame preparation failed; restored pose and queued "
            "re-detection");
        continue;
      }

      const auto solve_start = clk::now();
      int corr_rounds = 0;
      if (!ExecuteAdaptiveTrackingStep(iteration, &corr_rounds)) {
        body_->set_body2world_pose(pose_before_update);
        redetect_requested_.store(true, std::memory_order_release);
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "M3T tracking failed; restored pose and queued re-detection");
        continue;
      }
      const double solve_ms =
          dsec(clk::now() - solve_start).count() * 1.0e3;
      if (!body_->body2world_pose().matrix().allFinite()) {
        body_->set_body2world_pose(pose_before_update);
        redetect_requested_.store(true, std::memory_order_release);
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "M3T produced a non-finite pose; restored pose and queued "
            "re-detection");
        continue;
      }
      body_->set_body2world_pose(ClosestSymmetricPose(
          body_->body2world_pose(), pose_before_update,
          rotation_symmetries_));

      solve_sum += solve_ms;
      solve_min = std::min(solve_min, solve_ms);
      solve_max = std::max(solve_max, solve_ms);
      ++solve_count;
      ++loop_count;
      corr_round_sum += corr_rounds;
      corr_round_max = std::max(corr_round_max, corr_rounds);

      m3t::Transform3fA ground_truth;
      if (LookupGroundTruth(&ground_truth)) {
        const auto estimate = body_->body2world_pose();
        const double position_error =
            (estimate.translation() -
             ground_truth.translation()).norm();
        const double rotation_error = SymmetricRotationErrorDeg(
            estimate.rotation(), ground_truth.rotation(),
            rotation_symmetries_);
        position_error_sum += position_error;
        position_error_max =
            std::max(position_error_max, position_error);
        rotation_error_sum += rotation_error;
        rotation_error_max =
            std::max(rotation_error_max, rotation_error);
        if (position_error > lost_threshold_ ||
            (lost_rotation_threshold_ > 0.0 &&
             rotation_error > lost_rotation_threshold_)) {
          ++lost_count;
        }
        ++error_count;
      }

      if (event_driven_ ||
          clk::now() - last_snapshot >= snapshot_period) {
        StoreSnapshot();
        last_snapshot = clk::now();
      }

      if (clk::now() - last_log >= dsec{log_period_}) {
        const double window_seconds =
            dsec(clk::now() - window_start).count();
        const double mean_solve =
            solve_sum / std::max(1, solve_count);
        const double mean_corr =
            solve_count
                ? static_cast<double>(corr_round_sum) / solve_count
                : 0.0;
        const double mean_position_error =
            error_count ? 1.0e3 * position_error_sum / error_count : -1.0;
        const double mean_rotation_error =
            error_count ? rotation_error_sum / error_count : -1.0;
        const bool tracked =
            error_count > 0 && lost_count * 2 <= error_count;
        RCLCPP_INFO(
            get_logger(),
            "solve: %d | mean %.2f ms (%.0f Hz) | min %.2f max %.2f | "
            "loop %.0f Hz | corr %.1f/%d | err pos %.1f/%.1f mm "
            "rot %.1f/%.1f deg | %s",
            solve_count, mean_solve,
            mean_solve > 0.0 ? 1000.0 / mean_solve : 0.0,
            solve_min, solve_max,
            loop_count / std::max(1.0e-6, window_seconds),
            mean_corr, corr_round_max, mean_position_error,
            1.0e3 * position_error_max, mean_rotation_error,
            rotation_error_max,
            error_count == 0 ? "no-GT" : (tracked ? "TRACKED" : "LOST"));

        solve_count = 0;
        loop_count = 0;
        error_count = 0;
        lost_count = 0;
        corr_round_sum = 0;
        corr_round_max = 0;
        solve_sum = 0.0;
        solve_min = 1.0e9;
        solve_max = 0.0;
        position_error_sum = 0.0;
        position_error_max = 0.0;
        rotation_error_sum = 0.0;
        rotation_error_max = 0.0;
        window_start = last_log = clk::now();
      }

      if (track_period.count() > 0.0) {
        std::this_thread::sleep_until(
            iteration_start +
            std::chrono::duration_cast<clk::duration>(track_period));
      }
    }
  }

  void PublisherThreadMain() noexcept {
    try {
      PublisherLoop();
    } catch (const std::exception &error) {
      FailWorker("publisher", error.what());
    } catch (...) {
      FailWorker("publisher", "unknown exception");
    }
  }

  void PublisherLoop() {
    ConfigureCurrentThread(
        "m3t_publisher", publisher_thread_nice_, get_logger());
    const auto period = std::chrono::duration_cast<clk::duration>(
        dsec{1.0 / publish_rate_});
    auto next_publish = clk::now() + period;
    std::shared_ptr<const m3t_ros2::Snapshot> last_snapshot;
    while (running_.load(std::memory_order_acquire) && rclcpp::ok()) {
      {
        std::unique_lock<std::mutex> lock{publisher_wait_mutex_};
        publisher_cv_.wait_until(lock, next_publish, [this]() {
          return !running_.load(std::memory_order_acquire) ||
                 !rclcpp::ok();
        });
      }
      if (!running_.load(std::memory_order_acquire) || !rclcpp::ok()) {
        break;
      }

      const auto snapshot = std::atomic_load_explicit(
          &latest_snapshot_, std::memory_order_acquire);
      if (snapshot) {
        const bool is_new_snapshot = snapshot != last_snapshot;
        ros_publisher_->Publish(*snapshot, is_new_snapshot);
        last_snapshot = snapshot;
      }

      next_publish += period;
      const auto now = clk::now();
      if (next_publish <= now) next_publish = now + period;
    }
  }

  // Parameters
  std::string modalities_;
  std::string model_cache_dir_;
  std::string color_topic_;
  std::string depth_topic_;
  std::string color_info_topic_;
  std::string depth_info_topic_;
  std::string gt_frame_;
  double track_rate_{0.0};
  double publish_rate_{60.0};
  double log_period_{2.0};
  bool event_driven_{true};
  bool adaptive_iterations_{true};
  int min_corr_iterations_{2};
  int max_corr_iterations_{7};
  int n_update_iterations_{2};
  double convergence_translation_threshold_{0.0001};
  double convergence_rotation_threshold_deg_{0.05};
  int convergence_required_rounds_{2};
  int publisher_thread_nice_{5};
  float depth_scale_{0.001f};
  int64_t sync_tolerance_ns_{20000000};
  double lost_threshold_{0.05};
  double lost_rotation_threshold_{45.0};
  bool use_gt_initial_pose_{true};
  bool publish_overlay_{false};
  bool publish_keypoints_{false};
  bool use_region_{false};
  bool use_depth_{false};
  bool use_texture_{false};
  m3t::Transform3fA initial_pose_{m3t::Transform3fA::Identity()};
  m3t::Transform3fA detector_initial_pose_{
      m3t::Transform3fA::Identity()};
  std::vector<Eigen::Matrix3f> rotation_symmetries_;
  m3t_ros2::RosPublisherConfig publisher_config_;

  // ROS interfaces (executor thread).
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
      color_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
      depth_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
      color_info_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
      depth_info_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr redetect_service_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // Image ingress. CvImage owners keep zero-copy views alive.
  std::mutex input_sync_mutex_;
  cv_bridge::CvImageConstPtr pending_color_;
  cv_bridge::CvImageConstPtr pending_depth_;
  int64_t pending_color_stamp_{0};
  int64_t pending_depth_stamp_{0};
  std::mutex frame_wait_mutex_;
  std::condition_variable frame_cv_;

  // M3T state is exclusively owned by tracker_thread_ after construction.
  std::shared_ptr<m3t::Body> body_;
  std::shared_ptr<m3t_ros2::RosColorCamera> color_camera_;
  std::shared_ptr<m3t_ros2::RosDepthCamera> depth_camera_;
  std::shared_ptr<m3t::Link> link_;
  std::shared_ptr<m3t::Optimizer> optimizer_;
  std::shared_ptr<m3t::Tracker> tracker_;
  std::shared_ptr<m3t::StaticDetector> detector_;
  std::shared_ptr<m3t::RendererGeometry> overlay_geometry_;
  std::shared_ptr<m3t::FullNormalRenderer> overlay_renderer_;
  std::atomic<bool> redetect_requested_{false};

  // Immutable tracker-to-publisher handoff.
  std::shared_ptr<const m3t_ros2::Snapshot> latest_snapshot_;
  std::unique_ptr<m3t_ros2::RosPublisher> ros_publisher_;
  std::mutex publisher_wait_mutex_;
  std::condition_variable publisher_cv_;

  std::atomic<bool> running_{true};
  std::thread tracker_thread_;
  std::thread publisher_thread_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<M3TTrackerNode>();
    rclcpp::spin(node);
    node->Stop();
  } catch (const std::exception &error) {
    std::cerr << "m3t_tracker_node: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}