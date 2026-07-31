// SPDX-License-Identifier: MIT
// Full-frame RGB renderer for a single texture-mapped OBJ body.

#ifndef M3T_ROS2_TEXTURED_RENDERER_HPP_
#define M3T_ROS2_TEXTURED_RENDERER_HPP_

#include <GL/glew.h>

#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include <m3t/body.h>
#include <m3t/renderer.h>
#include <m3t/renderer_geometry.h>

namespace m3t_ros2 {

class FullTexturedRenderer final : public m3t::FullRenderer {
 public:
  FullTexturedRenderer(
      const std::string &name,
      const std::shared_ptr<m3t::RendererGeometry> &renderer_geometry,
      const std::shared_ptr<m3t::Body> &body,
      const std::filesystem::path &texture_path,
      const m3t::Transform3fA &world2camera_pose,
      const m3t::Intrinsics &intrinsics, float z_min = 0.02f,
      float z_max = 10.0f);
  ~FullTexturedRenderer();

  bool SetUp() override;
  bool StartRendering() override;
  bool FetchColorImage();

  const cv::Mat &color_image() const;

 private:
  bool LoadVertexData(std::vector<float> *vertex_data);
  bool CreateGpuResources(const std::vector<float> &vertex_data,
                          const cv::Mat &texture);
  void DeleteGpuResources();

  std::shared_ptr<m3t::Body> body_;
  std::filesystem::path texture_path_;
  cv::Mat color_image_;
  GLuint vertex_array_{0};
  GLuint vertex_buffer_{0};
  GLuint texture_{0};
  GLuint framebuffer_{0};
  GLuint color_renderbuffer_{0};
  GLuint depth_renderbuffer_{0};
  GLuint shader_program_{0};
  GLsizei n_vertices_{0};
  bool image_rendered_{false};
  bool color_image_fetched_{false};
  bool gpu_resources_created_{false};
};

}  // namespace m3t_ros2

#endif  // M3T_ROS2_TEXTURED_RENDERER_HPP_
