// SPDX-License-Identifier: MIT

#include "m3t_ros2/textured_renderer.hpp"

#include <GLFW/glfw3.h>

#include <array>
#include <iostream>
#include <utility>

#include <opencv2/imgcodecs.hpp>

#include <tiny_obj_loader/tiny_obj_loader.h>

namespace m3t_ros2 {
namespace {

constexpr char kVertexShader[] =
    "#version 330 core\n"
    "layout(location = 0) in vec3 a_position;\n"
    "layout(location = 1) in vec2 a_tex_coord;\n"
    "out vec2 tex_coord;\n"
    "uniform mat4 transform;\n"
    "void main() {\n"
    "  gl_Position = transform * vec4(a_position, 1.0);\n"
    "  tex_coord = vec2(a_tex_coord.x, 1.0 - a_tex_coord.y);\n"
    "}\n";

constexpr char kFragmentShader[] =
    "#version 330 core\n"
    "in vec2 tex_coord;\n"
    "out vec4 fragment_color;\n"
    "uniform sampler2D color_texture;\n"
    "void main() {\n"
    "  fragment_color = texture(color_texture, tex_coord);\n"
    "}\n";

}  // namespace

FullTexturedRenderer::FullTexturedRenderer(
    const std::string &name,
    const std::shared_ptr<m3t::RendererGeometry> &renderer_geometry,
    const std::shared_ptr<m3t::Body> &body,
    const std::filesystem::path &texture_path,
    const m3t::Transform3fA &world2camera_pose,
    const m3t::Intrinsics &intrinsics, float z_min, float z_max)
    : m3t::FullRenderer{name, renderer_geometry, world2camera_pose,
                        intrinsics, z_min, z_max},
      body_{body},
      texture_path_{texture_path} {}

FullTexturedRenderer::~FullTexturedRenderer() {
  std::lock_guard<std::mutex> lock{mutex_};
  DeleteGpuResources();
}

bool FullTexturedRenderer::SetUp() {
  std::lock_guard<std::mutex> lock{mutex_};
  set_up_ = false;
  if (!renderer_geometry_ptr_ || !renderer_geometry_ptr_->set_up()) {
    std::cerr << "Renderer geometry is not set up" << std::endl;
    return false;
  }
  if (!body_ || !body_->set_up()) {
    std::cerr << "Textured renderer body is not set up" << std::endl;
    return false;
  }

  std::vector<float> vertex_data;
  if (!LoadVertexData(&vertex_data)) return false;
  const cv::Mat texture = cv::imread(texture_path_.string(), cv::IMREAD_COLOR);
  if (texture.empty()) {
    std::cerr << "Could not load texture image " << texture_path_ << std::endl;
    return false;
  }

  CalculateProjectionMatrix();
  color_image_ = cv::Mat{intrinsics_.height, intrinsics_.width, CV_8UC3,
                         cv::Scalar{0, 0, 0}};
  DeleteGpuResources();
  if (!CreateGpuResources(vertex_data, texture)) return false;

  image_rendered_ = false;
  color_image_fetched_ = false;
  set_up_ = true;
  return true;
}

bool FullTexturedRenderer::StartRendering() {
  std::lock_guard<std::mutex> lock{mutex_};
  if (!set_up_ || !renderer_geometry_ptr_->MakeContextCurrent()) {
    return false;
  }

  glViewport(0, 0, intrinsics_.width, intrinsics_.height);
  glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);
  glFrontFace(GL_CCW);
  glCullFace(GL_FRONT);
  if (body_->geometry_enable_culling()) {
    glEnable(GL_CULL_FACE);
  } else {
    glDisable(GL_CULL_FACE);
  }

  const m3t::Transform3fA geometry2camera{
      world2camera_pose_ * body_->geometry2world_pose()};
  const Eigen::Matrix4f transform{
      projection_matrix_ * geometry2camera.matrix()};

  glUseProgram(shader_program_);
  const GLint transform_location =
      glGetUniformLocation(shader_program_, "transform");
  glUniformMatrix4fv(transform_location, 1, GL_FALSE, transform.data());
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, texture_);
  glUniform1i(glGetUniformLocation(shader_program_, "color_texture"), 0);
  glBindVertexArray(vertex_array_);
  glDrawArrays(GL_TRIANGLES, 0, n_vertices_);
  glBindVertexArray(0);
  glBindTexture(GL_TEXTURE_2D, 0);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  renderer_geometry_ptr_->DetachContext();

  image_rendered_ = true;
  color_image_fetched_ = false;
  return true;
}

bool FullTexturedRenderer::FetchColorImage() {
  std::lock_guard<std::mutex> lock{mutex_};
  if (!set_up_ || !image_rendered_) return false;
  if (color_image_fetched_) return true;
  if (!renderer_geometry_ptr_->MakeContextCurrent()) return false;

  glPixelStorei(GL_PACK_ALIGNMENT, (color_image_.step & 3) ? 1 : 4);
  glPixelStorei(
      GL_PACK_ROW_LENGTH,
      static_cast<GLint>(color_image_.step / color_image_.elemSize()));
  glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);
  glReadPixels(0, 0, intrinsics_.width, intrinsics_.height, GL_BGR,
               GL_UNSIGNED_BYTE, color_image_.data);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  renderer_geometry_ptr_->DetachContext();
  color_image_fetched_ = true;
  return true;
}

const cv::Mat &FullTexturedRenderer::color_image() const {
  return color_image_;
}

bool FullTexturedRenderer::LoadVertexData(
    std::vector<float> *vertex_data) {
  tinyobj::attrib_t attributes;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;
  std::string warning;
  std::string error;
  const auto geometry_path = body_->geometry_path();
  const std::string material_directory =
      geometry_path.parent_path().string() + "/";
  if (!tinyobj::LoadObj(
          &attributes, &shapes, &materials, &warning, &error,
          geometry_path.string().c_str(), material_directory.c_str(), true,
          false)) {
    std::cerr << "Could not load textured OBJ " << geometry_path << ": "
              << error << std::endl;
    return false;
  }
  if (!warning.empty()) std::cerr << warning << std::endl;
  if (attributes.texcoords.empty()) {
    std::cerr << "Textured OBJ has no texture coordinates: "
              << geometry_path << std::endl;
    return false;
  }

  for (const auto &shape : shapes) {
    size_t index_offset = 0;
    for (const auto n_face_vertices : shape.mesh.num_face_vertices) {
      if (n_face_vertices != 3) {
        std::cerr << "Textured OBJ contains a non-triangle face" << std::endl;
        return false;
      }
      const std::array<int, 3> order =
          body_->geometry_counterclockwise()
              ? std::array<int, 3>{0, 1, 2}
              : std::array<int, 3>{2, 1, 0};
      for (const int offset : order) {
        const auto &index = shape.mesh.indices[index_offset + offset];
        if (index.vertex_index < 0 || index.texcoord_index < 0 ||
            static_cast<size_t>(index.vertex_index) >=
                body_->vertices().size() ||
            static_cast<size_t>(2 * index.texcoord_index + 1) >=
                attributes.texcoords.size()) {
          std::cerr << "Textured OBJ contains an invalid vertex or UV index"
                    << std::endl;
          return false;
        }
        const Eigen::Vector3f &point = body_->vertices()[index.vertex_index];
        vertex_data->insert(
            vertex_data->end(), {point.x(), point.y(), point.z(),
                                 attributes.texcoords[2 *
                                                      index.texcoord_index],
                                 attributes.texcoords[
                                     2 * index.texcoord_index + 1]});
      }
      index_offset += n_face_vertices;
    }
  }
  n_vertices_ = static_cast<GLsizei>(vertex_data->size() / 5);
  return n_vertices_ > 0;
}

bool FullTexturedRenderer::CreateGpuResources(
    const std::vector<float> &vertex_data, const cv::Mat &texture) {
  bool success = m3t::CreateShaderProgram(
      renderer_geometry_ptr_.get(), kVertexShader, kFragmentShader,
      &shader_program_);
  if (!success) return false;
  if (!renderer_geometry_ptr_->MakeContextCurrent()) return false;

  glGenVertexArrays(1, &vertex_array_);
  glBindVertexArray(vertex_array_);
  glGenBuffers(1, &vertex_buffer_);
  glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);
  glBufferData(GL_ARRAY_BUFFER, vertex_data.size() * sizeof(float),
               vertex_data.data(), GL_STATIC_DRAW);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), nullptr);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float),
                        reinterpret_cast<void *>(3 * sizeof(float)));
  glEnableVertexAttribArray(1);
  glBindVertexArray(0);

  glGenTextures(1, &texture_);
  glBindTexture(GL_TEXTURE_2D, texture_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glPixelStorei(GL_UNPACK_ALIGNMENT, (texture.step & 3) ? 1 : 4);
  glPixelStorei(
      GL_UNPACK_ROW_LENGTH,
      static_cast<GLint>(texture.step / texture.elemSize()));
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, texture.cols, texture.rows, 0,
               GL_BGR, GL_UNSIGNED_BYTE, texture.data);
  glBindTexture(GL_TEXTURE_2D, 0);
  glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);

  glGenRenderbuffers(1, &color_renderbuffer_);
  glBindRenderbuffer(GL_RENDERBUFFER, color_renderbuffer_);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_RGB8, intrinsics_.width,
                        intrinsics_.height);
  glGenRenderbuffers(1, &depth_renderbuffer_);
  glBindRenderbuffer(GL_RENDERBUFFER, depth_renderbuffer_);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24,
                        intrinsics_.width, intrinsics_.height);
  glBindRenderbuffer(GL_RENDERBUFFER, 0);

  glGenFramebuffers(1, &framebuffer_);
  glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                            GL_RENDERBUFFER, color_renderbuffer_);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                            GL_RENDERBUFFER, depth_renderbuffer_);
  success =
      glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  renderer_geometry_ptr_->DetachContext();
  gpu_resources_created_ = true;
  if (!success) {
    std::cerr << "Textured renderer framebuffer is incomplete" << std::endl;
    DeleteGpuResources();
  }
  return success;
}

void FullTexturedRenderer::DeleteGpuResources() {
  if (!gpu_resources_created_ || !renderer_geometry_ptr_ ||
      !renderer_geometry_ptr_->MakeContextCurrent()) {
    return;
  }
  glDeleteFramebuffers(1, &framebuffer_);
  glDeleteRenderbuffers(1, &color_renderbuffer_);
  glDeleteRenderbuffers(1, &depth_renderbuffer_);
  glDeleteTextures(1, &texture_);
  glDeleteBuffers(1, &vertex_buffer_);
  glDeleteVertexArrays(1, &vertex_array_);
  glDeleteProgram(shader_program_);
  renderer_geometry_ptr_->DetachContext();
  framebuffer_ = 0;
  color_renderbuffer_ = 0;
  depth_renderbuffer_ = 0;
  texture_ = 0;
  vertex_buffer_ = 0;
  vertex_array_ = 0;
  shader_program_ = 0;
  gpu_resources_created_ = false;
}

}  // namespace m3t_ros2
