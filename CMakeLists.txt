cmake_minimum_required(VERSION 3.8)
project(m3t_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# M3T + its transitive dependencies (the imported target references these)
find_package(Eigen3 REQUIRED)
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_package(M3T REQUIRED)

set(ROS_DEPS
  rclcpp std_msgs sensor_msgs geometry_msgs tf2 tf2_ros tf2_eigen std_srvs
  visualization_msgs cv_bridge image_transport)

add_executable(m3t_tracker_node src/tracker_node.cpp)
target_include_directories(m3t_tracker_node PRIVATE include)
ament_target_dependencies(m3t_tracker_node ${ROS_DEPS})
target_link_libraries(m3t_tracker_node m3t)

add_executable(m3t_image_publisher_node src/image_publisher_node.cpp)
target_include_directories(m3t_image_publisher_node PRIVATE include)
ament_target_dependencies(m3t_image_publisher_node ${ROS_DEPS})
target_link_libraries(m3t_image_publisher_node m3t)

add_executable(m3t_synthetic_source_node
  src/synthetic_source_node.cpp
  src/textured_renderer.cpp)
target_include_directories(m3t_synthetic_source_node PRIVATE include)
ament_target_dependencies(m3t_synthetic_source_node ${ROS_DEPS})
target_link_libraries(m3t_synthetic_source_node m3t)

install(TARGETS
        m3t_tracker_node
        m3t_image_publisher_node
        m3t_synthetic_source_node
        DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/m3t_smoke_test
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz config assets DESTINATION share/${PROJECT_NAME})

ament_package()
