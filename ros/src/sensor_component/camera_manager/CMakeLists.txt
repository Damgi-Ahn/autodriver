cmake_minimum_required(VERSION 3.14)
project(camera_manager VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(autodriver_cmake REQUIRED)  # Injects CMAKE_MODULE_PATH

include(AutodriverCompilerOptions)

# ── ROS2 ──────────────────────────────────────────────────────────────────────
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ipc_unix_socket REQUIRED)

# ── System dependencies ───────────────────────────────────────────────────────
find_package(CUDAToolkit REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED IMPORTED_TARGET
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-video-1.0
)
find_package(yaml-cpp REQUIRED)
find_package(NvBufSurface REQUIRED)

# ── Executable ────────────────────────────────────────────────────────────────
add_executable(camera_manager_node
  src/camera_manager_node.cpp
  src/camera_manager.cpp
  src/camera_pipeline.cpp
  src/camera_config.cpp
)
autodriver_set_compiler_options(camera_manager_node)

target_include_directories(camera_manager_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(camera_manager_node PRIVATE
  autodriver::ipc_unix_socket
  PkgConfig::GST
  NvBufSurface::NvBufSurface
  CUDA::cudart
  yaml-cpp
)

ament_target_dependencies(camera_manager_node
  rclcpp
  sensor_msgs
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS camera_manager_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/  DESTINATION include)
install(DIRECTORY config/   DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/   DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
