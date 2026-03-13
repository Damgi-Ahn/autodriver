cmake_minimum_required(VERSION 3.14)
project(camera_manager VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake_auto REQUIRED)
find_package(autodriver_cmake REQUIRED)  # Injects CMAKE_MODULE_PATH

include(AutodriverCompilerOptions)
ament_auto_find_build_dependencies()  # Finds: rclcpp, sensor_msgs, ipc_unix_socket

# ── System dependencies ───────────────────────────────────────────────────────
find_package(AutodriverCUDA QUIET)
if(NOT AutodriverCUDA_FOUND)
  message(STATUS "gpu_watchdog: CUDA not found; skipping build of this package.")
  ament_auto_package()
  return()
endif()

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
  Autodriver::CUDA
  yaml-cpp
)

ament_target_dependencies(camera_manager_node
  rclcpp
  sensor_msgs
)

# ── Tests ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # test_camera_config — YAML parsing logic, no hardware required
  ament_add_gtest(test_camera_config
    test/test_camera_config.cpp
    src/camera_config.cpp
  )
  autodriver_set_compiler_options(test_camera_config)
  target_include_directories(test_camera_config PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_camera_config PRIVATE yaml-cpp)

  # test_camera_pipeline — pipeline string / initial state, no GStreamer device
  ament_add_gtest(test_camera_pipeline
    test/test_camera_pipeline.cpp
    src/camera_pipeline.cpp
    src/camera_config.cpp
  )
  autodriver_set_compiler_options(test_camera_pipeline)
  target_include_directories(test_camera_pipeline PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_camera_pipeline PRIVATE
    autodriver::ipc_unix_socket
    PkgConfig::GST
    NvBufSurface::NvBufSurface
    Autodriver::CUDA
    yaml-cpp)
  ament_target_dependencies(test_camera_pipeline rclcpp sensor_msgs)
endif()

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS camera_manager_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/  DESTINATION include)
install(DIRECTORY config/   DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/   DESTINATION share/${PROJECT_NAME}/launch)

ament_auto_package()
