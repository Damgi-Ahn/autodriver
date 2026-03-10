cmake_minimum_required(VERSION 3.14)
project(camera_manager VERSION 0.1.0 LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── ROS2 dependencies ─────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ipc_unix_socket REQUIRED)

# ── CUDA ──────────────────────────────────────────────────────────────────────
find_package(CUDAToolkit REQUIRED)

# ── GStreamer ─────────────────────────────────────────────────────────────────
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-video-1.0
)

# ── yaml-cpp ──────────────────────────────────────────────────────────────────
find_package(yaml-cpp REQUIRED)

# ── NvBufSurface (JetPack multimedia API) ─────────────────────────────────────
find_path(NVBUFSURFACE_INCLUDE_DIR
  NAMES nvbufsurface.h
  PATHS /usr/include /usr/local/include
  PATH_SUFFIXES aarch64-linux-gnu
)
find_library(NVBUFSURFACE_LIBRARY
  NAMES nvbufsurface
  PATHS /usr/lib /usr/local/lib
  PATH_SUFFIXES aarch64-linux-gnu
)
if(NOT NVBUFSURFACE_INCLUDE_DIR OR NOT NVBUFSURFACE_LIBRARY)
  message(FATAL_ERROR "NvBufSurface not found. Ensure JetPack 6.0 is installed.")
endif()

# ── Executable ───────────────────────────────────────────────────────────────
add_executable(camera_manager_node
  src/camera_manager_node.cpp
  src/camera_manager.cpp
  src/camera_pipeline.cpp
  src/camera_config.cpp
)

target_include_directories(camera_manager_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${GST_INCLUDE_DIRS}
    ${NVBUFSURFACE_INCLUDE_DIR}
    ${CUDAToolkit_INCLUDE_DIRS}
)

target_link_libraries(camera_manager_node
  PRIVATE
    autodriver::ipc_unix_socket
    ${GST_LIBRARIES}
    ${NVBUFSURFACE_LIBRARY}
    CUDA::cudart
    yaml-cpp
)

target_compile_options(camera_manager_node PRIVATE
  -Wall -Wextra -O2
  ${GST_CFLAGS_OTHER}
)

ament_target_dependencies(camera_manager_node
  rclcpp
  sensor_msgs
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS camera_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
