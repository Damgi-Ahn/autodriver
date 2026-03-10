cmake_minimum_required(VERSION 3.14)
project(tensorrt_inference_manager VERSION 0.1.0 LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── ROS2 ──────────────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ipc_unix_socket REQUIRED)

# ── CUDA ──────────────────────────────────────────────────────────────────────
find_package(CUDAToolkit REQUIRED)

# ── TensorRT 10 (JetPack 6.0) ─────────────────────────────────────────────────
find_path(TENSORRT_INCLUDE_DIR
  NAMES NvInfer.h
  PATHS /usr/include /usr/local/include
  PATH_SUFFIXES aarch64-linux-gnu
)
find_library(TENSORRT_LIBRARY
  NAMES nvinfer
  PATHS /usr/lib /usr/local/lib
  PATH_SUFFIXES aarch64-linux-gnu
)
find_library(TENSORRT_PLUGIN_LIBRARY
  NAMES nvinfer_plugin
  PATHS /usr/lib /usr/local/lib
  PATH_SUFFIXES aarch64-linux-gnu
)
find_library(TENSORRT_ONNXPARSER_LIBRARY
  NAMES nvonnxparser
  PATHS /usr/lib /usr/local/lib
  PATH_SUFFIXES aarch64-linux-gnu
)
if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_LIBRARY)
  message(FATAL_ERROR "TensorRT not found. Ensure JetPack 6.0 TensorRT is installed.")
endif()

# ── NvBufSurface ──────────────────────────────────────────────────────────────
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

# ── yaml-cpp ──────────────────────────────────────────────────────────────────
find_package(yaml-cpp REQUIRED)

# ── Executable ───────────────────────────────────────────────────────────────
add_executable(tensorrt_inference_manager_node
  src/inference_manager_node.cpp
  src/inference_manager.cpp
  src/model_runner.cpp
  src/nvbuf_pool.cpp
  src/scheduler.cpp
)

target_include_directories(tensorrt_inference_manager_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${TENSORRT_INCLUDE_DIR}
    ${NVBUFSURFACE_INCLUDE_DIR}
    ${CUDAToolkit_INCLUDE_DIRS}
)

target_link_libraries(tensorrt_inference_manager_node
  PRIVATE
    autodriver::ipc_unix_socket
    ${TENSORRT_LIBRARY}
    ${TENSORRT_PLUGIN_LIBRARY}
    ${TENSORRT_ONNXPARSER_LIBRARY}
    ${NVBUFSURFACE_LIBRARY}
    CUDA::cudart
    yaml-cpp
)

target_compile_options(tensorrt_inference_manager_node PRIVATE -Wall -Wextra -O2)

ament_target_dependencies(tensorrt_inference_manager_node
  rclcpp
  sensor_msgs
  std_msgs
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS tensorrt_inference_manager_node
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
