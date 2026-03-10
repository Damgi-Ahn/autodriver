cmake_minimum_required(VERSION 3.14)
project(tensorrt_inference_manager VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(autodriver_cmake REQUIRED)

include(AutodriverCompilerOptions)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ipc_unix_socket REQUIRED)

find_package(CUDAToolkit REQUIRED)
find_package(TensorRT REQUIRED)
if(DEFINED TensorRT_VERSION_MAJOR AND TensorRT_VERSION_MAJOR LESS 10)
  message(FATAL_ERROR
    "TensorRT 10+ required (enqueueV3 API). Found: ${TensorRT_VERSION}")
endif()
find_package(NvBufSurface REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(tensorrt_inference_manager_node
  src/inference_manager_node.cpp
  src/inference_manager.cpp
  src/model_runner.cpp
  src/nvbuf_pool.cpp
  src/scheduler.cpp
)
autodriver_set_compiler_options(tensorrt_inference_manager_node)

target_include_directories(tensorrt_inference_manager_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(tensorrt_inference_manager_node PRIVATE
  autodriver::ipc_unix_socket
  TensorRT::nvinfer
  TensorRT::nvinfer_plugin
  TensorRT::nvonnxparser
  NvBufSurface::NvBufSurface
  CUDA::cudart
  yaml-cpp
)

ament_target_dependencies(tensorrt_inference_manager_node
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS tensorrt_inference_manager_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/  DESTINATION include)
install(DIRECTORY config/   DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/   DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
