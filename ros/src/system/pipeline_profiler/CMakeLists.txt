cmake_minimum_required(VERSION 3.14)
project(pipeline_profiler VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(pipeline_profiler_node
  src/pipeline_profiler_node.cpp
  src/pipeline_profiler.cpp
)

target_include_directories(pipeline_profiler_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(pipeline_profiler_node PRIVATE -Wall -Wextra -O2)

ament_target_dependencies(pipeline_profiler_node
  rclcpp
  std_msgs
)

install(TARGETS pipeline_profiler_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
