cmake_minimum_required(VERSION 3.14)
project(pipeline_profiler VERSION 0.1.0 LANGUAGES CXX)

list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../cmake")

include(AutodriverCompilerOptions)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)


add_executable(pipeline_profiler_node
  src/pipeline_profiler_node.cpp
  src/pipeline_profiler.cpp
)
autodriver_set_compiler_options(pipeline_profiler_node)

target_include_directories(pipeline_profiler_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)




ament_target_dependencies(pipeline_profiler_node rclcpp std_msgs diagnostic_msgs)

install(TARGETS pipeline_profiler_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/  DESTINATION include)
install(DIRECTORY launch/   DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
