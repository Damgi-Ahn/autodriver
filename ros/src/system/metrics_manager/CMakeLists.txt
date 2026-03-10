cmake_minimum_required(VERSION 3.14)
project(metrics_manager VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(autodriver_cmake REQUIRED)

include(AutodriverCompilerOptions)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(CUDAToolkit REQUIRED)

add_executable(metrics_manager_node
  src/metrics_manager_node.cpp
  src/metrics_manager.cpp
)
autodriver_set_compiler_options(metrics_manager_node)

target_include_directories(metrics_manager_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(metrics_manager_node PRIVATE CUDA::cudart)

ament_target_dependencies(metrics_manager_node rclcpp std_msgs diagnostic_msgs)

install(TARGETS metrics_manager_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/  DESTINATION include)
install(DIRECTORY launch/   DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
