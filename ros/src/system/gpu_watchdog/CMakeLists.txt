cmake_minimum_required(VERSION 3.14)
project(gpu_watchdog VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(autodriver_cmake REQUIRED)

include(AutodriverCompilerOptions)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)


add_executable(gpu_watchdog_node
  src/gpu_watchdog_node.cpp
  src/gpu_watchdog.cpp
)
autodriver_set_compiler_options(gpu_watchdog_node)

target_include_directories(gpu_watchdog_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)



ament_target_dependencies(gpu_watchdog_node rclcpp std_msgs diagnostic_msgs)

install(TARGETS gpu_watchdog_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/  DESTINATION include)
install(DIRECTORY launch/   DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
