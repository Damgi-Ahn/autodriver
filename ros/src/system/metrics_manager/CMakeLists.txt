cmake_minimum_required(VERSION 3.14)
project(metrics_manager VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(autodriver_cmake REQUIRED)

include(AutodriverCompilerOptions)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(CUDAToolkit REQUIRED)

# ---------------------------------------------------------------------------
# Library
# ---------------------------------------------------------------------------
add_library(metrics_manager_lib STATIC
  src/metrics_manager.cpp
)
autodriver_set_compiler_options(metrics_manager_lib)

target_include_directories(metrics_manager_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(metrics_manager_lib PUBLIC CUDA::cudart)

ament_target_dependencies(metrics_manager_lib PUBLIC
  rclcpp std_msgs diagnostic_msgs)

# ---------------------------------------------------------------------------
# Executable
# ---------------------------------------------------------------------------
add_executable(metrics_manager_node
  src/metrics_manager_node.cpp
)
autodriver_set_compiler_options(metrics_manager_node)
target_link_libraries(metrics_manager_node PRIVATE metrics_manager_lib)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_metrics_manager test/test_metrics_manager.cpp)
  autodriver_set_compiler_options(test_metrics_manager)
  target_link_libraries(test_metrics_manager PRIVATE metrics_manager_lib)
endif()

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS metrics_manager_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
