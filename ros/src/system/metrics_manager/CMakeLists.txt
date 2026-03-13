cmake_minimum_required(VERSION 3.14)
project(metrics_manager VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake_auto REQUIRED)
find_package(autodriver_cmake REQUIRED)

include(AutodriverCompilerOptions)
ament_auto_find_build_dependencies()  # Finds: rclcpp, std_msgs, diagnostic_msgs


find_package(AutodriverCUDA QUIET)
if(NOT AutodriverCUDA_FOUND)
  message(STATUS "metrics_manager: CUDA not found; skipping build of this package.")
  ament_auto_package()
  return()
endif()

# ---------------------------------------------------------------------------
# Library
# ---------------------------------------------------------------------------
ament_auto_add_library(metrics_manager_lib STATIC
  src/metrics_manager.cpp
)
autodriver_set_compiler_options(metrics_manager_lib)

target_include_directories(metrics_manager_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(metrics_manager_lib PUBLIC Autodriver::CUDA)

# ---------------------------------------------------------------------------
# Executable
# ---------------------------------------------------------------------------
ament_auto_add_executable(metrics_manager_node
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
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  include
)