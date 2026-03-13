cmake_minimum_required(VERSION 3.14)
project(pipeline_profiler VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake_auto REQUIRED)
find_package(autodriver_cmake REQUIRED)

include(AutodriverCompilerOptions)
ament_auto_find_build_dependencies()  # Finds: rclcpp, std_msgs

# ---------------------------------------------------------------------------
# Library
# ---------------------------------------------------------------------------
ament_auto_add_library(pipeline_profiler_lib STATIC
  src/pipeline_profiler.cpp
)
autodriver_set_compiler_options(pipeline_profiler_lib)

target_include_directories(pipeline_profiler_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---------------------------------------------------------------------------
# Executable
# ---------------------------------------------------------------------------
ament_auto_add_executable(pipeline_profiler_node
  src/pipeline_profiler_node.cpp
)
autodriver_set_compiler_options(pipeline_profiler_node)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_auto_add_gtest(test_pipeline_profiler test/test_pipeline_profiler.cpp)
  autodriver_set_compiler_options(test_pipeline_profiler)
  target_link_libraries(test_pipeline_profiler pipeline_profiler_lib)
  target_include_directories(test_pipeline_profiler PRIVATE include)
endif()

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
)
