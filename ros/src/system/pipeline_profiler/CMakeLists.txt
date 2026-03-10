cmake_minimum_required(VERSION 3.14)
project(pipeline_profiler VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(autodriver_cmake REQUIRED)

include(AutodriverCompilerOptions)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# ---------------------------------------------------------------------------
# Library
# ---------------------------------------------------------------------------
add_library(pipeline_profiler_lib STATIC
  src/pipeline_profiler.cpp
)
autodriver_set_compiler_options(pipeline_profiler_lib)

target_include_directories(pipeline_profiler_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(pipeline_profiler_lib PUBLIC
  rclcpp std_msgs diagnostic_msgs)

# ---------------------------------------------------------------------------
# Executable
# ---------------------------------------------------------------------------
add_executable(pipeline_profiler_node
  src/pipeline_profiler_node.cpp
)
autodriver_set_compiler_options(pipeline_profiler_node)
target_link_libraries(pipeline_profiler_node PRIVATE pipeline_profiler_lib)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pipeline_profiler test/test_pipeline_profiler.cpp)
  autodriver_set_compiler_options(test_pipeline_profiler)
  target_link_libraries(test_pipeline_profiler PRIVATE pipeline_profiler_lib)
endif()

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS pipeline_profiler_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
