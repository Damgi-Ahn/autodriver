cmake_minimum_required(VERSION 3.14)
project(gpu_watchdog VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake_auto REQUIRED)
find_package(autodriver_cmake REQUIRED)

include(AutodriverCompilerOptions)
ament_auto_find_build_dependencies()  # Finds: rclcpp, std_msgs, diagnostic_msgs

find_package(AutodriverCUDA REQUIRED)

# ---------------------------------------------------------------------------
# Library (shared with tests)
# ---------------------------------------------------------------------------
add_library(gpu_watchdog_lib STATIC
  src/gpu_watchdog.cpp
)
autodriver_set_compiler_options(gpu_watchdog_lib)

target_include_directories(gpu_watchdog_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(gpu_watchdog_lib PUBLIC Autodriver::CUDA)

ament_target_dependencies(gpu_watchdog_lib PUBLIC
  rclcpp std_msgs diagnostic_msgs)

# ---------------------------------------------------------------------------
# Executable
# ---------------------------------------------------------------------------
add_executable(gpu_watchdog_node
  src/gpu_watchdog_node.cpp
)
autodriver_set_compiler_options(gpu_watchdog_node)
target_link_libraries(gpu_watchdog_node PRIVATE gpu_watchdog_lib)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gpu_watchdog test/test_gpu_watchdog.cpp)
  autodriver_set_compiler_options(test_gpu_watchdog)
  target_link_libraries(test_gpu_watchdog PRIVATE gpu_watchdog_lib)
endif()

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS gpu_watchdog_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/  DESTINATION share/${PROJECT_NAME}/launch)

ament_auto_package()
