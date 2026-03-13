cmake_minimum_required(VERSION 3.14)
project(hybrid_localization_evaluation_tool VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake_auto REQUIRED)
find_package(autodriver_cmake REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Widgets)

include(AutodriverCompilerOptions)
ament_auto_find_build_dependencies()  # Finds: rclcpp, diagnostic_msgs

# ---------------------------------------------------------------------------
# Library
# ---------------------------------------------------------------------------
add_library(hybrid_localization_evaluation_tool_lib STATIC
  src/diagnostic_parser.cpp
  src/evaluation_node.cpp
  src/kpi_engine.cpp
  src/main_window.cpp
  src/ros_qt_bridge.cpp
  src/storage_exporter.cpp
)
autodriver_set_compiler_options(hybrid_localization_evaluation_tool_lib)

target_include_directories(hybrid_localization_evaluation_tool_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(hybrid_localization_evaluation_tool_lib PUBLIC
  Qt6::Widgets
)

ament_target_dependencies(hybrid_localization_evaluation_tool_lib PUBLIC
  rclcpp
  diagnostic_msgs
  nav_msgs
)

# ---------------------------------------------------------------------------
# Executable
# ---------------------------------------------------------------------------
add_executable(hybrid_localization_evaluation_tool
  src/main.cpp
)
autodriver_set_compiler_options(hybrid_localization_evaluation_tool)
target_link_libraries(hybrid_localization_evaluation_tool PRIVATE
  hybrid_localization_evaluation_tool_lib
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS hybrid_localization_evaluation_tool DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_diagnostic_parser test/test_diagnostic_parser.cpp)
  autodriver_set_compiler_options(test_diagnostic_parser)
  target_link_libraries(test_diagnostic_parser PRIVATE hybrid_localization_evaluation_tool_lib)

  ament_add_gtest(test_kpi_engine test/test_kpi_engine.cpp)
  autodriver_set_compiler_options(test_kpi_engine)
  target_link_libraries(test_kpi_engine PRIVATE hybrid_localization_evaluation_tool_lib)
endif()

ament_auto_package()
