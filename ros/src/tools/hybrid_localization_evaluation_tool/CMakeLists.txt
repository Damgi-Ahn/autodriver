cmake_minimum_required(VERSION 3.14)
project(hybrid_localization_evaluation_tool VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake_auto REQUIRED)
find_package(autodriver_cmake REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Widgets Charts)

include(AutodriverCompilerOptions)
ament_auto_find_build_dependencies()  # Finds: rclcpp, diagnostic_msgs

# ---------------------------------------------------------------------------
# Library
# ---------------------------------------------------------------------------
add_library(hybrid_localization_evaluation_tool_lib STATIC
  src/diagnostic_parser.cpp
  src/evaluation_node.cpp
  src/kpi_engine.cpp
  src/alert_engine.cpp
  src/session_store.cpp
  src/ros_qt_bridge.cpp
  src/storage_exporter.cpp
  src/health_state_engine.cpp
  src/ground_truth_analyzer.cpp
  src/main_window.cpp
  src/ui/overview_tab.cpp
  src/ui/fusion_quality_tab.cpp
  src/ui/covariance_noise_tab.cpp
  src/ui/sensor_timing_tab.cpp
  src/ui/pose_trajectory_tab.cpp
  src/ui/events_inspector_tab.cpp
  src/ui/export_session_tab.cpp
  src/ui/ground_truth_error_tab.cpp
  include/hybrid_localization_evaluation_tool/ui/overview_tab.hpp
  include/hybrid_localization_evaluation_tool/ui/fusion_quality_tab.hpp
  include/hybrid_localization_evaluation_tool/ui/covariance_noise_tab.hpp
  include/hybrid_localization_evaluation_tool/ui/sensor_timing_tab.hpp
  include/hybrid_localization_evaluation_tool/ui/pose_trajectory_tab.hpp
  include/hybrid_localization_evaluation_tool/ui/events_inspector_tab.hpp
  include/hybrid_localization_evaluation_tool/ui/export_session_tab.hpp
  include/hybrid_localization_evaluation_tool/ui/ground_truth_error_tab.hpp
)
autodriver_set_compiler_options(hybrid_localization_evaluation_tool_lib)
set_target_properties(hybrid_localization_evaluation_tool_lib PROPERTIES AUTOMOC ON)

target_include_directories(hybrid_localization_evaluation_tool_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(hybrid_localization_evaluation_tool_lib PUBLIC
  Qt6::Widgets
  Qt6::Charts
)

ament_target_dependencies(hybrid_localization_evaluation_tool_lib PUBLIC
  rclcpp
  diagnostic_msgs
  nav_msgs
  geometry_msgs
  std_msgs
)

# ---------------------------------------------------------------------------
# Executable
# ---------------------------------------------------------------------------
add_executable(hybrid_localization_evaluation_tool
  src/main.cpp
)
autodriver_set_compiler_options(hybrid_localization_evaluation_tool)
set_target_properties(hybrid_localization_evaluation_tool PROPERTIES AUTOMOC ON)
target_link_libraries(hybrid_localization_evaluation_tool PRIVATE
  hybrid_localization_evaluation_tool_lib
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS hybrid_localization_evaluation_tool DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/  DESTINATION share/${PROJECT_NAME}/launch)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_auto_add_gtest(test_diagnostic_parser test/test_diagnostic_parser.cpp)
  autodriver_set_compiler_options(test_diagnostic_parser)
  target_link_libraries(test_diagnostic_parser hybrid_localization_evaluation_tool_lib)

  ament_auto_add_gtest(test_kpi_engine test/test_kpi_engine.cpp)
  autodriver_set_compiler_options(test_kpi_engine)
  target_link_libraries(test_kpi_engine hybrid_localization_evaluation_tool_lib)
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
