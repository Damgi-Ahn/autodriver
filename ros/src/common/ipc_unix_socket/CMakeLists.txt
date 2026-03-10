cmake_minimum_required(VERSION 3.14)
project(ipc_unix_socket VERSION 0.1.0 LANGUAGES CXX)

# Prepend the workspace-level cmake module directory so shared Find* modules
# and compiler-option helpers are resolved before any system paths.
list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../cmake")

include(AutodriverCompilerOptions)

find_package(ament_cmake REQUIRED)

# ── Library ───────────────────────────────────────────────────────────────────
add_library(${PROJECT_NAME} SHARED
  src/ipc_unix_socket.cpp
)
autodriver_set_compiler_options(${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ── Tests ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_${PROJECT_NAME}
    test/test_ipc_unix_socket.cpp
  )
  autodriver_set_compiler_options(test_${PROJECT_NAME})
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
  target_include_directories(test_${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
endif()

# ── Install ───────────────────────────────────────────────────────────────────
install(
  TARGETS ${PROJECT_NAME}
  EXPORT  ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)
install(
  EXPORT ${PROJECT_NAME}Targets
  FILE   ${PROJECT_NAME}Targets.cmake
  NAMESPACE autodriver::
  DESTINATION lib/cmake/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
