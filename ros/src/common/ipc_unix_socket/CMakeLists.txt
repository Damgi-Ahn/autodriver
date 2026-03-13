cmake_minimum_required(VERSION 3.14)
project(ipc_unix_socket VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake_auto REQUIRED)
find_package(autodriver_cmake REQUIRED)

include(AutodriverCompilerOptions)
ament_auto_find_build_dependencies()

# ── Library ───────────────────────────────────────────────────────────────────
ament_auto_add_library(${PROJECT_NAME} SHARED
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
ament_auto_package(
  INSTALL_TO_SHARE
  include
)
