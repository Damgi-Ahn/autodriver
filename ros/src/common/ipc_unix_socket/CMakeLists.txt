cmake_minimum_required(VERSION 3.14)
project(ipc_unix_socket VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)

# ── Library ───────────────────────────────────────────────────────────────────
add_library(${PROJECT_NAME} SHARED
  src/ipc_unix_socket.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -O2)

# ── Install ───────────────────────────────────────────────────────────────────
install(
  TARGETS ${PROJECT_NAME}
  EXPORT  ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  EXPORT ${PROJECT_NAME}Targets
  FILE   ${PROJECT_NAME}Targets.cmake
  NAMESPACE autodriver::
  DESTINATION lib/cmake/${PROJECT_NAME}
)

# ── Ament export ──────────────────────────────────────────────────────────────
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
