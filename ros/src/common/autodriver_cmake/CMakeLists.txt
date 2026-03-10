cmake_minimum_required(VERSION 3.14)
project(autodriver_cmake)

find_package(ament_cmake REQUIRED)

# ── Install shared cmake modules ──────────────────────────────────────────────
# Downstream packages resolve these via ${autodriver_cmake_DIR} which ament
# sets to share/autodriver_cmake/cmake/ after find_package(autodriver_cmake).
install(
  DIRECTORY cmake/
  DESTINATION share/${PROJECT_NAME}/cmake
)

# ── Package export ─────────────────────────────────────────────────────────────
# CONFIG_EXTRAS injects list(APPEND CMAKE_MODULE_PATH ...) into the generated
# autodriver_camakeConfig.cmake so all downstream packages gain access to
# FindNvBufSurface, FindTensorRT, and AutodriverCompilerOptions immediately
# after find_package(autodriver_cmake REQUIRED).
ament_package(
  CONFIG_EXTRAS "autodriver_cmake-extras.cmake"
)
