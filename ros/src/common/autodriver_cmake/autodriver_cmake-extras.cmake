# autodriver_cmake-extras.cmake
# ---------------------------------------------------------------------------
# Automatically included by autodriver_camakeConfig.cmake when downstream
# packages call find_package(autodriver_cmake REQUIRED).
#
# Effect: adds this package's cmake/ directory to CMAKE_MODULE_PATH so that
# downstream CMakeLists.txt can use:
#   include(AutodriverCompilerOptions)
#   find_package(NvBufSurface REQUIRED)
#   find_package(TensorRT REQUIRED)
# without knowing the install prefix.
#
# ${autodriver_cmake_DIR} is set by find_package to the directory containing
# autodriver_camakeConfig.cmake, which is share/autodriver_cmake/cmake/.
# Our module files are installed to the same directory.
# ---------------------------------------------------------------------------
list(APPEND CMAKE_MODULE_PATH "${autodriver_cmake_DIR}")
