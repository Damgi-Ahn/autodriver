# AutodriverCompilerOptions.cmake
# ---------------------------------------------------------------------------
# Shared compiler option definitions for all autodriver ROS2 packages.
# Include via: include(AutodriverCompilerOptions)
# Requires: CMAKE_MODULE_PATH to include this file's directory.
# ---------------------------------------------------------------------------

function(autodriver_set_compiler_options target)
  target_compile_options(${target} PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Wno-unused-parameter   # ROS2 callbacks often have unused params
    -Wcast-align
    -Wformat=2
    $<$<CONFIG:Release>:-O2>
    $<$<CONFIG:Debug>:-g -O0>
  )
  target_compile_features(${target} PUBLIC cxx_std_17)
endfunction()
