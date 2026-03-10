# FindAutodriverCUDA.cmake
# ---------------------------------------------------------------------------
# Locates CUDA on Jetson Orin / JetPack 6.0 (aarch64) and generic x86_64.
#
# Wraps CMake's built-in CUDAToolkit module with Jetson-specific path hints
# and validates minimum version + target architecture.
#
# Imported target:
#   Autodriver::CUDA  — wraps CUDA::cudart, adds Jetson system include paths
#
# Cache variables (override if needed):
#   CUDAToolkit_ROOT        — Force a specific CUDA installation directory
#   AutodriverCUDA_ARCH     — GPU arch flag string (default: sm_87 for Orin)
#   AutodriverCUDA_MIN_VERSION — Minimum required CUDA version (default: 12.0)
#
# Variables set on success:
#   AutodriverCUDA_FOUND
#   AutodriverCUDA_VERSION
#   AutodriverCUDA_INCLUDE_DIRS
#   AutodriverCUDA_LIBRARIES
#
# Search strategy for CUDAToolkit_ROOT (first match wins):
#   1. User-supplied CUDAToolkit_ROOT cache variable
#   2. /usr/local/cuda  (symlink maintained by update-alternatives on Jetson)
#   3. Versioned paths /usr/local/cuda-12.{6,5,4,3,2,1,0}
#   4. CMake's built-in CUDAToolkit search paths
# ---------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.17)  # CUDAToolkit module requires CMake 3.17+

include(FindPackageHandleStandardArgs)

# ── Configurable defaults ─────────────────────────────────────────────────────
set(AutodriverCUDA_ARCH "sm_87" CACHE STRING
    "Target GPU architecture flag (sm_87 = Jetson Orin Ampere)")
set(AutodriverCUDA_MIN_VERSION "12.0" CACHE STRING
    "Minimum required CUDA toolkit version")

# ── Jetson path probing ───────────────────────────────────────────────────────
# Probe standard Jetson install locations only when the caller has not already
# set CUDAToolkit_ROOT. The first path that contains cuda_runtime.h wins.
if(NOT DEFINED CUDAToolkit_ROOT)
  set(_cuda_candidate_paths
    /usr/local/cuda          # canonical symlink (JetPack update-alternatives)
    /usr/local/cuda-12.6
    /usr/local/cuda-12.5
    /usr/local/cuda-12.4
    /usr/local/cuda-12.3
    /usr/local/cuda-12.2
    /usr/local/cuda-12.1
    /usr/local/cuda-12.0
  )
  foreach(_path IN LISTS _cuda_candidate_paths)
    if(EXISTS "${_path}/include/cuda_runtime.h")
      set(CUDAToolkit_ROOT "${_path}" CACHE PATH
          "CUDA toolkit root (auto-detected by FindAutodriverCUDA)")
      message(STATUS "FindAutodriverCUDA: auto-detected CUDAToolkit_ROOT=${_path}")
      break()
    endif()
  endforeach()
  unset(_cuda_candidate_paths)
  unset(_path)
endif()

# ── Delegate to CMake's CUDAToolkit finder ────────────────────────────────────
# Pass QUIET so our own error messaging takes over on failure.
find_package(CUDAToolkit ${AutodriverCUDA_MIN_VERSION} QUIET)

# ── Version gate ──────────────────────────────────────────────────────────────
if(CUDAToolkit_FOUND)
  set(AutodriverCUDA_VERSION "${CUDAToolkit_VERSION}")
  if(CUDAToolkit_VERSION VERSION_LESS AutodriverCUDA_MIN_VERSION)
    message(${AutodriverCUDA_FIND_REQUIRED_MSG}
      "FindAutodriverCUDA: CUDA >= ${AutodriverCUDA_MIN_VERSION} required; "
      "found ${CUDAToolkit_VERSION} at ${CUDAToolkit_ROOT}.")
    set(CUDAToolkit_FOUND FALSE)
  endif()
endif()

# ── Standard result handling ──────────────────────────────────────────────────
find_package_handle_standard_args(AutodriverCUDA
  REQUIRED_VARS
    CUDAToolkit_INCLUDE_DIRS
    CUDAToolkit_LIBRARY_DIR
  VERSION_VAR
    AutodriverCUDA_VERSION
  FAIL_MESSAGE
    "Could not find CUDA >= ${AutodriverCUDA_MIN_VERSION}. "
    "Set CUDAToolkit_ROOT to the CUDA installation directory."
)

# ── Imported target: Autodriver::CUDA ────────────────────────────────────────
if(AutodriverCUDA_FOUND AND NOT TARGET Autodriver::CUDA)
  add_library(Autodriver::CUDA INTERFACE IMPORTED)

  # Core: delegate to CUDA::cudart (provided by CUDAToolkit)
  target_link_libraries(Autodriver::CUDA INTERFACE CUDA::cudart)

  # Jetson-specific system include directories.
  # Marked SYSTEM so their warnings are suppressed in downstream targets.
  set(_jetson_system_includes
    /usr/local/cuda/include           # cuda_runtime.h et al.
    /usr/include/aarch64-linux-gnu    # Tegra multimedia headers
  )
  foreach(_inc IN LISTS _jetson_system_includes)
    if(EXISTS "${_inc}")
      target_include_directories(Autodriver::CUDA SYSTEM INTERFACE "${_inc}")
    endif()
  endforeach()
  unset(_jetson_system_includes)
  unset(_inc)

  # Compile definition: downstream code can #ifdef AUTODRIVER_CUDA_ARCH_SM_87
  # to guard Orin-specific paths (e.g. sysfs GPU load at /sys/devices/gpu.0/).
  string(REPLACE "sm_" "SM_" _arch_upper "${AutodriverCUDA_ARCH}")
  target_compile_definitions(Autodriver::CUDA INTERFACE
    "AUTODRIVER_CUDA_ARCH_${_arch_upper}"
  )
  unset(_arch_upper)

  set(AutodriverCUDA_INCLUDE_DIRS "${CUDAToolkit_INCLUDE_DIRS}")
  set(AutodriverCUDA_LIBRARIES    "CUDA::cudart")
endif()

mark_as_advanced(
  CUDAToolkit_ROOT
  AutodriverCUDA_ARCH
  AutodriverCUDA_MIN_VERSION
)
