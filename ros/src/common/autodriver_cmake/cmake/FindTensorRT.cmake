# FindTensorRT.cmake
# ---------------------------------------------------------------------------
# Locates NVIDIA TensorRT 10 on Jetson Orin / JetPack 6.0.
#
# Imported targets:
#   TensorRT::nvinfer          (core inference library)
#   TensorRT::nvinfer_plugin   (built-in plugin registry)
#   TensorRT::nvonnxparser     (ONNX model parser)
#
# Variables set:
#   TensorRT_FOUND
#   TensorRT_VERSION           (major.minor.patch from NvInferVersion.h)
#   TensorRT_INCLUDE_DIRS
#
# Cache variables for user overrides:
#   TensorRT_ROOT              (top-level install prefix)
#
# JetPack 6.0 default locations:
#   headers: /usr/include/aarch64-linux-gnu/
#   libs:    /usr/lib/aarch64-linux-gnu/
# ---------------------------------------------------------------------------

include(FindPackageHandleStandardArgs)

# ── Header ────────────────────────────────────────────────────────────────────
find_path(TensorRT_INCLUDE_DIR
  NAMES NvInfer.h
  HINTS
    ${TensorRT_ROOT}/include
  PATHS
    /usr/include
    /usr/local/include
  PATH_SUFFIXES
    aarch64-linux-gnu
  DOC "TensorRT include directory"
)

# ── Core library ──────────────────────────────────────────────────────────────
find_library(TensorRT_nvinfer_LIBRARY
  NAMES nvinfer
  HINTS
    ${TensorRT_ROOT}/lib
  PATHS
    /usr/lib
    /usr/local/lib
  PATH_SUFFIXES
    aarch64-linux-gnu
  DOC "TensorRT core library (nvinfer)"
)

find_library(TensorRT_nvinfer_plugin_LIBRARY
  NAMES nvinfer_plugin
  HINTS ${TensorRT_ROOT}/lib
  PATHS /usr/lib /usr/local/lib
  PATH_SUFFIXES aarch64-linux-gnu
  DOC "TensorRT plugin library"
)

find_library(TensorRT_nvonnxparser_LIBRARY
  NAMES nvonnxparser
  HINTS ${TensorRT_ROOT}/lib
  PATHS /usr/lib /usr/local/lib
  PATH_SUFFIXES aarch64-linux-gnu
  DOC "TensorRT ONNX parser library"
)

# ── Version detection from NvInferVersion.h ───────────────────────────────────
if(TensorRT_INCLUDE_DIR AND EXISTS "${TensorRT_INCLUDE_DIR}/NvInferVersion.h")
  file(STRINGS "${TensorRT_INCLUDE_DIR}/NvInferVersion.h" _trt_ver_lines
       REGEX "#define NV_TENSORRT_(MAJOR|MINOR|PATCH)")
  foreach(_line ${_trt_ver_lines})
    if(_line MATCHES "NV_TENSORRT_MAJOR ([0-9]+)")
      set(TensorRT_VERSION_MAJOR "${CMAKE_MATCH_1}")
    elseif(_line MATCHES "NV_TENSORRT_MINOR ([0-9]+)")
      set(TensorRT_VERSION_MINOR "${CMAKE_MATCH_1}")
    elseif(_line MATCHES "NV_TENSORRT_PATCH ([0-9]+)")
      set(TensorRT_VERSION_PATCH "${CMAKE_MATCH_1}")
    endif()
  endforeach()
  set(TensorRT_VERSION
    "${TensorRT_VERSION_MAJOR}.${TensorRT_VERSION_MINOR}.${TensorRT_VERSION_PATCH}")
endif()

# ── Result ────────────────────────────────────────────────────────────────────
find_package_handle_standard_args(TensorRT
  REQUIRED_VARS
    TensorRT_INCLUDE_DIR
    TensorRT_nvinfer_LIBRARY
  VERSION_VAR
    TensorRT_VERSION
)

# ── Imported targets ──────────────────────────────────────────────────────────
if(TensorRT_FOUND)
  set(TensorRT_INCLUDE_DIRS "${TensorRT_INCLUDE_DIR}")

  if(NOT TARGET TensorRT::nvinfer)
    add_library(TensorRT::nvinfer SHARED IMPORTED)
    set_target_properties(TensorRT::nvinfer PROPERTIES
      IMPORTED_LOCATION             "${TensorRT_nvinfer_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${TensorRT_INCLUDE_DIR}"
    )
  endif()

  if(TensorRT_nvinfer_plugin_LIBRARY AND NOT TARGET TensorRT::nvinfer_plugin)
    add_library(TensorRT::nvinfer_plugin SHARED IMPORTED)
    set_target_properties(TensorRT::nvinfer_plugin PROPERTIES
      IMPORTED_LOCATION "${TensorRT_nvinfer_plugin_LIBRARY}"
    )
  endif()

  if(TensorRT_nvonnxparser_LIBRARY AND NOT TARGET TensorRT::nvonnxparser)
    add_library(TensorRT::nvonnxparser SHARED IMPORTED)
    set_target_properties(TensorRT::nvonnxparser PROPERTIES
      IMPORTED_LOCATION "${TensorRT_nvonnxparser_LIBRARY}"
    )
  endif()
endif()

mark_as_advanced(
  TensorRT_INCLUDE_DIR
  TensorRT_nvinfer_LIBRARY
  TensorRT_nvinfer_plugin_LIBRARY
  TensorRT_nvonnxparser_LIBRARY
)
