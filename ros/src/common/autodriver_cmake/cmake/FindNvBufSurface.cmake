# FindNvBufSurface.cmake
# ---------------------------------------------------------------------------
# Locates the NVIDIA NvBufSurface multimedia API on Jetson / JetPack 6.0.
#
# Imported target: NvBufSurface::NvBufSurface
#
# Variables set on success:
#   NvBufSurface_FOUND
#   NvBufSurface_INCLUDE_DIRS
#   NvBufSurface_LIBRARIES
#
# Search strategy (in order):
#   1. User-supplied NvBufSurface_ROOT cache variable
#   2. Standard JetPack 6.0 paths on aarch64
#   3. Generic /usr/include and /usr/lib fallbacks
# ---------------------------------------------------------------------------

include(FindPackageHandleStandardArgs)

# ── Header search ─────────────────────────────────────────────────────────────
find_path(NvBufSurface_INCLUDE_DIR
  NAMES nvbufsurface.h
  HINTS
    ${NvBufSurface_ROOT}/include
  PATHS
    /usr/include
    /usr/local/include
  PATH_SUFFIXES
    aarch64-linux-gnu
  DOC "NvBufSurface include directory"
)

# ── Library search ────────────────────────────────────────────────────────────
find_library(NvBufSurface_LIBRARY
  NAMES nvbufsurface
  HINTS
    ${NvBufSurface_ROOT}/lib
  PATHS
    /usr/lib
    /usr/local/lib
  PATH_SUFFIXES
    aarch64-linux-gnu
  DOC "NvBufSurface library"
)

find_library(NvBufSurfaceTransform_LIBRARY
  NAMES nvbufsurftransform
  HINTS
    ${NvBufSurface_ROOT}/lib
  PATHS
    /usr/lib
    /usr/local/lib
  PATH_SUFFIXES
    aarch64-linux-gnu
  DOC "NvBufSurface transform library (optional)"
)

# ── Version detection (JetPack stores version in /etc/nv_tegra_release) ──────
if(EXISTS "/etc/nv_tegra_release")
  file(READ "/etc/nv_tegra_release" _nv_release)
  string(REGEX MATCH "R([0-9]+)" _match "${_nv_release}")
  set(NvBufSurface_VERSION_MAJOR "${CMAKE_MATCH_1}")
endif()

# ── Standard result handling ──────────────────────────────────────────────────
find_package_handle_standard_args(NvBufSurface
  REQUIRED_VARS
    NvBufSurface_INCLUDE_DIR
    NvBufSurface_LIBRARY
  VERSION_VAR
    NvBufSurface_VERSION_MAJOR
)

if(NvBufSurface_FOUND AND NOT TARGET NvBufSurface::NvBufSurface)
  # Primary imported target
  add_library(NvBufSurface::NvBufSurface SHARED IMPORTED)
  set_target_properties(NvBufSurface::NvBufSurface PROPERTIES
    IMPORTED_LOCATION             "${NvBufSurface_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${NvBufSurface_INCLUDE_DIR}"
  )

  # Optional transform library
  if(NvBufSurfaceTransform_LIBRARY)
    add_library(NvBufSurface::Transform SHARED IMPORTED)
    set_target_properties(NvBufSurface::Transform PROPERTIES
      IMPORTED_LOCATION "${NvBufSurfaceTransform_LIBRARY}"
    )
    target_link_libraries(NvBufSurface::NvBufSurface
      INTERFACE NvBufSurface::Transform
    )
  endif()

  set(NvBufSurface_INCLUDE_DIRS "${NvBufSurface_INCLUDE_DIR}")
  set(NvBufSurface_LIBRARIES    "${NvBufSurface_LIBRARY}")
endif()

mark_as_advanced(
  NvBufSurface_INCLUDE_DIR
  NvBufSurface_LIBRARY
  NvBufSurfaceTransform_LIBRARY
)
