# autodriver_cmake

**Package:** `ros/src/common/autodriver_cmake`
**Type:** CMake helper package (no compiled artifacts)
**Build system:** `ament_cmake` (kept as-is; uses `CONFIG_EXTRAS` for `CMAKE_MODULE_PATH` injection)

---

## Overview

Provides shared CMake modules for the entire autodriver ROS 2 workspace.
After `find_package(autodriver_cmake REQUIRED)`, all modules are automatically
available via `CMAKE_MODULE_PATH` — no additional `include()` calls needed for
the Find modules.

---

## Provided Modules

### `AutodriverCompilerOptions.cmake`

Defines `autodriver_set_compiler_options(target)` — applies project-wide compiler
flags to a target:
- C++17 standard
- Strict warning flags (`-Wall -Wextra -Wpedantic`)
- Release optimisations when `CMAKE_BUILD_TYPE=Release`

Usage:
```cmake
include(AutodriverCompilerOptions)
add_executable(my_node src/my_node.cpp)
autodriver_set_compiler_options(my_node)
```

---

### `FindAutodriverCUDA.cmake`

Jetson Orin-aware CUDA detection. Probes versioned paths (`/usr/local/cuda-12.x`)
before falling back to CMake's built-in `CUDAToolkit`, enforces a minimum version
(default: 12.0), and creates the `Autodriver::CUDA` imported target.

`Autodriver::CUDA` wraps `CUDA::cudart` and adds:
- Jetson system include paths (`/usr/local/cuda/include`, `/usr/include/aarch64-linux-gnu`)
  as `SYSTEM` includes (suppresses upstream warnings)
- Compile definition `AUTODRIVER_CUDA_ARCH_SM_87` (Jetson Orin Ampere)

**Cache variables:**

| Variable | Default | Description |
|---|---|---|
| `CUDAToolkit_ROOT` | auto-detected | Override CUDA install directory |
| `AutodriverCUDA_ARCH` | `sm_87` | Target GPU architecture |
| `AutodriverCUDA_MIN_VERSION` | `12.0` | Minimum acceptable CUDA version |

Usage:
```cmake
find_package(AutodriverCUDA REQUIRED)
target_link_libraries(my_target PRIVATE Autodriver::CUDA)
```

---

### `FindNvBufSurface.cmake`

Locates NVIDIA NvBufSurface multimedia API on Jetson / JetPack 6.0.

Creates imported targets:
- `NvBufSurface::NvBufSurface` — `libnvbufsurface`
- `NvBufSurface::Transform` — `libnvbufsurftransform` (optional, linked as INTERFACE)

Usage:
```cmake
find_package(NvBufSurface REQUIRED)
target_link_libraries(my_target PRIVATE NvBufSurface::NvBufSurface)
```

---

### `FindTensorRT.cmake`

Locates TensorRT 10 libraries on Jetson / JetPack 6.0.

Creates imported targets:
- `TensorRT::nvinfer`
- `TensorRT::nvinfer_plugin`
- `TensorRT::nvonnxparser`

Usage:
```cmake
find_package(TensorRT REQUIRED)
target_link_libraries(my_target PRIVATE TensorRT::nvinfer)
```

---

## How CMAKE_MODULE_PATH Injection Works

`autodriver_cmake-extras.cmake` is registered as a `CONFIG_EXTRAS` file in
`ament_package()`. When any downstream package calls
`find_package(autodriver_cmake REQUIRED)`, ament executes:

```cmake
list(APPEND CMAKE_MODULE_PATH "${autodriver_cmake_DIR}")
```

where `autodriver_cmake_DIR` resolves to `share/autodriver_cmake/cmake/` in the
install tree. All four Find modules are in that directory and immediately usable.

---

## Downstream Usage Pattern

Every package that needs hardware-specific deps follows this pattern:

```cmake
find_package(ament_cmake_auto REQUIRED)
find_package(autodriver_cmake REQUIRED)       # injects CMAKE_MODULE_PATH

include(AutodriverCompilerOptions)
ament_auto_find_build_dependencies()          # finds ROS deps from package.xml

find_package(AutodriverCUDA REQUIRED)         # uses FindAutodriverCUDA.cmake
find_package(NvBufSurface REQUIRED)           # uses FindNvBufSurface.cmake
find_package(TensorRT REQUIRED)               # uses FindTensorRT.cmake

add_executable(my_node src/my_node.cpp)
autodriver_set_compiler_options(my_node)
target_link_libraries(my_node PRIVATE Autodriver::CUDA NvBufSurface::NvBufSurface)
ament_auto_package()
```

Note: `autodriver_cmake` itself stays as `ament_cmake` (not `ament_cmake_auto`)
because `ament_auto_package()` does not support the `CONFIG_EXTRAS` argument
required for `CMAKE_MODULE_PATH` injection.
