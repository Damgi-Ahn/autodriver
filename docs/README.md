# Autodriver Documentation

**Platform:** NVIDIA Jetson Orin (JetPack 6.0)
**Framework:** ROS 2 / ament_cmake_auto
**Language:** C++17

---

## Documents

### System Design

- [**architecture.md**](architecture.md) — Full system design: process topology, thread model, GPU memory flow, IPC data flow, topic map, config files, launch hierarchy, key decisions

### ROS 2 Nodes

| Document | Node | Role |
|---|---|---|
| [camera_manager.md](nodes/camera_manager.md) | `camera_manager_node` | Multi-camera GStreamer NVMM capture + IPC sender |
| [tensorrt_inference_manager.md](nodes/tensorrt_inference_manager.md) | `tensorrt_inference_manager_node` | TRT10 multi-model batched inference + IPC server |
| [gpu_watchdog.md](nodes/gpu_watchdog.md) | `gpu_watchdog_node` | GPU health monitor (stall, thermal, memory) |
| [metrics_manager.md](nodes/metrics_manager.md) | `metrics_manager_node` | 1 Hz pipeline health aggregator |
| [pipeline_profiler.md](nodes/pipeline_profiler.md) | `pipeline_profiler_node` | Lock-free per-stage latency profiler |

### Libraries & Build Infrastructure

| Document | Package | Role |
|---|---|---|
| [ipc_unix_socket.md](libraries/ipc_unix_socket.md) | `ipc_unix_socket` | Zero-copy DMABUF FD transport (SCM_RIGHTS) |
| [autodriver_cmake.md](libraries/autodriver_cmake.md) | `autodriver_cmake` | Shared CMake modules: FindAutodriverCUDA, FindNvBufSurface, FindTensorRT, compiler options |

---

## Quick Start

### Build

```bash
cd ros
colcon build --symlink-install
source install/setup.bash
```

### Launch Full System

```bash
ros2 launch autodriver_bringup autodriver.launch.xml \
  cameras_yaml:=/path/to/cameras.yaml \
  models_yaml:=/path/to/models.yaml \
  system_yaml:=/path/to/system.yaml
```

### Launch Perception Only

```bash
ros2 launch autodriver_bringup perception.launch.xml \
  cameras_yaml:=/path/to/cameras.yaml \
  models_yaml:=/path/to/models.yaml \
  system_yaml:=/path/to/system.yaml
```

### Launch Monitoring Only

```bash
ros2 launch autodriver_bringup monitoring.launch.xml
```

---

## Package Dependency Order

```
autodriver_cmake
    ↑
    ├── ipc_unix_socket
    ├── camera_manager       (+ GStreamer, CUDA, NvBufSurface)
    ├── tensorrt_inference_manager  (+ TensorRT, CUDA, NvBufSurface)
    ├── gpu_watchdog         (+ CUDA)
    ├── metrics_manager      (+ CUDA)
    ├── pipeline_profiler
    └── autodriver_bringup   (launch + config only)
```

---

## Repository Structure

```
autodriver/
├── docs/                        ← This documentation
│   ├── README.md
│   ├── architecture.md
│   ├── nodes/
│   │   ├── camera_manager.md
│   │   ├── tensorrt_inference_manager.md
│   │   ├── gpu_watchdog.md
│   │   ├── metrics_manager.md
│   │   └── pipeline_profiler.md
│   └── libraries/
│       ├── ipc_unix_socket.md
│       └── autodriver_cmake.md
└── ros/src/
    ├── common/
    │   ├── autodriver_cmake/
    │   └── ipc_unix_socket/
    ├── sensor_component/
    │   └── camera_manager/
    ├── perception/
    │   └── tensorrt_inference_manager/
    ├── system/
    │   ├── gpu_watchdog/
    │   ├── metrics_manager/
    │   └── pipeline_profiler/
    └── autodriver_bringup/
```
