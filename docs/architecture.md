# Autodriver — System Architecture

**Platform:** NVIDIA Jetson Orin (JetPack 6.0, aarch64)
**Framework:** ROS 2 (ament_cmake_auto)
**Language:** C++17

---

## 1. Design Goals

| Goal | Mechanism |
|---|---|
| Zero-copy frame path | NVMM DMABUF FDs passed over Unix socket; CPU never touches pixel data |
| GPU-resident pipeline | NvBufSurface pool pre-allocated; TRT operates on device memory end-to-end |
| Multi-camera scaling | Up to 12 CSI/USB cameras; independent GStreamer threads per camera |
| Multi-model batching | Model-centric queues; each model batches independently on its own CUDA stream |
| Observability | Per-stage latency profiler, GPU watchdog, 1 Hz metrics publisher |

---

## 2. Process Topology

```
┌─────────────────────────────────────────────────────────────────┐
│  Process: camera_manager_node                                   │
│                                                                 │
│  ┌──────────────┐  ┌──────────────┐      ┌──────────────┐      │
│  │ Camera 0     │  │ Camera 1     │  ... │ Camera 11    │      │
│  │ GST Thread   │  │ GST Thread   │      │ GST Thread   │      │
│  │ appsink NV12 │  │ appsink NV12 │      │ appsink NV12 │      │
│  └──────┬───────┘  └──────┬───────┘      └──────┬───────┘      │
│         └────────────┬────┘──────────────────────┘              │
│                      │  IPC write mutex                         │
│                      ▼                                          │
│            ipc::SendFd(dmabuf_fd, FrameMeta)                   │
│            Unix socket client → /tmp/autodriver/frames.sock    │
└─────────────────────────────┬───────────────────────────────────┘
                              │  Unix Domain Socket (SCM_RIGHTS)
                              │  FrameMeta (24 bytes) + DMABUF FD
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│  Process: tensorrt_inference_manager_node                       │
│                                                                 │
│  init_thread_: AcceptConnection → IpcReceiveLoop               │
│         │                                                       │
│         │  ipc::RecvFd()                                        │
│         ▼                                                       │
│  HybridScheduler::SubmitFrame()                                 │
│    ImportFd → AcquireSlot → CopyFromImport                      │
│    NvBufSurfaceDestroy → close(dmabuf_fd)                       │
│         │                                                       │
│         ▼  (routed by camera_id → model mapping)                │
│  ┌─────────────┐  ┌─────────────┐  ┌──────────────┐           │
│  │ YOLO Queue  │  │ Lane Queue  │  │  Seg Queue   │           │
│  │ batch=4     │  │ batch=2     │  │  batch=2     │           │
│  │ lockfree    │  │ lockfree    │  │  lockfree    │           │
│  └──────┬──────┘  └──────┬──────┘  └──────┬───────┘           │
│         └───────────┬────┘─────────────────┘                   │
│                     │  (per-model drain thread)                 │
│                     ▼                                           │
│  NvBufSurfacePool → BuildInputTensor → TRT enqueueV3()         │
│  cudaStreamSynchronize → OnInferenceResult → PublishResult     │
└─────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────┐
│  Process: gpu_watchdog_node        — sysfs + CUDA health     │
│  Process: metrics_manager_node     — 1 Hz aggregated metrics  │
│  Process: pipeline_profiler_node   — per-stage latency stats  │
└──────────────────────────────────────────────────────────────┘
```

---

## 3. Thread Model

| Thread | Owner | Count | Role |
|---|---|---|---|
| GST pipeline thread | camera_manager | 1 per camera (max 12) | GStreamer main loop + appsink callback |
| Camera stagger thread | camera_manager | 1 | Opens cameras with 100 ms delay each (I2C congestion prevention) |
| IPC init_thread_ | inference_manager | 1 | `AcceptConnection` (blocks) then `IpcReceiveLoop` |
| Model drain thread | inference_manager | 1 per model | Waits on queue, builds batch, runs TRT, publishes results |
| GPU poll thread | gpu_watchdog | 1 | Reads sysfs + CUDA at `poll_rate_hz` |
| ROS2 executor | all nodes | per rclcpp executor | Timer callbacks, topic publishing |

---

## 4. GPU Memory Architecture

```
GStreamer NVMM Buffer (camera capture)
        │
        │  NvBufSurface* (DMABUF fd export via SCM_RIGHTS)
        ▼
NvBufSurface Pool (128 pre-allocated slots)
  Allocator: NvBufSurfaceCreate(NVBUF_MEM_DEFAULT)
  Format: NV12
        │
        │  NvBufSurfaceCopy (device → device, no CPU)
        │  ImportFd + AcquireSlot + CopyFromImport
        ▼
TensorRT Batch Input Buffer
  Pinned CUDA memory (pre-allocated per model)
  Built via NvBufSurface → cudaMemcpyAsync (Y + UV planes)
        │
        │  context_->enqueueV3(stream_)
        ▼
TensorRT Output Buffer
  cudaMemcpyAsync → host pinned memory
        │
        │  result JSON serialised on CPU
        ▼
  ROS2 publish → /perception/{model}/results
```

**Rules:**
- NvBufSurface pool slot is released after TRT batch inference completes
- `NvBufSurfaceCreate/Destroy` never called in the hot path
- Each TRT model owns one CUDA stream from the pre-created pool of 8

---

## 5. IPC Data Flow (Zero-Copy DMABUF)

```
camera_manager side                    inference_manager side
──────────────────                     ──────────────────────
appsink new-sample callback
  │
  ├── map GstBuffer → NvBufSurface*
  ├── get DMABUF fd
  ├── fill FrameMeta {camera_id, timestamp, w, h, format}
  └── ipc::SendFd(ipc_fd_, dmabuf_fd, meta)
        sendmsg() + SCM_RIGHTS
                                         ipc::RecvFd(peer_fd_, &dmabuf_fd, &meta)
                                           recvmsg() + ancillary fd
                                           │
                                           ├── NvBufSurfacePool::ImportFd(dmabuf_fd)
                                           ├── pool_.AcquireSlot()
                                           ├── pool_.CopyFromImport(import_surf, slot)
                                           ├── NvBufSurfaceDestroy(import_surf)
                                           └── close(dmabuf_fd)
                                               ↓
                                           HybridScheduler → model queues
```

**Key property:** The DMABUF fd is live only during `SendFd` → `CopyFromImport`. After `CopyFromImport`, the original fd is closed and ownership is fully in the NvBufSurface pool.

---

## 6. Package Build Dependency Graph

```
ament_cmake (system)
        ↑
autodriver_cmake  ← provides FindAutodriverCUDA, FindNvBufSurface,
        ↑            FindTensorRT, AutodriverCompilerOptions
        │
        ├── ipc_unix_socket  (no ROS runtime deps; exports autodriver::ipc_unix_socket)
        │           ↑
        ├── camera_manager  (rclcpp, sensor_msgs, ipc_unix_socket, GStreamer, CUDA)
        │
        └── tensorrt_inference_manager  (rclcpp, sensor_msgs, std_msgs,
                    ↑                    ipc_unix_socket, TensorRT, CUDA, NvBufSurface)
                    │
        ├── metrics_manager    (rclcpp, std_msgs, diagnostic_msgs, CUDA)
        ├── pipeline_profiler  (rclcpp, std_msgs)
        ├── gpu_watchdog       (rclcpp, std_msgs, diagnostic_msgs, CUDA)
        └── autodriver_bringup (launch + config only)
```

---

## 7. ROS 2 Topic Map

| Topic | Type | Publisher | Consumers |
|---|---|---|---|
| `/perception/{model}/results` | `std_msgs/String` (JSON) | tensorrt_inference_manager | metrics_manager, downstream planners |
| `/camera/{name}/camera_info` | `sensor_msgs/CameraInfo` | camera_manager | downstream |
| `/camera/{name}/compressed` | `sensor_msgs/CompressedImage` | camera_manager (debug only) | visualization |
| `/system/metrics` | `diagnostic_msgs/DiagnosticArray` | metrics_manager | monitoring |
| `/system/watchdog/status` | `diagnostic_msgs/DiagnosticArray` | gpu_watchdog | monitoring |
| `/system/watchdog/alert` | `std_msgs/String` | gpu_watchdog (on level change) | alerting |
| `/system/profiler` | `std_msgs/Float32MultiArray` | pipeline_profiler | timing analysis |

---

## 8. Configuration Files

| File | Owner | Key Fields |
|---|---|---|
| `config/cameras.yaml` | camera_manager | id, name, device, fps, width, height, format, priority, calibration_file, gst_source |
| `config/models.yaml` | tensorrt_inference_manager | engine_path, batch_size, timeout_ms, input_tensor, output_tensor, camera_ids |
| `config/system.yaml` | autodriver_bringup (shared) | ipc_socket_path, num_cuda_streams, nvbuf_pool_size, metrics.publish_rate |

---

## 9. Launch Files

| Launch File | Role |
|---|---|
| `autodriver_bringup/launch/autodriver.launch.xml` | Full system: includes perception + monitoring |
| `autodriver_bringup/launch/perception.launch.xml` | camera_manager + tensorrt_inference_manager only |
| `autodriver_bringup/launch/monitoring.launch.xml` | gpu_watchdog + metrics_manager + pipeline_profiler only |
| `autodriver_bringup/launch/perception_system.launch.xml` | Thin wrapper → autodriver.launch.xml (backward compat) |

---

## 10. Key Design Decisions

| Decision | Rationale |
|---|---|
| Unix socket + SCM_RIGHTS vs shared memory | Simpler fd lifecycle; OS handles buffer GC; no SHM segment management |
| NvBufSurface pool (128 slots, not per-frame alloc) | `NvBufSurfaceCreate` is expensive; pool keeps it off the hot path |
| Model-centric queues | Decouples capture rate from batch formation; each model batches independently |
| Sequential camera open (100 ms stagger) | Prevents simultaneous I2C transactions on Jetson CSI bus |
| GStreamer `tee` for debug branch | Inference appsink path never blocked by JPEG encoding |
| `enqueueV3` (TRT10) | Required API for TensorRT 10; supports explicit batch dimensions |
| 8 CUDA streams | Allows pipeline overlap between batch N inference and N+1 preprocessing |
| `FindAutodriverCUDA.cmake` instead of bare CUDAToolkit | Jetson-specific path probing, version gate ≥12.0, `Autodriver::CUDA` imported target |
| `ament_cmake_auto` | Removes boilerplate `find_package(rclcpp)` lines; deps driven by package.xml |
| `autodriver_cmake` kept as `ament_cmake` | Uses `CONFIG_EXTRAS` for `CMAKE_MODULE_PATH` injection; incompatible with `ament_auto_package()` |
| Atomic CAS for profiler max_us | Plain `store` would be a data race on `std::atomic<uint64_t>` with relaxed ordering |
