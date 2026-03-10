# camera_manager

**Package:** `ros/src/sensor_component/camera_manager`
**Node name:** `camera_manager_node`
**Namespace:** `autodriver::camera`

---

## Overview

Multi-camera GStreamer NVMM pipeline manager for NVIDIA Jetson Orin. Captures frames
from up to 12 CSI or USB cameras, exports each frame as a DMABUF file descriptor,
and transmits it zero-copy to `tensorrt_inference_manager` via Unix socket IPC.
CPU never touches pixel data in the hot path.

---

## Startup Sequence

1. `LoadConfig()` — parse `cameras.yaml` and set parameters
2. `ConnectToInferenceManager()` — connect as IPC **client** to `/tmp/autodriver/frames.sock`
3. `StartCameras()` — construct one `CameraPipeline` per camera spec; stagger starts by 100 ms each to prevent I2C congestion on the CSI bus
4. Each pipeline calls `OnFrame()` from its GStreamer appsink thread on every captured frame

---

## Shutdown Sequence

1. Destructor calls `ShutdownCameras()` — sends EOS to each pipeline, joins threads
2. `CloseSocket(ipc_fd_)` — disconnects from inference_manager

---

## GStreamer Pipeline Topology

**Inference path only:**
```
nvarguscamerasrc sensor-id=N
  → capsfilter (NVMM NV12 @ WxH fps/1)
  → appsink name=inference_sink
```

**With debug stream enabled:**
```
nvarguscamerasrc sensor-id=N
  → capsfilter (NVMM NV12)
  → tee name=t
t. → queue(maxbuf=2, leaky=downstream) → appsink name=inference_sink
t. → queue(maxbuf=1, leaky=downstream) → nvjpegenc → appsink name=debug_sink
```

Custom `gst_source` strings from `cameras.yaml` replace `nvarguscamerasrc`.

---

## Frame Drop Policy

Each camera can be configured with a priority in `cameras.yaml`:

| Priority | Behavior |
|---|---|
| `HIGH` | Every frame forwarded (min_interval_ns = 0) |
| `NORMAL` | Frames throttled to target FPS (1 / fps interval) |
| Default | Same as NORMAL |

`ShouldSendFrame()` is wait-free: compares `now_ns - last_sent_ns` against `min_interval_ns`.

---

## IPC Role

`camera_manager` is the **client**. It connects (`CreateClientSocket`) to the Unix socket bound by `tensorrt_inference_manager`. Each `OnFrame()` call acquires `ipc_mutex_` (multiple appsink threads) and calls `ipc::SendFd(ipc_fd_, dmabuf_fd, meta)`.

---

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `cameras_yaml` | string | — | Absolute path to cameras.yaml |
| `ipc_socket_path` | string | `/tmp/autodriver/frames.sock` | Unix socket path |
| `debug_stream` | bool | `false` | Enable NVJPEG CompressedImage stream |

---

## Subscribed Topics

None.

---

## Published Topics

| Topic | Type | Condition |
|---|---|---|
| `/camera/{name}/camera_info` | `sensor_msgs/CameraInfo` | Always (if calibration file available) |
| `/camera/{name}/compressed` | `sensor_msgs/CompressedImage` | Only when `debug_stream = true` |

---

## Key Classes

### `CameraManager` (node)
- Owns the IPC client socket fd (`ipc_fd_`)
- Owns `std::unordered_map<uint32_t, std::unique_ptr<CameraPipeline>> pipelines_`
- `ipc_mutex_` serialises concurrent `SendFd` calls from multiple GST threads
- 1 Hz `metrics_timer_` logs FPS and drop counts

### `CameraPipeline`
- Manages one GStreamer pipeline
- `Start()` — constructs and starts the pipeline; spawns `bus_thread_`
- `Stop()` — sends EOS; joins `bus_thread_`
- Auto-restart on `GST_MESSAGE_ERROR`: up to `kMaxRestarts = 5` attempts with 500 ms delay
- `frames_captured_` / `frames_dropped_` are `std::atomic<uint64_t>`
- `IsHealthy()` returns false after all restart attempts exhausted

### `CameraConfig`
- `LoadCameraConfig(path)` — parses YAML into `CameraManagerConfig`
- Validates: id, name, fps range (1–120), resolution, `FrameDropPolicy`
- Resolves calibration file paths relative to the YAML file location

---

## Configuration Example (`cameras.yaml`)

```yaml
cameras:
  - id: 0
    name: "front_main"
    device: "/dev/video0"
    fps: 30
    width: 1920
    height: 1080
    format: "NV12"
    priority: HIGH
    calibration_file: "../calibration/front_main.yaml"

  - id: 1
    name: "front_wide"
    device: "/dev/video1"
    fps: 15
    width: 1280
    height: 720
    format: "NV12"
    priority: NORMAL
```

---

## Tests

| Test file | What it tests |
|---|---|
| `test/test_camera_config.cpp` | YAML parsing, all priority/resolution/policy combinations, error cases (missing file, malformed YAML, invalid id/name) |
| `test/test_camera_pipeline.cpp` | GStreamer pipeline string construction, initial state (healthy=true, 0 frames), accessor correctness |

Tests run without hardware (no GStreamer device opened; pipeline string built but not started).
