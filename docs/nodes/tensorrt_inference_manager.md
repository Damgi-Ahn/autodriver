# tensorrt_inference_manager

**Package:** `ros/src/perception/tensorrt_inference_manager`
**Node name:** `tensorrt_inference_manager_node`
**Namespace:** `autodriver::inference`

---

## Overview

Multi-model TensorRT 10 inference manager for Jetson Orin. Receives DMABUF frames
from `camera_manager` over a Unix socket, routes them to per-model queues, builds
batches, and runs inference via `IExecutionContext::enqueueV3`. Results are
published as JSON strings on `/perception/{model}/results`.

---

## Startup Sequence

1. `LoadConfigs()` — parse `models.yaml` (model specs) and `system.yaml` (pool/stream counts)
2. `InitCudaStreams()` — create N CUDA streams (default: 8)
3. `InitNvBufPool()` — `NvBufSurfaceCreate` × pool_size (default: 128 slots)
4. `InitModelRunners()` — deserialise TRT engines, derive buffer sizes, allocate device memory
5. `InitScheduler()` — register model queues, create per-model drain threads
6. `BindIpcServer()` — `CreateServerSocket` (non-blocking bind + listen)
7. `init_thread_` — `AcceptAndReceive()`: blocks on `AcceptConnection`, then runs `IpcReceiveLoop`

The constructor returns after step 6. The node is immediately spinnable. Camera connection arrives asynchronously in `init_thread_`.

---

## Shutdown Sequence

1. Destructor calls `Shutdown()`
2. `ipc_running_ = false`; close `peer_fd_` → `IpcReceiveLoop` exits (reads `kPeerClosed`)
3. Join `init_thread_`
4. `scheduler_->Stop()` — drain threads exit and are joined
5. Destroy CUDA streams
6. `CloseServerSocket(server_fd_, ipc_socket_path_)` — closes fd + unlinks socket file

---

## IPC Role

`tensorrt_inference_manager` is the **server**. It binds the Unix socket and waits for `camera_manager` to connect. This is intentional: the inference node starts first (engine deserialisation takes longer) and the camera node retries connection with backoff.

`IpcReceiveLoop` receives `FrameMeta + DMABUF fd` in a tight loop:
```
while ipc_running_:
    RecvFd(peer_fd_, &dmabuf_fd, &meta)
    if kPeerClosed → break
    scheduler_->SubmitFrame(dmabuf_fd, meta)
```

---

## Frame Ingestion (DMABUF → Pool)

`HybridScheduler::SubmitFrame` performs:
1. `NvBufSurfacePool::ImportFd(dmabuf_fd)` — `NvBufSurfaceImportFromFd`
2. `pool_.AcquireSlot(timeout_ms)` — spin-wait on free list
3. `pool_.CopyFromImport(import_surf, slot)` — `NvBufSurfaceCopy` device→device
4. `NvBufSurfaceDestroy(import_surf)` — release the imported surface
5. `close(dmabuf_fd)` — release file descriptor
6. Route `QueuedFrame{pool_slot, meta, timestamp}` to matching model queues

Pool slot is released after TRT batch completes in `DrainLoop`.

---

## Hybrid Scheduler Design

```
SubmitPreImported(QueuedFrame)
        │
        ▼
Camera → Model Router  (configured in models.yaml: camera_ids per model)
        │
   ┌────▼────────────────────────────────────────┐
   │ Per-model queue (lock-free, capacity = 16)  │
   └────┬────────────────────────────────────────┘
        │  DrainLoop (one thread per model)
        │
        ├── fire when: queue.size() >= batch_size
        │   OR timer fires (timeout_ms since first enqueue)
        │
        ▼
   ModelRunner::RunBatch(frames)
   → BuildInputTensor (NvBufSurface → cudaMemcpyAsync)
   → setTensorAddress + enqueueV3(stream)
   → cudaStreamSynchronize
   → OnInferenceResult → PublishResult
```

Batch timeout prevents starvation when not enough frames arrive to fill a full batch.

---

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `models_yaml` | string | — | Path to models.yaml |
| `system_yaml` | string | — | Path to system.yaml |
| `ipc_socket_path` | string | `/tmp/autodriver/frames.sock` | Unix socket path |
| `num_cuda_streams` | int | `8` | Number of pre-created CUDA streams |
| `nvbuf_pool_size` | int | `128` | NvBufSurface pool slot count |

---

## Subscribed Topics

None.

---

## Published Topics

| Topic | Type | Description |
|---|---|---|
| `/perception/{model_name}/results` | `std_msgs/String` | JSON: `{"latency_ms": 12.3, "frames": 4, "boxes": [...]}` |

One publisher created per model at startup.

---

## Key Classes

### `InferenceManager` (node)
- Owns `server_fd_` + `peer_fd_`; `init_thread_` for non-blocking accept
- Owns `cuda_streams_`, `nvbuf_pool_`, `model_runners_`, `scheduler_`
- `result_pubs_`: per-model string publishers

### `HybridScheduler`
- `AddModelQueue(name, batch_size, timeout_ms, camera_ids)`
- `SubmitFrame(dmabuf_fd, meta)` — full DMABUF import → pool → route
- `SubmitPreImported(QueuedFrame)` — test seam (bypasses GPU import)
- Per-model `DrainLoop` thread: waits for batch ready, calls callback

### `ModelRunner`
- Deserialises TRT engine from `.engine` file (relative to models.yaml dir)
- `DeriveBufferSizes()` — `getTensorShape()` with dynamic dim (-1) handling
- `BuildInputTensor(frames, stream)` — NV12 Y+UV → TRT input buffer via `cudaMemcpyAsync`
- `RunBatch(frames)` — `setTensorAddress` per tensor + `enqueueV3` + `cudaStreamSynchronize`
- `AvgLatencyMs()` — EMA-smoothed per-batch inference time

### `NvBufSurfacePool`
- `Init(size, width, height, format)` — `NvBufSurfaceCreate` × size
- `ImportFd(dmabuf_fd)` — `NvBufSurfaceImportFromFd`; static, no pool slot consumed
- `CopyFromImport(src, slot)` — `NvBufSurfaceCopy` device→device
- `AcquireSlot(timeout_ms)` — spin-wait with deadline; returns `kInvalidSlot` on timeout
- `ReleaseSlot(slot)` — returns slot to free list

---

## Configuration Example (`models.yaml`)

```yaml
models:
  - name: yolo_v8
    engine_path: engines/yolo_v8n.engine
    batch_size: 4
    timeout_ms: 50
    input_tensor: images
    output_tensor: output0
    camera_ids: [0, 1, 2, 3]

  - name: lane_detector
    engine_path: engines/lane_v2.engine
    batch_size: 2
    timeout_ms: 33
    input_tensor: input
    output_tensor: lanes
    camera_ids: [0, 1]
```

---

## Tests

| Test file | What it tests |
|---|---|
| `test/test_scheduler.cpp` | Queue routing by camera_id, batch-size fire, timeout partial batch, multi-model routing, concurrent submissions, stop-with-pending |
| `test/test_nvbuf_pool.cpp` | Free-list accounting, slot range validity, exhaustion behaviour, acquire-after-release, concurrent producer/consumer stress |

Tests use `MockPool` / `TestablePool` subclasses that override GPU calls, so they run on any host without Jetson hardware.
