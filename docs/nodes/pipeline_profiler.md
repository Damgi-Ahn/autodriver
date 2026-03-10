# pipeline_profiler

**Package:** `ros/src/system/pipeline_profiler`
**Node name:** `pipeline_profiler_node`
**Namespace:** `autodriver::system`

---

## Overview

Per-stage latency profiler for the autodriver perception pipeline. Any pipeline
thread calls `record(StageTimestamp)` to log the nanosecond timestamp at which
each stage completed for a given frame. The node aggregates inter-stage gap
statistics over a publish window and emits them as a `Float32MultiArray` on
`/system/profiler`.

All statistics accumulation is lock-free (atomic operations only). `record()` is
safe to call from any thread concurrently.

---

## Pipeline Stages

```
CAMERA_CAPTURE (0) — frame exits GStreamer appsink
     ↓ gap[0]
IPC_SEND       (1) — DMABUF fd sent over Unix socket
     ↓ gap[1]
IPC_RECV       (2) — DMABUF fd received by inference_manager
     ↓ gap[2]
SCHEDULER      (3) — frame enqueued into model queue
     ↓ gap[3]
BATCH_BUILD    (4) — batch assembled, TRT input tensor filled
     ↓ gap[4]
INFERENCE      (5) — enqueueV3 + cudaStreamSynchronize complete
     ↓ gap[5]
PUBLISH        (6) — ROS 2 result topic published
```

6 inter-stage gaps total (`kNumStages - 1 = 6`).

---

## `StageTimestamp` Structure

```cpp
struct StageTimestamp {
  uint64_t frame_id{0};
  uint64_t camera_id{0};
  std::array<uint64_t, kNumStages> ts{};  // ns; 0 = stage not reached
};
```

Stages that were not reached for a frame are left as 0 and are skipped in `record()`.

---

## Statistics Model

For each gap `i = ts[i+1] - ts[i]` (in µs):

- `count` — number of frames that reached both stage i and i+1
- `sum_us` — total latency sum for windowed average
- `max_us` — running maximum (CAS loop for correctness)

After each publish, all three accumulators are atomically reset to 0 via
`exchange(0)` — no cumulative drift across windows.

---

## Published Topic

| Topic | Type | Frequency | Layout |
|---|---|---|---|
| `/system/profiler` | `std_msgs/Float32MultiArray` | `publish_rate_hz` | `[avg_gap0, max_gap0, avg_gap1, max_gap1, ...]` in µs |

Array has `2 * (kNumStages - 1) = 12` elements. `data[2*i]` = avg µs for gap i,
`data[2*i+1]` = max µs for gap i.

---

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `publish_rate_hz` | double | `1.0` | Publish frequency |

---

## Subscribed Topics

None.

---

## Usage

Instrument pipeline stages by calling `record()` after each stage completes:

```cpp
// In camera_manager appsink callback:
StageTimestamp ts;
ts.frame_id  = frame_counter++;
ts.camera_id = camera_id;
ts.ts[static_cast<size_t>(Stage::CAMERA_CAPTURE)] = PipelineProfiler::now_ns();
// ... send IPC ...
ts.ts[static_cast<size_t>(Stage::IPC_SEND)] = PipelineProfiler::now_ns();
profiler_->record(ts);

// In inference_manager IPC receive:
ts.ts[static_cast<size_t>(Stage::IPC_RECV)] = PipelineProfiler::now_ns();
profiler_->record(ts);
```

`record()` skips any gap where either endpoint timestamp is 0, so partial
`StageTimestamp` objects (where later stages have not been reached yet) are safe.

---

## Implementation Notes

- `max_us` update uses a CAS loop (`compare_exchange_weak`) to avoid data races
  when multiple threads call `record()` concurrently with relaxed ordering.
- `now_ns()` uses `CLOCK_MONOTONIC` via `clock_gettime()`.

---

## Tests

`test/test_pipeline_profiler.cpp` — covers:
- Single-stage gap recording
- Multi-gap frame
- Zero-timestamp skipping
- Window reset (accumulators cleared after publish)
- Concurrent `record()` from multiple threads
- `stage_name()` string correctness for all stages
