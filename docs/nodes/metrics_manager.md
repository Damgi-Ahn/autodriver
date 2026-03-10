# metrics_manager

**Package:** `ros/src/system/metrics_manager`
**Node name:** `metrics_manager_node`
**Namespace:** `autodriver::system`

---

## Overview

System metrics aggregator for the autodriver perception pipeline. Collects camera
FPS/drop counts, per-model inference latency and batch statistics, GPU utilisation,
memory, and temperature. Publishes a `diagnostic_msgs/DiagnosticArray` at 1 Hz.

Also performs topic stall detection: any subscribed `/perception/*/results` topic
that has not received a message within `stall_threshold_s` is flagged in diagnostics.

---

## Data Sources

| Data | Source mechanism |
|---|---|
| Camera FPS / drops | External push: `UpdateCameraMetrics(CameraMetrics)` |
| Model latency / batches | Auto-subscription to `/perception/{name}/results` |
| GPU utilisation, memory | `cudaMemGetInfo()` + `/sys/devices/gpu.0/load` |
| GPU temperature | `/sys/class/thermal/thermal_zone1/temp` |

---

## Model Result Parsing

`OnModelResult` receives a `std_msgs/String` with a JSON payload. It extracts
fields via `std::string::find` (no external JSON dependency):

```json
{"latency_ms": 12.3, "frames": 4, "boxes": [...]}
```

Latency is smoothed with an EMA (α = 0.1):
```
avg_latency_ms = 0.9 * prev + 0.1 * new_latency_ms
```

---

## Topic Stall Detection

`CheckTopicHealth()` is called each publish cycle. For each subscribed model topic:
- If `now_ns - last_msg_ns > stall_threshold_s * 1e9` and `healthy == true`:
  - Sets `healthy = false`, increments `stall_count`
  - Logs `RCLCPP_WARN`
- If a message arrives: `healthy = true`, `last_msg_ns = now_ns`

First message also transitions `healthy` from `false` → `true`.

---

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `publish_rate_hz` | double | `1.0` | DiagnosticArray publish frequency |
| `stall_threshold_s` | double | `5.0` | Topic silence duration before stall flag |
| `model_names` | string[] | `[]` | List of model names to auto-subscribe |

---

## Published Topics

| Topic | Type | Frequency | Description |
|---|---|---|---|
| `/system/metrics` | `diagnostic_msgs/DiagnosticArray` | `publish_rate_hz` | Full pipeline health summary |

---

## Subscribed Topics

| Topic | Type | Description |
|---|---|---|
| `/perception/{name}/results` | `std_msgs/String` | Auto-subscribed per entry in `model_names` |

---

## DiagnosticArray Layout

One `DiagnosticStatus` per subsystem:

| Status name | Contents |
|---|---|
| `gpu` | utilization_pct, memory_used_mb, memory_total_mb, temperature_c |
| `camera/{id}` | fps, frames_captured, frames_dropped |
| `model/{name}` | avg_latency_ms, batches_run, avg_batch_size |
| `topic_health/{name}` | healthy (true/false), stall_count |

---

## Thread Safety

`camera_metrics_`, `model_metrics_`, and `topic_health_` are protected by `metrics_mutex_`.
`UpdateCameraMetrics()` and `UpdateModelMetrics()` acquire the mutex and update in place.
`CollectAndPublish()` (timer callback) acquires the mutex to snapshot all data.

---

## Tests

`test/test_metrics_manager.cpp` — covers parameter loading, EMA latency smoothing,
topic stall detection timing, GPU metrics structure, and DiagnosticArray key presence.
