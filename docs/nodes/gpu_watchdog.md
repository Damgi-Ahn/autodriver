# gpu_watchdog

**Package:** `ros/src/system/gpu_watchdog`
**Node name:** `gpu_watchdog_node`
**Namespace:** `autodriver::system`

---

## Overview

GPU health monitor for NVIDIA Jetson Orin. Polls hardware state from sysfs and
CUDA at a configurable rate, detects anomalies (stall, memory pressure, thermal
throttling), and publishes diagnostic messages. Raises an alert topic only on
`AlertLevel` transitions to minimise downstream noise.

---

## Startup

Constructor:
1. Declares parameters and reads config
2. Creates publishers for `/system/watchdog/status` and `/system/watchdog/alert`
3. Sets `running_ = true`, starts `poll_thread_`

---

## Detection Logic (`Evaluate`)

`Evaluate(GpuSnapshot)` is public for unit testing without hardware. It is called
each poll cycle with data from `ReadSnapshot()`.

| Condition | Detection | Level |
|---|---|---|
| `gpu_load_pct == 0` for `> stall_threshold_s` | GPU stall (pipeline dead) | `kError` |
| `gpu_temp_c > throttle_temp_c` OR `throttling == true` | Thermal throttling | `kWarn` |
| `gpu_mem_used_kb / gpu_mem_total_kb > mem_warn_pct / 100` | Memory pressure | `kWarn` |
| None of the above | Normal | `kOk` |

Error outranks Warn. Stall accumulator resets when `gpu_load_pct > 0`.

---

## Sysfs Paths (Jetson Orin / JetPack 6)

| Reading | Path | Scale |
|---|---|---|
| GPU load | `/sys/devices/gpu.0/load` | 0–1000 per-mille → divided by 10 → `[0, 100]` |
| GPU temp | `/sys/class/thermal/thermal_zone1/temp` | milli-°C → divided by 1000 |
| CPU temp | `/sys/class/thermal/thermal_zone0/temp` | milli-°C → divided by 1000 |
| GPU memory | `cudaMemGetInfo()` | bytes |

---

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `poll_rate_hz` | double | `2.0` | Polling frequency |
| `stall_threshold_s` | double | `5.0` | Zero-load duration before kError |
| `mem_warn_pct` | double | `90.0` | Memory usage % threshold for kWarn |
| `throttle_temp_c` | double | `85.0` | GPU temperature °C threshold for kWarn |

---

## Published Topics

| Topic | Type | Frequency | Description |
|---|---|---|---|
| `/system/watchdog/status` | `diagnostic_msgs/DiagnosticArray` | Every poll cycle | Full GPU snapshot as diagnostics |
| `/system/watchdog/alert` | `std_msgs/String` | On level change only | `"OK"` / `"WARN"` / `"ERROR"` |

---

## Subscribed Topics

None.

---

## DiagnosticArray Layout

Each status message contains one `DiagnosticStatus` with:
- `name`: `"gpu_watchdog"`
- `level`: 0=OK, 1=WARN, 2=ERROR
- `message`: human-readable summary
- `values`: key/value pairs for `gpu_load_pct`, `gpu_temp_c`, `cpu_temp_c`, `gpu_mem_used_kb`, `gpu_mem_total_kb`, `throttling`, `stall_accumulator_s`

---

## Tests

`test/test_gpu_watchdog.cpp` — 11 test cases covering:
- Nominal OK with non-zero load
- Zero load accumulation → kError after stall threshold
- Stall reset when load returns
- Thermal warn / OK boundary
- Memory warn at 90% threshold
- Throttling flag
- Zero total memory guard

Tests inject `GpuSnapshot` directly into `Evaluate()` — no sysfs or CUDA access needed.
