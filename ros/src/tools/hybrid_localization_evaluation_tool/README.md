# Hybrid Localization Evaluation Tool

ROS2 + C++17 + Qt6 desktop tool for evaluating `hybrid_localization` quality using
`/diagnostics` and output odometry streams. This package provides a real-time
dashboard, KPI aggregation, and CSV export for offline analysis.

## Inputs

- `/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`)
  - Reads `DiagnosticStatus.name == "hybrid_localization"` and parses key-value pairs.
- `/localization/kinematic_state` (`nav_msgs/msg/Odometry`)
  - Used for output availability and output rate KPI.

## Outputs

- GUI dashboard (Qt6) with:
  - Current localization status
  - Update rates and skip reasons
  - NIS summary cards
  - Delay summary
  - Output availability, rates, and process CPU usage
- Optional CSV export (enabled via parameter `csv_output_dir`)
  - Raw diagnostics frames: `hybrid_eval_raw_*.csv`
  - KPI snapshots: `hybrid_eval_kpi_*.csv`

## Parameters

- `window_sec` (double, default `10.0`)
  - KPI aggregation window length in seconds.
- `expected_output_rate_hz` (double, default `50.0`)
  - Expected output odom rate; used for availability ratio.
- `csv_output_dir` (string, default empty)
  - Output directory for CSV export. Empty disables export.

## Launch

```bash
ros2 launch hybrid_localization_evaluation_tool hybrid_localization_evaluation_tool.launch.xml
```

Launch arguments:

- `window_sec`
- `expected_output_rate_hz`
- `csv_output_dir`
- `diagnostics_topic`
- `output_odom_topic`

## KPI Algorithms (Summary)

1. **Update Rate**
   - For GNSS position, GNSS velocity, and heading:
     - `applied_ratio = applied_count / total_count`
     - `reason_hist` counts `*_update_reason` strings.

2. **NIS Summary**
   - For each NIS metric (`gnss_pos_nis`, `gnss_vel_nis`, `heading_yaw_nis`):
     - `mean`, `p95`, `max`, `count`.

3. **Delay Summary**
   - For delays (`gnss_delay`, `gnss_vel_delay`, `velocity_delay`, `steering_delay`):
     - `mean`, `p95`, `max`, `count`.

4. **Output Availability**
   - `output_count` = number of odom messages in window.
   - `expected_count = window_sec * expected_output_rate_hz`.
   - `availability_ratio = min(1.0, output_count / expected_count)`.
   - `last_age_sec` from most recent odom stamp to current time.

5. **Rates**
   - `diag_rate_hz = diag_samples / window_sec`
   - `output_rate_hz = output_count / window_sec`

## Tests

Run:

```bash
colcon test --packages-select hybrid_localization_evaluation_tool
colcon test-result --all
```

Tests cover:

- Diagnostic parsing of required keys.
- KPI aggregation and output availability math.
