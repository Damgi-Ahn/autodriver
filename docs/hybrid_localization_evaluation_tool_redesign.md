# Hybrid Localization Evaluation Tool Redesign Spec (for Claude)

## Purpose
Redesign `hybrid_localization_evaluation_tool` into a commercial-grade evaluation and debugging suite for the `hybrid_localization` stack. The output should be a clear implementation prompt/specification for Claude, with feature list, UX/UI layout, data processing requirements, and acceptance criteria.

## Scope
- Redesign the tool only. Do not change `hybrid_localization` runtime behavior unless explicitly called out as optional enhancements.
- Must ingest existing ROS2 topics and diagnostics from `hybrid_localization`.
- Must support real-time UI + offline analysis (CSV/ROS bag). Offline mode can be “Phase 2” but should be specified.

## Current System Analysis

### hybrid_localization_evaluation_tool (current)
**Architecture**
- ROS2 node + Qt6 GUI in one process.
- `EvaluationNode` subscribes to diagnostics and pose topics, sends snapshots to `RosQtBridge`.
- `KpiEngine` aggregates metrics in a sliding window.
- `StorageExporter` writes `raw` + `kpi` CSV.

**Data inputs used**
- `/diagnostics` → parse `DiagnosticStatus.name == "hybrid_localization"`.
- `/localization/kinematic_state` → output odom stamps for output rate/availability.
- `/sensing/gnss/pose_with_covariance` → GNSS pose card.
- `/localization/pose_twist_fusion_filter/pose` → FGO pose card.
- `/localization/kinematic_state` (again) → ESKF pose card.

**UI**
- Single main window with sections: status, KPIs, NIS, delay, reason hist, timestamps, CPU, pose cards, charts, raw diagnostics dump.
- Charts are basic line charts (NIS, delays).

**CSV output**
- Raw CSV: limited diagnostic keys.
- KPI CSV: limited KPI snapshot.

**Current limitations**
- Only a subset of diagnostics are parsed and displayed.
- No clear notion of alerting or anomaly detection.
- No per-sensor timing quality summary (IMU dt stats are published but not used).
- No visibility into covariance inflation, R/Q values, or FGO stats.
- No historical timeline or event annotations.
- Minimal configuration and no offline session loading.

### hybrid_localization (data availability)
**Key outputs (topics)**
- `/diagnostics` with name `hybrid_localization` and rich key/value diagnostics.
- `/localization/kinematic_state` (output odom)
- `/localization/pose_twist_fusion_filter/pose` (FGO pose)
- `/localization/kinematic_state_gnss` (GNSS odom)
- `/localization/fgo/keyframe_path` (FGO keyframe path)
- `/localization/state` (text status)
- `/sensing/imu/imu_data` (preprocessed IMU)
- `/sensing/gnss/velocity_status`
- `/sensing/gnss/pose_with_covariance`

**Diagnostic keys published** (from `EskfDiagnosticsPublisher`)
- `is_activated`
- `imu_count`, `gnss_count`, `gnss_vel_count`, `velocity_count`, `steering_count`
- `eskf_initialized`
- `gnss_pos_update_applied`, `gnss_pos_update_reason`, `gnss_pos_nis`, `gnss_pos_residual_norm`
- `gnss_vel_update_applied`, `gnss_vel_update_reason`, `gnss_vel_nis`, `gnss_vel_residual_norm`
- `heading_yaw_update_applied`, `heading_yaw_update_reason`, `heading_yaw_nis`, `heading_yaw_residual_rad`
- `gnss_status`
- `gnss_pos_R_xx`, `gnss_pos_R_yy`, `gnss_pos_R_zz`, `gnss_pos_status_inflate`, `gnss_pos_nis_inflate`
- `gnss_vel_R_xx`, `gnss_vel_R_yy`, `gnss_vel_R_zz`, `gnss_vel_status_inflate`, `gnss_vel_nis_inflate`
- `heading_yaw_var`, `heading_yaw_nis_inflate`, `heading_yaw_var_eff`, `heading_yaw_var_applied`,
  `heading_status_inflate`, `heading_recover_inflate`, `heading_yaw_var_source`
- `vehicle_speed_var`, `vehicle_nhc_var`, `vehicle_zupt_var`, `vehicle_yaw_rate_var`
- `imu_gyro_noise_std`, `imu_accel_noise_std`, `imu_gyro_bias_noise_std`, `imu_accel_bias_noise_std`
- `P_trace`, `P_max_diag`, `P_min_diag`, `P_min_eig`, `P_pos_max_diag`, `P_vel_max_diag`,
  `P_att_max_diag`, `P_bg_max_diag`, `P_ba_max_diag`
- `imu_dt_min`, `imu_dt_mean`, `imu_dt_max`
- `gnss_delay`, `gnss_vel_delay`, `velocity_delay`, `steering_delay`
- `fgo_keyframe_count`, `fgo_correction_count`

**Parameter context (important for interpretation)**
- NIS gates: `eskf_nis_gate_gnss_pos_3d`, `eskf_nis_gate_gnss_vel_3d`, `eskf_nis_gate_heading_yaw`
- GNSS covariance scaling, status-based inflates, recovery holdoff/ramp
- IMU preprocessing options and gravity removal
- Vehicle constraints (NHC, ZUPT, yaw-rate)

## Analysis Points to Extract and Highlight
These are the “must inspect” signals for quality, reliability, and tuning:
- Activation & initialization timeline (trigger required, initial state, ESKF init)
- Sensor counts and rate stability (IMU, GNSS, GNSS velocity, vehicle speed, steering)
- Update accept/reject ratio and reason distribution
- NIS values vs gates, and inflation applied (per GNSS pos, GNSS vel, heading)
- Residual norms and residual spikes
- R/Q (measurement/process noise) applied vs configured
- Covariance health (P trace/max/min/eig) and drift over time
- IMU dt jitter and outliers
- Input delay metrics (GNSS/vel/steering/vehicle)
- Output availability and rate
- FGO keyframe/correction counts and growth rate
- Pose consistency between GNSS, ESKF output, and FGO pose
- Map frame consistency and TF health (optional)

## Redesign Goals
- Provide a “mission control” level dashboard for diagnosis and tuning.
- Expose all available diagnostics and provide intuitive high-level summaries.
- Support quick detection of sensor dropouts, gating issues, and covariance divergence.
- Allow offline review via CSV/rosbag without code changes.
- Make thresholds explicit and allow dynamic compare with configured params.

## Proposed Features (Must-have)
1. **Overview Dashboard**
   - Status banner (OK/WARN/ERROR/STALE) + last update time.
   - Activation/ESKF init state.
   - Output availability ratio and output rate.
   - “Red flags” summary cards (sensor delay spikes, NIS gate violations, covariance growth).

2. **Fusion Quality Panel**
   - NIS charts with gate overlays (pos/vel/heading).
   - Accept/reject ratios with reason histogram.
   - Residual norms time series.

3. **Covariance & Noise Panel**
   - P trace / min eig / max diag charts.
   - R/Q applied values vs configured values.
   - Inflation factors visualization.

4. **Sensor Timing Panel**
   - IMU dt min/mean/max with jitter indicator.
   - Delay time series for GNSS/vel/vehicle/steering.
   - Last message age per sensor.

5. **Pose & Trajectory Panel**
   - Pose cards for GNSS / ESKF / FGO (position, yaw, age).
   - Overlaid 2D trajectory plot (GNSS vs ESKF vs FGO).
   - Keyframe path overlay and correction points (from `/localization/fgo/keyframe_path`).

6. **Event & Alert Panel**
   - A chronological event log: gate rejections, GNSS status changes, activation changes.
   - Rule-based alerts (configurable thresholds) with severity.

7. **Raw Diagnostic Inspector**
   - Searchable table of key-value diagnostics.
   - “diff since last” and “top changes” view.

8. **Export & Session Panel**
   - Export CSV and “snapshot report” (PDF/HTML optional).
   - Offline mode to load CSV and replay time window.

## UX/UI Layout Proposal
Use a multi-tab desktop UI (Qt6). Suggested layout:

**Tab 1: Overview**
- Top: Status banner and key KPIs (availability, output rate, diag rate).
- Middle: Alert cards (latest warnings).
- Bottom: Mini charts (NIS trend, delay trend, P trace trend).

**Tab 2: Fusion Quality**
- Left: Update ratios + reason histogram (GNSS pos/vel, heading).
- Right: NIS charts with gate overlays and residual charts.

**Tab 3: Covariance & Noise**
- Cards for P stats, Q, R.
- Line charts for P trace, max diag, min eig.
- Inflation factors timeline.

**Tab 4: Sensor Timing**
- IMU dt stats (sparkline + numeric min/mean/max).
- Delay charts for GNSS/vel/vehicle/steering.
- Last message age list.

**Tab 5: Pose & Trajectory**
- Pose cards for GNSS/ESKF/FGO with timestamp age.
- 2D plot of trajectories (map frame), toggle layers.
- Keyframe path and correction markers.

**Tab 6: Events & Inspector**
- Event timeline list with filters.
- Raw diagnostics table with search and value diff.

**Tab 7: Export & Session**
- Export controls, report summary.
- Offline replay controls (time slider, playback speed).

## Data Processing / Metrics
- Maintain sliding window (default 10s) for KPI computation.
- Compute rolling statistics: mean, p95, max, count.
- Gate overlays from parameters (read from ROS params or config file).
- Age metrics: `now - last_stamp` for each sensor and output.
- Create alert rules:
  - NIS > gate for X% in window.
  - Output availability < threshold.
  - P trace or min eig out of bounds.
  - Sensor delay > threshold.
  - IMU dt jitter > threshold.

## Data Mapping (Input → UI)
| Data | Source | UI Usage |
|---|---|---|
| diag status level + message | `/diagnostics` | Status banner, alerts |
| update ratios + reasons | diagnostics keys | Fusion Quality tab |
| NIS + residuals | diagnostics keys | Fusion Quality tab |
| R/Q and inflation | diagnostics keys | Covariance & Noise tab |
| P stats | diagnostics keys | Covariance & Noise tab |
| IMU dt stats | diagnostics keys | Sensor Timing tab |
| delays | diagnostics keys | Sensor Timing tab |
| output rate/availability | `/localization/kinematic_state` | Overview |
| poses | GNSS/ESKF/FGO topics | Pose tab |
| keyframe path | `/localization/fgo/keyframe_path` | Pose tab |
| state text | `/localization/state` | Overview |

## Architecture Proposal
- **DataLayer**: ROS subscribers, raw sample cache, time sync.
- **MetricsEngine**: rolling stats, alert evaluation, derived KPIs.
- **SessionStore**: in-memory ring buffers + CSV exporter + replay loader.
- **UI Layer**: Qt widgets + charts, with a consistent dark theme and status color coding.

## Optional Enhancements (if allowed)
- Add new diagnostic keys from `hybrid_localization` for missing metrics.
- Publish a dedicated `/hybrid_localization/metrics` message for structured data.

## Acceptance Criteria
- All diagnostic keys listed above are parsed and visible in UI.
- At least 5 core panels implemented: Overview, Fusion Quality, Covariance & Noise, Sensor Timing, Pose.
- Ability to export CSV and load/export session summary.
- Alert rules configured by parameters.
- Smooth UI update at 5–10 Hz without blocking ROS callbacks.

## Claude Implementation Prompt
You are to implement the redesign of `hybrid_localization_evaluation_tool` in this repository. Use Qt6 and ROS2 C++17. Follow the spec above exactly. Create a multi-tab UI with the described panels. Ingest the listed topics and all diagnostics keys. Implement a metrics engine with a sliding window and an alert system. Preserve existing CSV export, extend it to include all new fields. Provide a simple offline CSV replay mode. Do not change `hybrid_localization` unless explicitly necessary; if needed, add optional hooks guarded by parameters. Update README with new usage and parameters.


## Diagnostics Key Reference (Verified)
Verified against `publish.cpp` and `diagnostics_publisher.{hpp,cpp}`. `publish.cpp` builds `EskfDiagnosticsInput`, and `EskfDiagnosticsPublisher` serializes the keys below into `/diagnostics`.

**Core state**
- `is_activated` (bool)
- `eskf_initialized` (bool)
- `imu_count`, `gnss_count`, `gnss_vel_count`, `velocity_count`, `steering_count` (counts)

**GNSS position update**
- `gnss_pos_update_applied` (bool)
- `gnss_pos_update_reason` (string)
- `gnss_pos_nis` (double)
- `gnss_pos_residual_norm` (double)
- `gnss_status` (int)
- `gnss_pos_R_xx`, `gnss_pos_R_yy`, `gnss_pos_R_zz` (double)
- `gnss_pos_status_inflate`, `gnss_pos_nis_inflate` (double)

**GNSS velocity update**
- `gnss_vel_update_applied` (bool)
- `gnss_vel_update_reason` (string)
- `gnss_vel_nis` (double)
- `gnss_vel_residual_norm` (double)
- `gnss_vel_R_xx`, `gnss_vel_R_yy`, `gnss_vel_R_zz` (double)
- `gnss_vel_status_inflate`, `gnss_vel_nis_inflate` (double)

**Heading yaw update**
- `heading_yaw_update_applied` (bool)
- `heading_yaw_update_reason` (string)
- `heading_yaw_nis` (double)
- `heading_yaw_residual_rad` (double)
- `heading_yaw_var` (double)
- `heading_yaw_var_eff` (double)
- `heading_yaw_var_applied` (double)
- `heading_yaw_nis_inflate` (double)
- `heading_status_inflate`, `heading_recover_inflate` (double)
- `heading_yaw_var_source` (string)

**Vehicle constraints (measurement noise)**
- `vehicle_speed_var`, `vehicle_nhc_var`, `vehicle_zupt_var`, `vehicle_yaw_rate_var` (double)

**IMU process noise (Q)**
- `imu_gyro_noise_std`, `imu_accel_noise_std`, `imu_gyro_bias_noise_std`, `imu_accel_bias_noise_std` (double)

**Covariance (P) health**
- `P_trace`, `P_max_diag`, `P_min_diag`, `P_min_eig`
- `P_pos_max_diag`, `P_vel_max_diag`, `P_att_max_diag`, `P_bg_max_diag`, `P_ba_max_diag`

**Timing**
- `imu_dt_min`, `imu_dt_mean`, `imu_dt_max` (ms)
- `gnss_delay`, `gnss_vel_delay`, `velocity_delay`, `steering_delay` (ms)

**FGO stats**
- `fgo_keyframe_count`, `fgo_correction_count`

## UI Wireflow (Detailed)
Goal: clear, task-driven flow from status triage → root cause analysis → export.

1. **Startup / Session Selection**
   - Auto-detect live ROS topics. Show green “Live” badge if connected.
   - Offer “Load Session” to replay CSV/rosbag if no live data.
   - Action: choose `Live` or `Offline` mode.

2. **Overview Tab (Default Landing)**
   - Top banner: system state (OK/WARN/ERROR/STALE), last update time, activation, ESKF init.
   - KPI row: output rate, availability, diag rate, last output age.
   - Red flag cards: NIS gate violations, delay spikes, covariance drift.
   - “Go to Root Cause” button → opens Fusion Quality or Timing based on highest-severity alert.

3. **Fusion Quality Tab**
   - Update ratios and reason histograms (GNSS pos/vel/heading).
   - NIS charts with gate overlays and violation markers.
   - Residual norms chart.
   - Click on a spike → opens event detail drawer showing raw diagnostics at that time.

4. **Covariance & Noise Tab**
   - P trace, min eig, max diag charts with thresholds.
   - R/Q values vs configured parameters (diff shown).
   - Inflation factors plotted over time.
   - “Parameter Compare” panel shows configured vs applied.

5. **Sensor Timing Tab**
   - IMU dt min/mean/max sparkline + jitter indicator.
   - Delay charts for GNSS, GNSS vel, velocity, steering.
   - Last message age badges; clicking a badge highlights corresponding timeline section.

6. **Pose & Trajectory Tab**
   - Pose cards (GNSS, ESKF, FGO) with age + yaw.
   - 2D trajectory plot with layer toggles.
   - Keyframe path overlay and correction markers.

7. **Events & Inspector Tab**
   - Event list (filters by severity, type).
   - Raw diagnostics table with search, diff since last, and pinned keys.

8. **Export & Session Tab**
   - Export CSV (raw + KPI + events + parameters snapshot).
   - Generate “Session Report” (HTML/PDF optional).
   - Offline playback controls (time slider, speed, pause).

## Data Schema (CSV)
### 1) Raw Diagnostics CSV
Filename: `hybrid_eval_raw_YYYYMMDD_HHMMSS.csv`

Columns:
- `stamp_sec` (double)
- `is_activated` (0/1)
- `eskf_initialized` (0/1)
- `imu_count`, `gnss_count`, `gnss_vel_count`, `velocity_count`, `steering_count` (int)
- `gnss_pos_update_applied` (0/1)
- `gnss_pos_update_reason` (string)
- `gnss_pos_nis` (double)
- `gnss_pos_residual_norm` (double)
- `gnss_status` (int)
- `gnss_pos_R_xx`, `gnss_pos_R_yy`, `gnss_pos_R_zz` (double)
- `gnss_pos_status_inflate`, `gnss_pos_nis_inflate` (double)
- `gnss_vel_update_applied` (0/1)
- `gnss_vel_update_reason` (string)
- `gnss_vel_nis` (double)
- `gnss_vel_residual_norm` (double)
- `gnss_vel_R_xx`, `gnss_vel_R_yy`, `gnss_vel_R_zz` (double)
- `gnss_vel_status_inflate`, `gnss_vel_nis_inflate` (double)
- `heading_yaw_update_applied` (0/1)
- `heading_yaw_update_reason` (string)
- `heading_yaw_nis` (double)
- `heading_yaw_residual_rad` (double)
- `heading_yaw_var`, `heading_yaw_var_eff`, `heading_yaw_var_applied` (double)
- `heading_yaw_nis_inflate`, `heading_status_inflate`, `heading_recover_inflate` (double)
- `heading_yaw_var_source` (string)
- `vehicle_speed_var`, `vehicle_nhc_var`, `vehicle_zupt_var`, `vehicle_yaw_rate_var` (double)
- `imu_gyro_noise_std`, `imu_accel_noise_std`, `imu_gyro_bias_noise_std`, `imu_accel_bias_noise_std` (double)
- `P_trace`, `P_max_diag`, `P_min_diag`, `P_min_eig` (double)
- `P_pos_max_diag`, `P_vel_max_diag`, `P_att_max_diag`, `P_bg_max_diag`, `P_ba_max_diag` (double)
- `imu_dt_min_ms`, `imu_dt_mean_ms`, `imu_dt_max_ms` (double)
- `gnss_delay_ms`, `gnss_vel_delay_ms`, `velocity_delay_ms`, `steering_delay_ms` (double)
- `fgo_keyframe_count`, `fgo_correction_count` (int)
- `diag_level` (0=OK,1=WARN,2=ERROR,3=STALE)

Notes:
- For diagnostics not present in a given sample, leave blank.
- All durations in milliseconds where possible, consistent with diagnostic strings.

### 2) KPI Snapshot CSV
Filename: `hybrid_eval_kpi_YYYYMMDD_HHMMSS.csv`

Columns:
- `stamp_sec` (double)
- `window_sec` (double)
- `output_count`, `output_expected` (double)
- `output_ratio` (double)
- `output_last_age_sec` (double)
- `diag_rate_hz`, `output_rate_hz` (double)
- `pos_applied_ratio`, `pos_applied_count`, `pos_total`
- `vel_applied_ratio`, `vel_applied_count`, `vel_total`
- `heading_applied_ratio`, `heading_applied_count`, `heading_total`
- `pos_nis_mean`, `pos_nis_p95`, `pos_nis_max`, `pos_nis_count`
- `vel_nis_mean`, `vel_nis_p95`, `vel_nis_max`, `vel_nis_count`
- `heading_nis_mean`, `heading_nis_p95`, `heading_nis_max`, `heading_nis_count`
- `gnss_delay_mean`, `gnss_delay_p95`, `gnss_delay_max`, `gnss_delay_count`
- `gnss_vel_delay_mean`, `gnss_vel_delay_p95`, `gnss_vel_delay_max`, `gnss_vel_delay_count`
- `velocity_delay_mean`, `velocity_delay_p95`, `velocity_delay_max`, `velocity_delay_count`
- `steering_delay_mean`, `steering_delay_p95`, `steering_delay_max`, `steering_delay_count`

### 3) Events CSV (New)
Filename: `hybrid_eval_events_YYYYMMDD_HHMMSS.csv`

Columns:
- `stamp_sec`
- `severity` (INFO/WARN/ERROR)
- `event_type` (e.g., `NIS_GATE_VIOLATION`, `SENSOR_DELAY`, `COVARIANCE_DRIFT`, `ACTIVATION_CHANGE`)
- `message`
- `key`
- `value`

### 4) Session Summary CSV (New)
Filename: `hybrid_eval_session_YYYYMMDD_HHMMSS.csv`

Columns:
- `start_time_sec`, `end_time_sec`, `duration_sec`
- `overall_status`
- `alerts_total`, `alerts_warn`, `alerts_error`
- `availability_min`, `availability_mean`
- `output_rate_mean`
- `nis_gate_violation_rate_pos`, `nis_gate_violation_rate_vel`, `nis_gate_violation_rate_heading`
- `max_P_trace`, `min_P_min_eig`

## Implementation Notes
- Keep UI update at 5–10 Hz, decouple ROS callbacks from rendering.
- Prefer a ring buffer for raw samples to support replay and charts.
- All new CSV outputs should be optional via parameters.
