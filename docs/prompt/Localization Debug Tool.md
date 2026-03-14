# Claude Implementation Prompt

You are to implement a **commercial-grade Localization Debug and Evaluation Tool** for a hybrid localization system.
This tool must support **real-time monitoring, offline analysis, ground truth evaluation, statistical validation, and tuning workflow support**.

The implementation should follow **OEM-level autonomous driving localization debugging tools** used in production environments.

The system will ingest ROS2 topics from a hybrid localization stack and provide **evaluation, diagnostics, visualization, and session comparison**.

---

# 1. System Purpose

The goal is to provide a **complete localization debugging and evaluation platform**.

The tool must support:

* real-time system monitoring
* localization quality analysis
* sensor timing analysis
* statistical validation of filter consistency
* ground truth comparison
* tuning workflow support
* session comparison and reporting

The tool should be usable by **localization engineers during algorithm development and field testing**.

---

# 2. Core Features (Must Implement)

The system must implement the following analysis modules:

1. Ground Truth Error Analysis (ATE / RPE / RMSE)
2. Timestamp Alignment Analyzer
3. NIS χ² Distribution Test
4. Innovation Statistical Test
5. Localization Failure Detection
6. Session Comparison Tool
7. Error Heatmap Visualization

---

# 3. System Architecture

Implement the following architecture.

```
ROS2 Topics
   │
   ▼
Data Ingestion Layer
   │
   ▼
Time Alignment Engine
   │
   ▼
Evaluation Engine
   ├── GroundTruthAnalyzer
   ├── NISAnalyzer
   ├── InnovationAnalyzer
   ├── CovarianceAnalyzer
   └── SensorTimingAnalyzer
   │
   ▼
Failure Detection Engine
   │
   ▼
Session Store (SQLite)
   │
   ├── Visualization UI
   ├── Session Comparison Engine
   └── Report Generator
```

The UI must be **decoupled from ROS callbacks** to ensure responsiveness.

---

# 4. ROS Topics to Ingest

The tool must subscribe to:

```
/localization/kinematic_state
/localization/kinematic_state_gnss
/localization/pose_twist_fusion_filter/pose
/localization/fgo/keyframe_path
/sensing/imu/imu_data
/diagnostics
/localization/state
/tf
```

Ground truth source:

```
/localization/kinematic_state_gnss
```

Assume this topic provides **RTK GNSS accuracy** and can be treated as ground truth.

---

# 5. Data Ingestion Layer

Create a **DataLayer** that subscribes to ROS topics and converts messages into internal samples.

Define a structure:

```
struct LocalizationSample
{
    double timestamp;

    Pose eskf_pose;
    Pose fgo_pose;
    Pose gnss_pose;

    double imu_dt;

    double gnss_delay;
    double velocity_delay;
    double steering_delay;

    Diagnostics diagnostics;
};
```

All samples must be stored in a **ring buffer**.

Default capacity:

```
10 minutes of data
```

---

# 6. Time Alignment Engine

Localization sensors operate at different rates:

```
IMU: 200 Hz
GNSS: 10 Hz
ESKF output: 50 Hz
FGO: 5 Hz
```

Implement a time alignment module that:

1. uses **ESKF timestamp as reference**
2. interpolates sensor data to match the reference time

Interpolation:

```
linear interpolation for pose and velocity
nearest for discrete status
```

The module must compute:

```
sensor_latency
sensor_jitter
timestamp_offset
```

---

# 7. Ground Truth Error Analysis

Use GNSS RTK pose as ground truth.

Implement the following metrics.

---

## Absolute Trajectory Error (ATE)

Definition:

```
ATE_i = || T_est_i - T_gt_i ||
```

Compute:

```
ATE_mean
ATE_RMSE
ATE_max
```

Compute for:

```
ESKF pose
FGO pose
```

---

## Relative Pose Error (RPE)

Definition:

```
RPE_i = (T_est_i^-1 T_est_{i+Δ})^-1
        (T_gt_i^-1 T_gt_{i+Δ})
```

Compute:

```
translation error
rotation error
```

Δ should be configurable:

```
1s
5s
10s
```

---

## Visualization

Plot:

```
position error vs time
yaw error vs time
velocity error vs time
```

Also implement **trajectory error heatmap**.

Each trajectory point should be colored based on error magnitude.

---

# 8. NIS χ² Distribution Test

For each measurement update:

```
gnss_pos_nis
gnss_vel_nis
heading_yaw_nis
```

Compute expected χ² distribution.

Overlay the theoretical distribution with observed NIS histogram.

Evaluate filter consistency:

```
consistency_score = fraction_of_samples_within_gate
```

---

# 9. Innovation Statistical Test

Analyze innovation sequence:

```
innovation = residual
```

Compute:

```
autocorrelation function
```

If lag correlation is high:

```
model mismatch likely
```

Visualization:

```
ACF plot
```

---

# 10. Timestamp Alignment Analyzer

Compute timing diagnostics.

Metrics:

```
imu_dt_jitter
gnss_latency
vehicle_latency
sensor_age
```

Detect:

```
late measurements
out-of-order messages
timestamp jumps
```

Visualize:

```
timestamp difference charts
```

---

# 11. Localization Failure Detector

Implement a health monitoring state machine.

States:

```
OK
DEGRADED
FAILURE
RECOVERING
```

Transition conditions include:

### Covariance explosion

```
P_trace > threshold
```

### Sensor dropout

```
sensor_age > threshold
```

### NIS violations

```
violation_ratio > threshold
```

### Trajectory jump

```
pose difference > threshold
```

Each transition should generate an **event**.

---

# 12. Session Storage System

Store sessions in **SQLite**.

Tables:

```
raw_samples
kpi_samples
events
trajectory
```

This allows:

```
offline replay
fast query
session comparison
```

---

# 13. Session Comparison Tool

Allow comparison of two sessions.

Example workflow:

```
session_A
session_B
```

Compare:

```
ATE
RPE
NIS distribution
residual statistics
covariance growth
sensor delay
```

Visualization:

```
overlay plots
```

---

# 14. Visualization UI (OEM Style)

Use **Qt6 multi-tab interface**.

Implement the following tabs.

---

## Overview

Purpose: quick system health check.

Display:

```
system status
activation state
ESKF initialization
output rate
availability
active alerts
```

Include mini charts:

```
NIS trend
delay trend
P trace
```

---

## Fusion Quality

Display:

```
update ratios
update reasons histogram
NIS charts with gate overlay
residual norms
```

Allow clicking on spikes to inspect detailed diagnostics.

---

## Ground Truth Error

Display:

```
ATE charts
RPE charts
error histogram
```

Include trajectory error heatmap.

---

## Sensor Timing

Display:

```
IMU dt statistics
GNSS delay chart
vehicle delay chart
last message age
```

---

## Covariance

Charts:

```
P_trace
P_max_diag
P_min_eig
```

Also display applied noise values.

---

## Trajectory

Map visualization.

Layers:

```
GNSS
ESKF
FGO
Ground Truth
```

Also overlay:

```
error heatmap
innovation vectors
```

---

## Events

Timeline of system events.

Filter by:

```
severity
type
sensor
```

---

## Session Compare

Load two sessions.

Display comparison plots.

---

# 15. Localization Tuning Workflow Support

The tool must support a **research workflow**.

Typical workflow:

1. Run localization stack
2. Record rosbag
3. Load session into tool
4. Evaluate metrics
5. Identify problems
6. Adjust parameters
7. Re-run and compare sessions

The tool must support:

```
parameter snapshot recording
session comparison
automated report generation
```

---

# 16. Report Generation

Generate a report including:

```
trajectory plots
error metrics
NIS analysis
sensor timing
failure events
```

Export format:

```
HTML
PDF (optional)
```

---

# 17. Performance Requirements

The UI must update at:

```
5–10 Hz
```

ROS callbacks must not block UI rendering.

Use:

```
separate worker threads
```

---

# 18. Code Structure

Recommended project layout:

```
localization_debug_tool
 ├── data_layer
 │
 ├── time_alignment
 │
 ├── evaluation
 │    ground_truth_analyzer
 │    nis_analyzer
 │    innovation_analyzer
 │
 ├── failure_detection
 │
 ├── session
 │
 ├── visualization
 │
 └── export
```

---

# 19. Acceptance Criteria

The implementation is considered complete if:

1. All ROS topics are ingested correctly
2. Ground truth comparison works
3. ATE and RPE metrics are computed
4. NIS χ² distribution is visualized
5. timestamp alignment analysis works
6. localization failure detection generates events
7. session comparison produces overlay plots
8. UI remains responsive at 5–10 Hz

---

If implemented correctly, this tool will function as an **OEM-level localization debugging and evaluation platform** used in autonomous driving development.
