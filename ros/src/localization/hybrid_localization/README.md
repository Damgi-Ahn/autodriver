# hybrid_localization

ESKF-FGO 하이브리드 6-DOF 차량 측위 노드.

IMU 관성 전파(ESKF Frontend)와 GTSAM 기반 슬라이딩 윈도우 FGO(Factor Graph Optimization)를
결합하여 GNSS 열화 구간에서도 연속적이고 일관된 위치·자세 추정치를 제공한다.

---

## 목차

1. [알고리즘 개요](#1-알고리즘-개요)
2. [처리 파이프라인 (5-Stage)](#2-처리-파이프라인-5-stage)
3. [입력 / 출력 토픽](#3-입력--출력-토픽)
4. [파라미터 레퍼런스](#4-파라미터-레퍼런스)
5. [상태 머신](#5-상태-머신)
6. [좌표계 규약](#6-좌표계-규약)
7. [성능 특성](#7-성능-특성)
8. [한계 및 알려진 이슈](#8-한계-및-알려진-이슈)
9. [설계 결정 사항](#9-설계-결정-사항)

---

## 1. 알고리즘 개요

### ESKF (Error-State Kalman Filter)

15-DOF 오차 상태를 추적한다.

| 인덱스 | 상태 | 단위 |
|--------|------|------|
| 0–2 | 위치 오차 δp | m |
| 3–5 | 속도 오차 δv | m/s |
| 6–8 | 자세 오차 δθ (SO(3) 접선 벡터) | rad |
| 9–11 | 자이로 바이어스 δb_g | rad/s |
| 12–14 | 가속도 바이어스 δb_a | m/s² |

IMU 전파(4차 Runge-Kutta)로 공칭 상태를 업데이트하고, GNSS/헤딩/차량 측정치가
도착할 때 오차 상태를 보정한다. NIS(Normalized Innovation Squared) 게이팅으로
이상치를 제거하며, 게이팅 실패 시 측정 노이즈를 inflate하여 재시도한다.

### FGO (Factor Graph Optimization)

GTSAM ISAM2 기반 슬라이딩 윈도우 최적화기.
각 키프레임 i에서 변수 {X(i), V(i), B(i)}를 정의하고,
다음 팩터들로 연결된 그래프를 최적화한다.

| 팩터 | 대상 변수 | 설명 |
|------|-----------|------|
| CombinedImuFactor | Xi, Vi, Bi ↔ Xj, Vj, Bj | IMU 사전적분 |
| GPSFactor | Xj | GNSS 위치 |
| GnssVelocityFactor | Vj | GNSS 속도 |
| GnssHeadingFactor | Xj | GPHDT 헤딩 yaw |
| NhcFactor | Xj, Vj | Non-Holonomic 제약 |
| PriorFactor\<Bias\> | Bj | 바이어스 랜덤 워크 제약 |

### ESKF-FGO 결합 방식

FGO 최적화 결과(최신 키프레임 상태)를 ESKF에 주입하여 누적 오차를 보정한다.
FGO는 전역 일관성 유지, ESKF는 100–200Hz 고주파 연속 출력을 담당한다.

---

## 2. 처리 파이프라인 (5-Stage)

```
IMU @ 200Hz ──────────────────────────────────────────────────────────────►
                │                                                           │
          Stage 1: ESKF Frontend                                    Stage 5: Publish
          ┌─────────────────────────────┐                         ┌─────────────────┐
          │ IMU 전처리 (LPF, bias 제거) │                         │ Odometry 발행   │
          │ ESKF 관성 전파 (RK4)        │◄── Stage 4: Corrector  │ TF 발행         │
          │ GNSS pos/vel 업데이트       │    FGO 상태 → ESKF 주입│ 상태 머신 출력  │
          │ 헤딩 yaw 업데이트           │                         │ 진단 발행       │
          │ 차량 제약 업데이트          │                         └─────────────────┘
          │  - NHC, ZUPT, yaw-rate     │
          └─────────────────────────────┘
                         │
                   키프레임 선택 (Stage 2)
                   거리/각도/시간 기준
                         │
                   Stage 3: FGO Backend
                   ┌─────────────────────────────┐
                   │ IMU 사전적분                 │
                   │ GTSAM ISAM2 슬라이딩 윈도우 │
                   │ GNSS/헤딩/NHC 팩터 추가     │
                   │ 그래프 최적화               │
                   └─────────────────────────────┘
```

### Stage 1 — ESKF Frontend

- IMU 데이터를 수신할 때마다 관성 전파 수행 (200Hz 권장).
- GNSS NavSatFix → map 프레임 ENU 위치로 투영 후 위치 업데이트.
- GNSS TwistStamped → 속도 업데이트.
- GPHDT (GNSS 헤딩) → yaw 업데이트. Rate gate, NIS gate, 상태-inflate 파이프라인 통과.
- 차량 속도(OBD), NHC, ZUPT, 조향 기반 yaw-rate 제약.

### Stage 2 — 키프레임 선택

이동 거리(`fgo.keyframe.min_dist_m`), 자세 변화(`fgo.keyframe.min_angle_deg`),
경과 시간(`fgo.keyframe.min_interval_sec` / `max_interval_sec`) 기준으로
ESKF 상태를 FGO 키프레임으로 스냅샷한다.

### Stage 3 — FGO Backend

GTSAM ISAM2 슬라이딩 윈도우(기본 20 키프레임)로 전역 최적화를 수행한다.
GNSS 불량 시 `adaptive_window_max`까지 윈도우를 확장하여 관성 추측항법 구간을
더 길게 커버할 수 있다.

### Stage 4 — EskfCorrector

FGO 최적화 결과를 받아 ESKF 상태를 보정한다.
`fgo.corrector.max_reintegration_window_sec` 이내의 IMU 데이터를 재적분하여
FGO-ESKF 상태 불일치를 해소한다.

### Stage 5 — Publish

`publish_rate`(기본 200Hz) 타이머에서 최신 ESKF 상태를 발행한다.
`output.flatten_roll_pitch=true`이면 roll/pitch를 약하게 눌러
평면 차량 모델에 맞는 자세를 출력한다.

---

## 3. 입력 / 출력 토픽

### 구독 (Subscribes)

| 토픽 (기본값) | 메시지 타입 | 설명 |
|---------------|-------------|------|
| `/sensing/imu/imu_raw` | `sensor_msgs/Imu` | IMU 가속도/각속도 (200Hz 권장) |
| `/sensing/gnss/navsatfix` | `sensor_msgs/NavSatFix` | GNSS 위치 (위·경·고도) |
| `/sensing/gnss/vel` | `geometry_msgs/TwistStamped` | GNSS 속도 (NED→ENU 변환 후 사용) |
| `/sensing/gnss/heading` | `skyautonet_msgs/Gphdt` | GPHDT GNSS 헤딩 (도 단위, CW+) |
| `/vehicle/status/velocity_status` | `autoware_vehicle_msgs/VelocityReport` | OBD 차량 속도 |
| `/vehicle/status/steering_status` | `autoware_vehicle_msgs/SteeringReport` | 조향각 (yaw-rate 추정용) |
| `/map/map_projector_info` | `tier4_map_msgs/MapProjectorInfo` | 지도 원점 / 투영 방식 |
| `initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 외부 초기 자세 (리맵 가능) |

모든 토픽 이름은 파라미터로 재정의할 수 있다
(`imu_topic`, `gnss_topic`, `gnss_vel_topic`, `heading_topic`, 등).

### 발행 (Publishes)

| 토픽 (기본값) | 메시지 타입 | 설명 |
|---------------|-------------|------|
| `/localization/kinematic_state` | `nav_msgs/Odometry` | 위치·속도·자세 + 공분산 (200Hz) |
| `/localization/pose_twist_fusion_filter/pose` | `geometry_msgs/PoseStamped` | 자세만 추출 (ESKF 초기화 후) |
| `/localization/state` | `std_msgs/String` | 상태 머신 문자열 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 상세 진단 (20Hz, 분주비 설정 가능) |
| `/sensing/gnss/pose_with_covariance` | `geometry_msgs/PoseWithCovarianceStamped` | GNSS 위치 출력 (옵션, 비활성 기본값) |

---

## 4. 파라미터 레퍼런스

전체 파라미터는 `config/hybrid_localization.param.yaml`에서 설정한다.
기본값은 `include/hybrid_localization/parameters.hpp`에 정의되어 있다.

### I/O

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `imu_topic` | `/sensing/imu/imu_raw` | IMU 토픽 |
| `gnss_topic` | `/sensing/gnss/navsatfix` | GNSS 위치 토픽 |
| `gnss_vel_topic` | `/sensing/gnss/vel` | GNSS 속도 토픽 |
| `heading_topic` | `/sensing/gnss/heading` | GNSS 헤딩 토픽 |
| `map_frame` | `map` | 지도 프레임 ID |
| `base_frame` | `base_link` | 차량 베이스 프레임 ID |
| `publish_tf` | `true` | map→base_link TF 발행 여부 |
| `publish_rate` | `200.0` | 출력 타이머 주기 [Hz] |
| `diag_publish_divider` | `10` | 진단 발행 분주비 (publish_rate/divider = 진단 Hz) |

### 초기화

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `init.mode` | `auto` | `auto`: 내부 자동 초기화 / `external_initialpose`: 외부 자세 대기 |
| `init.require_trigger` | `false` | SetBool 트리거 수신 전까지 비활성 (Autoware 호환) |
| `init.reset_on_external_pose` | `true` | 외부 자세 수신마다 필터 재초기화 여부 |
| `init_imu_calibration` | `false` | 시작 시 IMU 정지 캘리브레이션 수행 여부 |
| `imu_calibration_duration_sec` | `30.0` | 캘리브레이션 수집 시간 [s] |

### ESKF 공분산 초기값

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `eskf.init_pos_var` | `1.0e6` | 초기 위치 분산 [m²] |
| `eskf.init_vel_var` | `1.0e2` | 초기 속도 분산 [(m/s)²] |
| `eskf.init_att_var` | `(30°)²` | 초기 자세 분산 [rad²] |
| `eskf.init_bg_var` | `0.01` | 초기 자이로 바이어스 분산 [(rad/s)²] |
| `eskf.init_ba_var` | `0.25` | 초기 가속도 바이어스 분산 [(m/s²)²] |

### ESKF 프로세스 노이즈 (IMU)

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `eskf.gyro_noise_std` | `0.05` | 자이로 측정 잡음 [rad/s/√Hz] |
| `eskf.accel_noise_std` | `1.0` | 가속도 측정 잡음 [m/s²/√Hz] |
| `eskf.gyro_bias_noise_std` | `1e-4` | 자이로 바이어스 확산 잡음 [rad/s²/√Hz] |
| `eskf.accel_bias_noise_std` | `1e-3` | 가속도 바이어스 확산 잡음 [m/s³/√Hz] |

### ESKF NIS 게이팅

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `eskf.nis_gate_gnss_pos_3d` | `11.34` | GNSS 위치 NIS 게이트 (χ², 3-DOF, 99%) |
| `eskf.nis_gate_gnss_vel_3d` | `11.34` | GNSS 속도 NIS 게이트 |
| `eskf.nis_gate_heading_yaw` | `6.63` | 헤딩 yaw NIS 게이트 (χ², 1-DOF, 99%) |
| `eskf.nis_gate_inflate` | `true` | NIS 초과 시 R inflate 후 재시도 여부 |
| `eskf.nis_gate_inflate_max` | `10000.0` | R inflate 최대 배율 |
| `eskf.max_correction_pos_m` | `50.0` | 단일 업데이트 최대 위치 보정량 [m] |
| `eskf.max_correction_vel_mps` | `20.0` | 단일 업데이트 최대 속도 보정량 [m/s] |
| `eskf.max_correction_att_rad` | `0.0` | 단일 업데이트 최대 자세 보정량 [rad] (0=비활성) |

### GNSS

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `enable_gnss_pos_update` | `true` | GNSS 위치 업데이트 on/off |
| `enable_gnss_vel_update` | `true` | GNSS 속도 업데이트 on/off |
| `gnss.use_navsatfix_covariance` | `true` | NavSatFix 내 공분산 사용 여부 |
| `gnss.covariance_scale` | `25.0` | GNSS 공분산 스케일링 배율 (과신 방지) |
| `gnss.pos_var_fallback` | `4.0` | 공분산 없을 때 위치 분산 fallback [m²] |
| `gnss.pos_var_min` | `0.0025` | 위치 분산 하한 [m²] |
| `gnss.pos_var_max` | `1.0e7` | 위치 분산 상한 [m²] |
| `gnss.pos_inflate_status_fix` | `1000000.0` | status=0(Fix) 시 위치 inflate |
| `gnss.pos_inflate_status_sbas` | `100.0` | status=1(SBAS) 시 위치 inflate |
| `gnss.vel_inflate_status_fix` | `25.0` | status=0 시 속도 inflate |
| `gnss.vel_inflate_status_sbas` | `4.0` | status=1 시 속도 inflate |
| `gnss.heading_neg_status_inflate` | `64.0` | status<0 시 헤딩 inflate |
| `gnss.min_status_for_pos_update` | `0` | 이 status 미만 시 위치 업데이트 스킵 |

#### GNSS Recovery (status 개선 전환 완충)

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `gnss.recover.enable` | `true` | holdoff+ramp 완충 사용 여부 |
| `gnss.recover.holdoff_sec` | `1.0` | status 개선 직후 업데이트 스킵 시간 [s] |
| `gnss.recover.ramp_sec` | `5.0` | holdoff 이후 inflate 감쇠 시간 [s] |
| `gnss.recover.pos_max_inflate` | `1000.0` | 위치 recover peak inflate 상한 |
| `gnss.recover.vel_max_inflate` | `50.0` | 속도 recover peak inflate 상한 |
| `gnss.recover.decay_exponent` | `5.0` | 감쇠 지수 (t=ramp_sec에서 exp(-5)≈0.7% 잔여) |

### 헤딩 (GPHDT)

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `heading.enable_yaw_update` | `true` | GPHDT 기반 yaw 업데이트 on/off |
| `heading.yaw_var` | `0.012` | 헤딩 측정 노이즈 분산 [rad²] |
| `heading.max_rate_radps` | `1.0` | yaw 변화율 게이트 [rad/s] (초과 시 skip) |
| `heading.rate_gate_skip_max_count` | `20` | 연속 rate-gate skip 허용 횟수 (초과 시 복구) |
| `heading.rate_gate_skip_max_sec` | `2.0` | 연속 rate-gate skip 허용 시간 [s] |
| `heading.rate_gate_bypass_yaw_var_scale` | `10.0` | 복구 바이패스 시 yaw_var 임시 inflate 배율 |
| `heading.rate_gate_init_grace_sec` | `5.0` | ESKF 초기화 후 rate-gate 유예 시간 [s] |
| `heading.min_gnss_status_for_yaw_update` | `1` | 이 status 미만 시 yaw 업데이트 차단 |

#### 헤딩 이벤트 Inflate

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `heading.event_inflate_invalid` | `64.0` | 비정상 헤딩(non-finite, status=-1) peak inflate |
| `heading.event_inflate_timeout` | `16.0` | 타임아웃/GNSS status skip/holdoff peak inflate |
| `heading.event_inflate_rate_gate` | `8.0` | rate-gate skip peak inflate |
| `heading.event_inflate_default` | `4.0` | 기타 이벤트 peak inflate |
| `heading.recover_peak_min_inflate` | `4.0` | status 개선 시 recover peak 하한 |

### 차량 제약

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `vehicle.enable_speed_update` | `true` | OBD 속도 업데이트 on/off |
| `vehicle.speed_var` | `0.25` | OBD 속도 분산 [(m/s)²] |
| `vehicle.min_speed_mps_for_speed_update` | `1.0` | 속도 업데이트 최소 속도 [m/s] |
| `vehicle.enable_nhc` | `true` | Non-Holonomic 제약 on/off |
| `vehicle.nhc_var` | `0.04` | NHC 분산 (횡/수직) [(m/s)²] |
| `vehicle.enable_zupt` | `false` | Zero-velocity update on/off |
| `vehicle.zupt_speed_threshold_mps` | `0.2` | ZUPT 정지 판단 속도 임계값 [m/s] |
| `vehicle.zupt_var` | `0.01` | ZUPT 분산 [(m/s)²] |
| `vehicle.enable_yaw_rate_update` | `true` | 조향 기반 yaw-rate 제약 on/off |
| `vehicle.yaw_rate_var` | `0.0004` | yaw-rate 분산 [(rad/s)²] |

### FGO 키프레임 선택

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `fgo.keyframe.min_dist_m` | `0.5` | 키프레임 생성 최소 이동 거리 [m] |
| `fgo.keyframe.min_angle_deg` | `5.0` | 키프레임 생성 최소 자세 변화 [°] |
| `fgo.keyframe.min_interval_sec` | `0.1` | 키프레임 최소 간격 [s] (최대 10Hz) |
| `fgo.keyframe.max_interval_sec` | `1.0` | 키프레임 최대 간격 [s] (강제 생성) |

### FGO 백엔드 (ISAM2)

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `fgo.backend.window_size` | `20` | 슬라이딩 윈도우 크기 (keyframe 수) |
| `fgo.backend.adaptive_window_enable` | `false` | GNSS 불량 시 윈도우 자동 확장 여부 |
| `fgo.backend.adaptive_window_max` | `40` | 적응형 최대 윈도우 크기 |
| `fgo.backend.prior_pos_noise_m` | `0.1` | 첫 키프레임 위치 prior 노이즈 [m] |
| `fgo.backend.prior_yaw_noise_rad` | `0.1` | 첫 키프레임 yaw prior 노이즈 [rad] |
| `fgo.backend.prior_vel_noise_mps` | `0.1` | 첫 키프레임 속도 prior 노이즈 [m/s] |
| `fgo.backend.nhc_lat_noise_mps` | `0.05` | NHC 횡방향 노이즈 [m/s] |
| `fgo.backend.nhc_vert_noise_mps` | `0.1` | NHC 수직방향 노이즈 [m/s] |
| `fgo.backend.isam2_relinearize_threshold` | `0.1` | ISAM2 재선형화 트리거 임계값 |
| `fgo.backend.isam2_relinearize_skip` | `1` | ISAM2 재선형화 체크 간격 (업데이트 횟수) |
| `fgo.backend.isam2_integration_cov` | `1e-8` | IMU 수치 적분 잡음 공분산 스케일 |
| `fgo.backend.isam2_bias_acc_omega_int` | `1e-5` | 바이어스 적분 잡음 공분산 스케일 |

### FGO Corrector

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `fgo.corrector.enabled` | `true` | FGO→ESKF 보정 주입 on/off |
| `fgo.corrector.inject_covariance` | `true` | FGO 공분산 ESKF 주입 여부 |
| `fgo.corrector.max_reintegration_window_sec` | `1.0` | IMU 재적분 최대 윈도우 [s] |
| `fgo.corrector.imu_buffer_max_samples` | `400` | IMU 버퍼 최대 샘플 수 (200Hz×2s) |

### 출력

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `output.flatten_roll_pitch` | `true` | roll/pitch 평탄화 (평면 차량 모드) |
| `output.roll_pitch_var` | `1000.0` | 평탄화 시 roll/pitch 제약 분산 [rad²] |

### 상태 머신

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `state.gnss_degraded_timeout_sec` | `5.0` | GNSS 불량 지속 시 DEGRADED 전이 타임아웃 [s] |
| `time_alignment_tolerance_ms` | `100.0` | 과거 측정치 허용 타임스탬프 허용 오차 [ms] |

---

## 5. 상태 머신

```
UNINITIALIZED
     │  GNSS + 헤딩 수신 (또는 external_initialpose)
     ▼
INITIALIZING
     │  ESKF 초기화 완료
     ▼
OPERATING ◄──── GNSS 복구
     │  GNSS 불량 > gnss_degraded_timeout_sec
     ▼
DEGRADED ──── 장기 GNSS 불량 또는 내부 오류
     │
     ▼
FAILED
```

상태는 `/localization/state` 토픽으로 문자열 형태로 발행된다.

---

## 6. 좌표계 규약

| 항목 | 규약 |
|------|------|
| 지도 프레임 | ENU (East-North-Up), map origin = `MapProjectorInfo` 기준 |
| 베이스 프레임 | `base_link` (차량 중심, z 위 방향) |
| 헤딩 입력 | ENU 기준 CW+ [°] → 내부적으로 표준 ENU yaw (CCW+ from East) [rad]로 변환 |
| GNSS 속도 입력 | `geometry_msgs/TwistStamped` NED 프레임 가정 → ENU로 변환 |
| 쿼터니언 | ROS 표준 (x, y, z, w) |

---

## 7. 성능 특성

### 정상 조건 (RTK-Fix 수신)

- 위치 정밀도: GNSS 공분산에 따라 결정 (RTK-Fix 기준 수평 ±0.05–0.1m 수준)
- 출력 지연: IMU 전파 기준 < 5ms (200Hz 타이머)
- 자세 정밀도: 헤딩 yaw 기준 ±0.1–0.3° (GPHDT 품질 의존)
- FGO 최적화 주기: 키프레임 생성 주기와 동일 (기본 최대 10Hz)

### GNSS 열화 구간 (관성 추측항법)

- 위치 오차 누적: IMU 바이어스 수렴 상태에 따라 크게 달라짐
  - 수렴된 바이어스 + 차량 제약: 10초 이내 수평 오차 < 1m 수준
  - 바이어스 미수렴: 오차 누적 빠름
- FGO 적응형 윈도우(`adaptive_window_enable=true`)로 이력 보존 가능
- NHC, ZUPT, yaw-rate 제약이 횡방향 오차 누적을 억제

### CPU 부하 (참고)

- ESKF 전파/업데이트: < 1ms per cycle
- FGO 키프레임 추가 + ISAM2 업데이트: 수ms (윈도우 크기에 비례)
- 전체 노드: 일반 산업용 SoC(Orin NX 등)에서 single-core 점유율 < 10%

---

## 8. 한계 및 알려진 이슈

### 알고리즘 한계

1. **IMU 바이어스 수렴 의존성**
   정지 캘리브레이션(`init_imu_calibration=true`) 없이 운용 시
   초기 바이어스 불확실성이 크다. 특히 z축 자이로 바이어스는
   yaw 드리프트에 직결된다.

2. **GNSS 단일 안테나**
   위치 정밀도는 GNSS 수신 환경(다중경로, 가시위성 수)에 직접 의존한다.
   도심 협로/터널 진입·진출 시 급격한 위치 점프가 발생할 수 있으며,
   이를 recover holdoff+ramp 메커니즘으로 완충하지만 완전 제거는 불가.

3. **헤딩 정밀도**
   GPHDT 단일 안테나 기반 헤딩은 저속 주행 시 정밀도가 낮다.
   `heading.min_gnss_status_for_yaw_update`로 최소 품질을 강제할 수 있으나
   저속 구간에서는 IMU 자이로 적분에 의존하게 된다.

4. **롤·피치 관측 불가**
   6-DOF ESKF이지만 롤·피치를 직접 관측하는 센서가 없다.
   `output.flatten_roll_pitch=true`(기본)로 출력을 평탄화하며,
   실제 경사로에서는 오차가 발생할 수 있다.

5. **지도 좌표계 고정**
   MapProjectorInfo 없이 초기화 불가. 좌표계 원점은 런타임에 고정되며
   한 세션 내에서 변경되지 않는다.

### 운용 주의사항

- **초기화**: GNSS Fix + 헤딩 동시 수신 시 자동 초기화.
  헤딩 없이는 초기화되지 않으므로 GPHDT 수신이 필수.
  `init.mode=external_initialpose` 모드에서는 외부 자세 수신 전까지 대기.

- **재초기화**: `init.reset_on_external_pose=true` 상태에서 `/initialpose` 수신 시
  필터가 완전 재초기화된다. 운행 중 의도치 않은 재초기화에 주의.

- **GNSS 공분산 스케일**: 수신기마다 공분산 과소추정이 일반적이므로
  `gnss.covariance_scale`로 보정이 필요하다 (기본 25배).

- **파라미터 튜닝 순서**:
  1. IMU 노이즈 파라미터 (`eskf.gyro_noise_std`, `eskf.accel_noise_std`)
  2. GNSS 공분산 스케일 (`gnss.covariance_scale`)
  3. 헤딩 분산 (`heading.yaw_var`)
  4. 차량 제약 분산 (`vehicle.speed_var`, `vehicle.nhc_var`)
  5. FGO 파라미터 (윈도우 크기, NHC 노이즈)

---

## 9. 설계 결정 사항

### NIS inflate 방식

NIS 게이트 초과 시 즉시 기각 대신 측정 노이즈 R을 inflate하여 재시도한다.
이를 통해 이상치는 약한 영향력으로나마 반영되고, 완전 기각으로 인한
장시간 업데이트 부재를 방지한다. `eskf.nis_gate_inflate_max`로 상한을 제한한다.

### 이중 inflate 파이프라인 (status + recover + NIS)

GNSS 및 헤딩 측정치는 세 단계 inflate를 통과한다:
1. **status inflate**: 수신기 fix 상태에 따른 기본 다운웨이트
2. **recover inflate**: status 개선 직후 과도기 완충 (exponential decay)
3. **NIS inflate**: 이상치 검출 시 추가 다운웨이트

세 inflate의 곱이 최종 측정 노이즈 R에 적용된다.

### ESKF-FGO 결합 방식

FGO 결과를 직접 ESKF 상태로 덮어쓰지 않고 오차 주입 방식을 사용한다.
보정 주입 후 FGO 최적화 시점과 현재 시점 사이의 IMU 데이터를 재적분하여
상태 연속성을 유지한다.

### 키프레임 선택 정책

고정 주파수 대신 이동 거리·자세 변화 기준으로 키프레임을 선택한다.
정차 중에는 키프레임이 생성되지 않아 FGO 그래프 크기가 불필요하게
증가하지 않는다. `max_interval_sec`로 최대 간격을 보장하여 장시간 정차 후
재출발 시에도 그래프가 갱신된다.

### recover peak 추정

GNSS status가 개선될 때 이전 status에서 현재 status로의 inflate 비율을
recover peak으로 사용한다. 이를 통해 실제 신호 품질 변화에 비례한
안전 마진을 자동으로 설정한다. 비율 추정이 불가능한 경우
`recover_peak_min_inflate`를 하한으로 사용한다.
