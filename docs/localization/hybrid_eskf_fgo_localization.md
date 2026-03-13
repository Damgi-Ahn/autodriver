# Hybrid ESKF-FGO Localization — 설계 명세

**작성일:** 2026-03-13
**플랫폼:** NVIDIA Jetson Orin (JetPack 6.0, aarch64)
**프레임워크:** ROS 2 / ament_cmake_auto
**언어:** C++17
**참조 구현:** `fodro_v1-maxen_10x4` ESKF localization (skyautonet)

---

## 1. 설계 동기

### 1.1 기존 ESKF의 한계

기존 `eskf_localization` 패키지는 실시간 IMU 전파 + GNSS 보정 구조로 동작하며 다음 한계를 갖는다.

| 한계 | 원인 | 영향 |
|------|------|------|
| 지연 측정치(delayed measurement) 처리 불가 | 재귀 필터 특성상 과거 시점 보정 불가 | LiDAR scan matching 결과 활용 불가 |
| 비가우시안 노이즈 취약 | EKF는 가우시안 가정 | GNSS 멀티패스, outlier 누적 |
| 루프 클로저 반영 불가 | 필터는 단방향 시간 진행 | 동일 장소 재방문 시 오차 누적 |
| 슬라이딩 윈도우 최적화 미지원 | 단일 공분산 행렬 전파 | 과거 측정치 재가중 불가 |

### 1.2 FGO(Factor Graph Optimization)의 역할

FGO는 **슬라이딩 윈도우** 방식으로 최근 N초 구간의 상태를 **일괄 비선형 최적화**로 재추정한다.

- 지연 측정치: 윈도우 내 어느 시점에도 팩터 추가 가능
- 루프 클로저: pose 간 제약으로 표현
- 견고한 outlier 제거: Huber / Cauchy robust kernel 적용

### 1.3 하이브리드 구조의 이점

```
실시간성 (ESKF, ~200Hz)  +  정확성 (FGO, ~10Hz)  +  견고성 (outlier rejection)
```

ESKF가 고빈도 실시간 출력을 담당하고, FGO가 정제된 pose를 사후 제공하여 ESKF의 기준점을 주기적으로 교정한다.

---

## 2. 시스템 아키텍처

### 2.1 프로세스 토폴로지

```
┌───────────────────────────────────────────────────┐
│  Process: hybrid_localization_node                │
│                                                   │
│  ┌─────────────────────────────────────────────┐  │
│  │  ESKF Frontend  (200 Hz, 실시간)            │  │
│  │                                             │  │
│  │  IMU propagate() ──→ State + Covariance     │  │
│  │  GNSS update()  ──→ NIS gate + correction   │  │
│  │  Vehicle update()─→ NHC / ZUPT              │  │
│  │       │                                     │  │
│  │       ├── publish /localization/odom (실시간)│  │
│  │       └── push KeyframeBuffer               │  │
│  └──────────────────────┬──────────────────────┘  │
│                         │  KeyframeBuffer         │
│  ┌──────────────────────▼──────────────────────┐  │
│  │  FGO Backend   (10 Hz, 슬라이딩 윈도우)     │  │
│  │                                             │  │
│  │  GTSAM / g2o Factor Graph                   │  │
│  │  ├── IMUPreintegration factor               │  │
│  │  ├── GNSSPositionFactor                     │  │
│  │  ├── GNSSVelocityFactor                     │  │
│  │  ├── HeadingFactor                          │  │
│  │  ├── LidarOdometryFactor (delayed OK)       │  │
│  │  └── PriorFactor (marginalization anchor)   │  │
│  │       │                                     │  │
│  │       ├── publish /localization/fgo_odom    │  │
│  │       └── ESKFCorrector::apply()            │  │
│  └─────────────────────────────────────────────┘  │
└───────────────────────────────────────────────────┘
```

### 2.2 데이터 흐름

```
IMU (200Hz) ──┐
              ├──→ [ESKF propagate] ──→ odom (실시간)
GNSS (10Hz) ──┤         │
Vehicle (50Hz)┘         │ KeyframeBuffer::push(stamp, state, imu_preint)
                        │
                        ▼
               [FGO Backend::optimize()]
                        │
                        ├── GNSS factors (delayed OK)
                        ├── LiDAR factors (latency ~200ms OK)
                        └── ESKFCorrector::apply(refined_pose)
                                   │
                                   ▼
                         [ESKF 기준점 재설정]
```

---

## 3. 상태 벡터 정의

### 3.1 ESKF 명목 상태 (15-DOF)

기존 eskf_localization 상태 벡터를 그대로 계승한다.

| 인덱스 | 기호 | 크기 | 단위 | 설명 |
|--------|------|------|------|------|
| 0–2   | `p`  | 3    | m    | map 프레임 위치 |
| 3–5   | `v`  | 3    | m/s  | map 프레임 속도 |
| 6–9   | `q`  | 4(3) | —    | 자세 (쿼터니언 / 오차상태는 so(3) 3DOF) |
| 10–12 | `b_g`| 3    | rad/s| 자이로 바이어스 |
| 13–15 | `b_a`| 3    | m/s² | 가속도계 바이어스 |

오차 상태 벡터 `δx ∈ ℝ¹⁵`.

### 3.2 FGO 노드 상태 (각 키프레임)

FGO 그래프의 각 노드(키프레임)는 아래를 포함한다.

| 변수 | 타입 | 설명 |
|------|------|------|
| `T_i` | SE(3) | map←base_link 변환 (pose) |
| `v_i` | ℝ³   | map 프레임 선속도 |
| `b_g_i` | ℝ³ | 자이로 바이어스 |
| `b_a_i` | ℝ³ | 가속도계 바이어스 |

슬라이딩 윈도우 크기: **W = 20 키프레임** (≒ 2초 @ 10Hz).

---

## 4. 센서 입력 및 팩터 종류

### 4.1 센서 입력 요약

| 센서 | 토픽 | 메시지 타입 | 빈도 |
|------|------|-------------|------|
| IMU | `/sensing/imu/imu_raw` | `sensor_msgs/Imu` | 200 Hz |
| GNSS 위치 | `/sensing/gnss/navsatfix` | `sensor_msgs/NavSatFix` | 10 Hz |
| GNSS 속도 | `/sensing/gnss/vel` | `geometry_msgs/TwistStamped` | 10 Hz |
| GNSS 방위 | `/sensing/gnss/heading` | `skyautonet_msgs/Gphdt` | 10 Hz |
| 차량 속도 | `/vehicle/status/velocity_status` | `autoware_vehicle_msgs/VelocityReport` | 50 Hz |
| 차량 조향 | `/vehicle/status/steering_status` | `autoware_vehicle_msgs/SteeringReport` | 50 Hz |
| LiDAR Odom | `/localization/lidar_odom` | `nav_msgs/Odometry` | 10 Hz (지연 ~200ms) |

### 4.2 FGO 팩터 정의

#### (a) IMU 사전적분 팩터 (IMUPreintegrationFactor)

연속된 키프레임 `i → j` 사이 IMU 측정치를 사전적분하여 상대 pose/velocity 제약을 생성.

```
residual = Δ̂R_ij^T * (R_i^T * R_j) - I
         + Δ̂v_ij - R_i^T * (v_j - v_i - g*Δt)
         + Δ̂p_ij - R_i^T * (p_j - p_i - v_i*Δt - 0.5*g*Δt²)
         + b_g_j - b_g_i
         + b_a_j - b_a_i
```

노이즈 파라미터: 기존 ESKF Q 행렬에서 인계.

#### (b) GNSS 위치 팩터 (GNSSPositionFactor)

```
residual = p_i - p_gnss
noise    = R_gnss (NavSatFix.covariance 또는 default)
robust   = Huber(k=0.5) [멀티패스 대응]
```

GNSS status 기반 활성화:
- status `-1`: skip
- status `0` (RTK Fixed): R × 1.0
- status `1` (RTK Float): R × 100

#### (c) GNSS 속도 팩터 (GNSSVelocityFactor)

```
residual = v_i - R_ENU_to_map * v_gnss_enu
noise    = diag(0.1, 0.1, 0.3)² [m/s]
```

#### (d) 방위 팩터 (HeadingFactor)

```
residual = yaw(T_i) - yaw_gnss
noise    = σ²_yaw (파라미터)
```

최대 변화율 게이팅: `|Δyaw/Δt| < 1.0 rad/s`.

#### (e) LiDAR 오도메트리 팩터 (LidarOdometryFactor)

KISS-ICP 또는 NDT 스캔 매칭 결과를 상대 pose 제약으로 추가.
지연 허용: 수신 시점의 타임스탬프로 윈도우 내 노드에 삽입.

```
residual = T_i^{-1} * T_j - ΔT_lidar
noise    = Σ_lidar (스캔 매칭 공분산)
robust   = Cauchy(k=1.0)
```

활성화 조건: GNSS status `≤ 1` 또는 `use_lidar_factor: true` 설정.

#### (f) 비홀로노믹 제약 팩터 (NHCFactor)

차량 side/vertical 속도 ≈ 0 제약.

```
residual = [v_base.y, v_base.z]
noise    = diag(0.05, 0.1)² [m/s]
```

#### (g) 마지널라이제이션 prior (MarginalizationFactor)

슬라이딩 윈도우에서 탈락하는 가장 오래된 키프레임을 Schur complement로 주변화하여 prior로 고정.

---

## 5. 슬라이딩 윈도우 관리

### 5.1 키프레임 선택 정책

새 키프레임은 아래 조건 중 하나를 만족하면 생성:

| 조건 | 임계값 |
|------|--------|
| 이동 거리 | ≥ 0.5 m |
| 회전 변화 | ≥ 5° |
| 경과 시간 | ≥ 0.1 s (최대 빈도 10 Hz) |

### 5.2 윈도우 크기 및 드롭

- 최대 윈도우: **W = 20 키프레임**
- 초과 시: 가장 오래된 키프레임 마지널라이제이션 → prior factor 추가 후 제거
- 마지널라이제이션 구현: Schur complement (GTSAM `Marginals` API 또는 직접 구현)

### 5.3 최적화 실행 주기

- 기본: **10 Hz** (100ms 주기 타이머)
- 최적화 라이브러리: **GTSAM 4.x** (권장) 또는 g2o
- 솔버: Levenberg-Marquardt (최대 iteration 5)

---

## 6. ESKF ↔ FGO 인터페이스

### 6.1 KeyframeBuffer

ESKF에서 FGO로 데이터를 전달하는 스레드 안전 버퍼.

```cpp
struct Keyframe {
    rclcpp::Time stamp;
    NominalState eskf_state;   // p, v, q, b_g, b_a
    Eigen::Matrix<double,15,15> P;  // 공분산
    ImuPreintegration preint;  // 이전 키프레임 이후 적분값
};

class KeyframeBuffer {
    std::deque<Keyframe> buffer_;
    std::mutex mtx_;
public:
    void push(const Keyframe& kf);
    std::optional<Keyframe> pop_oldest();
    std::vector<Keyframe> get_window(size_t n);
};
```

### 6.2 ESKFCorrector

FGO 결과를 ESKF에 반영하는 교정기.

```cpp
class ESKFCorrector {
public:
    // FGO가 최적화한 최신 키프레임 pose를 ESKF 기준점으로 재설정
    void apply(const rclcpp::Time& stamp,
               const NominalState& refined_state,
               const Eigen::Matrix<double,15,15>& refined_P);
private:
    // ESKF 기준점 교체 + 이후 IMU 재적분
    void reintegrate_from(const rclcpp::Time& anchor_stamp);
};
```

**교정 절차:**
1. FGO가 윈도우 내 최신 키프레임 상태 `x*_k` 출력
2. `ESKFCorrector::apply(t_k, x*_k, P*_k)` 호출
3. ESKF 명목 상태를 `x*_k`로 교체
4. `t_k` 이후 버퍼된 IMU 데이터로 현재 시각까지 재적분
5. 교정된 상태로 다음 ESKF propagate 계속

### 6.3 교정 빈도 및 안전장치

| 항목 | 값 |
|------|-----|
| 교정 빈도 | 10 Hz (FGO 최적화 주기와 동일) |
| 최대 교정 크기 (위치) | 1.0 m (초과 시 경고 + 단계적 적용) |
| 최대 교정 크기 (방위) | 5° (초과 시 경고) |
| 교정 후 ESKF 공분산 | FGO 공분산으로 대체 |

---

## 7. 초기화 절차

```
Phase 0 — GNSS/IMU 대기
  └── GNSS status == 0 (RTK Fixed) 확인
  └── IMU 정적 calibration (2초): b_a, b_g 추정

Phase 1 — ESKF 초기화 (기존 eskf_localization과 동일)
  └── 첫 GNSS position → p 초기화
  └── 첫 GNSS heading → q 초기화
  └── 공분산 초기화 (P0)

Phase 2 — FGO 그래프 시작
  └── 첫 W 키프레임 수집 후 그래프 구성
  └── 최초 최적화 실행
  └── ESKFCorrector 활성화

Phase 3 — 정상 운용
  └── ESKF 실시간 출력 + FGO 주기적 교정
```

---

## 8. ROS 2 패키지 구조

```
ros/src/localization/
└── hybrid_localization/
    ├── package.xml
    ├── CMakeLists.txt
    ├── config/
    │   └── hybrid_localization.param.yaml
    ├── launch/
    │   └── hybrid_localization.launch.xml
    ├── include/
    │   └── hybrid_localization/
    │       ├── hybrid_localization_node.hpp   # ROS2 노드
    │       ├── eskf/
    │       │   ├── eskf_core.hpp              # (eskf_localization에서 이식)
    │       │   ├── eskf_corrector.hpp         # FGO→ESKF 교정
    │       │   └── ...
    │       ├── fgo/
    │       │   ├── fgo_backend.hpp            # 슬라이딩 윈도우 최적화
    │       │   ├── keyframe_buffer.hpp        # ESKF→FGO 버퍼
    │       │   ├── factors/
    │       │   │   ├── imu_preintegration_factor.hpp
    │       │   │   ├── gnss_position_factor.hpp
    │       │   │   ├── gnss_velocity_factor.hpp
    │       │   │   ├── heading_factor.hpp
    │       │   │   ├── lidar_odom_factor.hpp
    │       │   │   └── nhc_factor.hpp
    │       │   └── marginalization.hpp
    │       ├── preprocess/                    # (eskf_localization에서 이식)
    │       │   ├── imu_preprocessor.hpp
    │       │   ├── gnss_preprocessor.hpp
    │       │   └── gnss_heading_arbitrator.hpp
    │       └── util/
    │           ├── map_projector.hpp
    │           ├── tf_cache.hpp
    │           └── time_processing.hpp
    ├── src/
    │   ├── main.cpp
    │   ├── hybrid_localization_node.cpp
    │   ├── eskf/
    │   │   ├── eskf_core.cpp
    │   │   └── eskf_corrector.cpp
    │   ├── fgo/
    │   │   ├── fgo_backend.cpp
    │   │   ├── keyframe_buffer.cpp
    │   │   ├── factors/
    │   │   │   ├── imu_preintegration_factor.cpp
    │   │   │   └── ...
    │   │   └── marginalization.cpp
    │   ├── preprocess/
    │   └── util/
    └── test/
        ├── test_eskf_core.cpp
        ├── test_imu_preintegration.cpp
        ├── test_fgo_backend.cpp
        └── test_eskf_corrector.cpp
```

---

## 9. ROS 2 토픽 인터페이스

### 9.1 Subscribe

| 토픽 | 타입 | 역할 |
|------|------|------|
| `/sensing/imu/imu_raw` | `sensor_msgs/Imu` | IMU propagate |
| `/sensing/gnss/navsatfix` | `sensor_msgs/NavSatFix` | GNSS 위치 팩터 |
| `/sensing/gnss/vel` | `geometry_msgs/TwistStamped` | GNSS 속도 팩터 |
| `/sensing/gnss/heading` | `skyautonet_msgs/Gphdt` | 방위 팩터 |
| `/vehicle/status/velocity_status` | `autoware_vehicle_msgs/VelocityReport` | NHC/ZUPT |
| `/vehicle/status/steering_status` | `autoware_vehicle_msgs/SteeringReport` | 요레이트 제약 |
| `/localization/lidar_odom` | `nav_msgs/Odometry` | LiDAR 오도메트리 팩터 |
| `/map/map_projector_info` | `tier4_map_msgs/MapProjectorInfo` | 좌표 투영 |
| `/initialpose3d` | `geometry_msgs/PoseWithCovarianceStamped` | 외부 초기화 |

### 9.2 Publish

| 토픽 | 타입 | 내용 |
|------|------|------|
| `/localization/kinematic_state` | `nav_msgs/Odometry` | ESKF 실시간 출력 (200 Hz) |
| `/localization/fgo_state` | `nav_msgs/Odometry` | FGO 정제 출력 (10 Hz) |
| `/localization/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 상태 진단 |
| `/localization/debug/window` | `visualization_msgs/MarkerArray` | 슬라이딩 윈도우 시각화 |

---

## 10. 파라미터 (config/hybrid_localization.param.yaml)

```yaml
hybrid_localization:
  # 공통
  frame_id: "map"
  base_frame_id: "base_link"
  imu_frame_id: "imu_link"
  ipc_socket_path: "/tmp/autodriver/localization.sock"

  # ESKF (eskf_localization 파라미터 계승)
  eskf:
    use_accel_propagation: false  # 기존 정책 유지
    imu:
      gyro_noise_std: 0.05        # rad/s/√Hz
      accel_noise_std: 1.0        # m/s²/√Hz
      gyro_bias_rw_std: 1.0e-4
      accel_bias_rw_std: 1.0e-3
      gyro_lpf_cutoff_hz: 10.0
    gnss:
      cov_scale: 1.0
      pos_inflate_status_fix: 1.0
      pos_inflate_status_sbas: 100.0
      recover:
        holdoff_sec: 1.0
        ramp_sec: 5.0
    heading:
      yaw_var: 0.01
      max_rate_radps: 1.0
    vehicle:
      speed_var: 0.01
      nhc_var: 0.05
      wheelbase_m: 2.85
      nhc_min_speed_mps: 0.3

  # FGO 백엔드
  fgo:
    window_size: 20              # 키프레임 수
    optimize_rate_hz: 10.0
    solver:
      max_iterations: 5
      lambda_initial: 1.0e-5
    keyframe:
      min_dist_m: 0.5
      min_angle_deg: 5.0
      max_interval_sec: 0.1
    factors:
      gnss_position:
        enable: true
        robust_kernel: "huber"
        robust_k: 0.5
      gnss_velocity:
        enable: true
      heading:
        enable: true
      lidar_odom:
        enable: true
        robust_kernel: "cauchy"
        robust_k: 1.0
        max_delay_sec: 0.5
      nhc:
        enable: true
    marginalization:
      enable: true

  # ESKF 교정기
  corrector:
    max_pos_correction_m: 1.0
    max_yaw_correction_deg: 5.0
    enable_reintegration: true
```

---

## 11. 의존성

### 11.1 외부 라이브러리

| 라이브러리 | 버전 | 용도 |
|------------|------|------|
| GTSAM | 4.2+ | FGO 팩터 그래프, 최적화 솔버 |
| Eigen | 3.4+ | 행렬/벡터 연산 |
| GeographicLib | 2.x | WGS84↔ENU 좌표 변환 |

### 11.2 ROS 2 패키지

| 패키지 | 역할 |
|--------|------|
| `rclcpp` | ROS 2 C++ 클라이언트 |
| `sensor_msgs`, `nav_msgs`, `geometry_msgs` | 표준 메시지 |
| `autoware_vehicle_msgs` | 차량 상태 메시지 |
| `tier4_map_msgs` | 지도 투영 메시지 |
| `tf2_ros` | 좌표 변환 |
| `diagnostic_msgs` | 진단 |
| `autodriver_cmake` | 빌드 인프라 (CUDA, TRT finder) |

---

## 12. 개발 단계 계획

| 단계 | 내용 | 산출물 |
|------|------|--------|
| **Stage 1** | ESKF 이식: eskf_localization → hybrid_localization | `eskf_core.cpp`, `hybrid_localization_node.cpp` (ESKF only) |
| **Stage 2** | KeyframeBuffer + ImuPreintegration 구현 | `keyframe_buffer.cpp`, `imu_preintegration_factor.cpp` |
| **Stage 3** | FGO 백엔드 구현 (GNSS 팩터만) | `fgo_backend.cpp`, GNSS 팩터, 마지널라이제이션 |
| **Stage 4** | ESKFCorrector 구현 + 통합 테스트 | `eskf_corrector.cpp`, 단위 테스트 |
| **Stage 5** | LiDAR 오도메트리 팩터 + 견고 커널 | `lidar_odom_factor.cpp` |
| **Stage 6** | 파라미터 튜닝, 진단, 시각화 | 파라미터 yaml, 진단 publisher |

---

## 13. 기존 ESKF와의 차이점 요약

| 항목 | eskf_localization | hybrid_localization |
|------|-------------------|---------------------|
| 추정 방식 | 재귀 EKF | EKF 프론트엔드 + FGO 백엔드 |
| 지연 측정치 | 불가 | 슬라이딩 윈도우 내 가능 |
| LiDAR 활용 | 미구현 (TODO) | LidarOdometryFactor |
| outlier 제거 | NIS gate (soft) | Robust kernel (Huber/Cauchy) |
| 루프 클로저 | 불가 | PoseBetweenFactor 추가 가능 |
| 공분산 전파 | 단일 P 행렬 | 윈도우 내 전 상태 공분산 |
| 실시간 출력 | O (200 Hz) | O (200 Hz, ESKF) |
| 정제 출력 | — | O (10 Hz, FGO) |
| 의존성 추가 | 없음 | GTSAM |
