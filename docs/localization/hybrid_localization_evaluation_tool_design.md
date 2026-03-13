# Hybrid Localization 평가 툴 설계 (1차)

## 1) 목적/범위

`ros/src/localization/hybrid_localization` 노드가 발행하는 **진단 데이터(/diagnostics)**와
**주요 출력 토픽**을 기반으로, Localization 품질을 정량/정성 평가하는
ROS2 + C++17 + Qt6 기반 데스크톱 툴의 초기 설계를 정의한다.

이번 문서의 초점:
- 하이브리드 localization의 진단 데이터 스키마 분석
- 같은 노드가 발행하는 토픽/메시지 분석
- 평가 툴 아키텍처(초안) 제시

---

## 2) 분석 대상 노드 I/O 요약

### 2.1 하이브리드 localization 노드의 출력 토픽

| 구분 | 토픽 | 타입 | 발행 조건/특징 |
|---|---|---|---|
| 핵심 결과 | `/localization/kinematic_state` (기본값) | `nav_msgs/msg/Odometry` | 노드 활성 + ESKF 초기화 + heading 수신 시 발행 |
| 핵심 결과(파생) | `/localization/pose_twist_fusion_filter/pose` | `geometry_msgs/msg/PoseStamped` | 위 odom 발행 시 함께 발행 |
| GNSS 보조 출력 | `/localization/kinematic_state_gnss` | `nav_msgs/msg/Odometry` | GNSS callback에서 발행 |
| GNSS pose(optional) | `/sensing/gnss/pose_with_covariance` (기본값) | `geometry_msgs/msg/PoseWithCovarianceStamped` | `gnss.publish_pose_with_covariance=true` 일 때만 |
| 전처리 IMU | `/sensing/imu/imu_data` | `sensor_msgs/msg/Imu` | IMU callback에서 전처리 후 발행 |
| GNSS 기반 속도 | `/sensing/gnss/velocity_status` | `autoware_vehicle_msgs/msg/VelocityReport` | GNSS velocity callback에서 발행 |
| 진단 | `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | publish timer 기준 10회 중 1회 다운샘플 발행 |
| TF | `/tf` (`map -> base_link`) | `tf2_msgs/msg/TFMessage` | `publish_tf=true` 이고 publish ready일 때 브로드캐스트 |

### 2.2 입력(평가 연계용 관측 대상)

평가 툴은 최소 아래 입력을 함께 구독해 상관 분석해야 한다.

- `/sensing/imu/imu_raw` (`sensor_msgs/msg/Imu`)
- `/sensing/gnss/navsatfix` (`sensor_msgs/msg/NavSatFix`)
- `/sensing/gnss/vel` (`geometry_msgs/msg/TwistStamped`)
- `/sensing/gnss/heading` (`skyautonet_msgs/msg/Gphdt`)
- `/vehicle/status/velocity_status` (`autoware_vehicle_msgs/msg/VelocityReport`)
- `/vehicle/status/steering_status` (`autoware_vehicle_msgs/msg/SteeringReport`)

---

## 3) 진단 데이터(/diagnostics) 상세 분석

`DiagnosticArray.status[0]`에 `name="hybrid_localization"` 엔트리 하나를 구성하고,
다수 Key-Value를 문자열로 담아 발행한다.

## 3.1 발행 주기

- publish timer는 `io.publish_rate` (기본 200Hz)로 동작
- 진단은 내부 카운터(`m_diag_counter`)로 10회에 1회 발행
- 즉 기본 약 **20Hz 진단 스트림**

## 3.2 주요 진단 키 그룹

### A. 상태/카운터
- `is_activated`
- `eskf_initialized`
- `imu_count`, `gnss_count`, `gnss_vel_count`, `velocity_count`, `steering_count`

### B. 업데이트 적용 여부 및 이유
- GNSS 위치: `gnss_pos_update_applied`, `gnss_pos_update_reason`, `gnss_pos_nis`, `gnss_pos_residual_norm`
- GNSS 속도: `gnss_vel_update_applied`, `gnss_vel_update_reason`, `gnss_vel_nis`, `gnss_vel_residual_norm`
- Heading yaw: `heading_yaw_update_applied`, `heading_yaw_update_reason`, `heading_yaw_nis`, `heading_yaw_residual_rad`

### C. GNSS 상태/측정 공분산(R)
- `gnss_status`
- Position R: `gnss_pos_R_xx`, `gnss_pos_R_yy`, `gnss_pos_R_zz`
- Velocity R: `gnss_vel_R_xx`, `gnss_vel_R_yy`, `gnss_vel_R_zz`
- Inflate 계수: `gnss_pos_status_inflate`, `gnss_pos_nis_inflate`, `gnss_vel_status_inflate`, `gnss_vel_nis_inflate`

### D. Heading R/Inflate/소스
- `heading_yaw_var`, `heading_yaw_var_eff`, `heading_yaw_var_applied`
- `heading_yaw_nis_inflate`
- `heading_status_inflate`, `heading_recover_inflate`
- `heading_yaw_var_source` (예: `normal`, `inflated_status`, `decay`)

### E. Vehicle 제약 노이즈
- `vehicle_speed_var`, `vehicle_nhc_var`, `vehicle_zupt_var`, `vehicle_yaw_rate_var`

### F. IMU 프로세스 노이즈(Q)
- `imu_gyro_noise_std`, `imu_accel_noise_std`
- `imu_gyro_bias_noise_std`, `imu_accel_bias_noise_std`

### G. 필터 공분산(P) 건강도
- `P_trace`, `P_max_diag`, `P_min_diag`, `P_min_eig`
- 블록별 최대 대각: `P_pos_max_diag`, `P_vel_max_diag`, `P_att_max_diag`, `P_bg_max_diag`, `P_ba_max_diag`

### H. 센서 지연/주기
- `imu_dt_min`, `imu_dt_mean`, `imu_dt_max`
- `gnss_delay`, `gnss_vel_delay`, `velocity_delay`, `steering_delay`

### I. KISS 관련 디버그(현재 has_kiss=true로 프레임 제공)
- enable/initialized/source/trust/nis/skip_reason/reset 등 다수 키
- 예: `kiss_enabled`, `kiss_initialized`, `kiss_trust`, `kiss_skip_reason`, `kiss_reset_count`

---

## 4) 평가 툴에서 우선 산출할 KPI 제안

1. **업데이트 적용률**
   - pos/vel/heading 각각 `applied=true 비율`
2. **게이팅/스킵 원인 분포**
   - `*_update_reason` 히스토그램 (time_alignment, gnss_status_skip, recover_holdoff, non_finite_measurement 등)
3. **NIS 품질 지표**
   - `gnss_pos_nis`, `gnss_vel_nis`, `heading_yaw_nis`의 평균/95p/최대
4. **불확실성 안정성**
   - `P_trace`, `P_min_eig` 추세 및 임계 초과 시간 비율
5. **센서 지연 건강도**
   - delay 평균/최대/경보 횟수
6. **출력 가용성**
   - `/localization/kinematic_state` publish-ready 구간 비율

---

## 5) ROS2 + C++17 + Qt6 기반 평가 툴 아키텍처 (초안)

## 5.1 구성도

- `EvaluationNode` (rclcpp)
  - 대상 토픽 구독, 파싱, 내부 버퍼 저장
  - KPI 계산 엔진 호출
- `KpiEngine` (순수 C++17)
  - 시간창(Window) 기반 통계 계산
  - 룰 기반 경보 판정
- `EvaluationMainWindow` (Qt6 Widgets)
  - 실시간 대시보드(상태, 그래프, 경보)
- `RosQtBridge` (Thread-safe Queue)
  - ROS callback thread ↔ Qt UI thread 연결
- `StorageExporter`
  - CSV/JSON 리포트, 스냅샷 내보내기

## 5.2 스레딩 모델

- ROS2 `MultiThreadedExecutor` 1개 스레드풀
- Qt UI 메인 스레드 분리
- 공유 데이터는 lock-free queue 또는 최소 범위 mutex로 보호

## 5.3 Google C++ Style 기준 설계 원칙

- 클래스/파일 책임 단일화 (`evaluation_node.*`, `kpi_engine.*`, `main_window.*`)
- 명확한 네이밍과 상수화 (`kDefaultWindowSec`, `kWarnDelayMs`)
- 헤더에서 인터페이스, 소스에서 구현 분리
- 예외보다 상태 반환/명시적 에러 코드 우선

---

## 6) 초기 구현(Phase-1) 범위

1. `/diagnostics` 파서 구현
   - `hybrid_localization` status만 필터
   - Key-Value를 typed struct로 매핑
2. 핵심 KPI 4종 실시간 계산
   - 업데이트 적용률 / reason 분포 / NIS 요약 / 지연 요약
3. Qt6 대시보드 3패널
   - 상태등(활성/초기화)
   - KPI 카드
   - 최근 경보 로그
4. CSV export
   - 원시 진단 프레임 + 1초 집계 KPI

---

## 7) 다음 단계에서 필요한 추가 확인

- 실제 런타임에서 `diagnostic_msgs/KeyValue` 누락 키 존재 여부(조건부 키 다수)
- `gnss_status` 의미(운영 시스템 표준값) 문서화
- 평가 기준 임계치(예: NIS, delay, P_trace) 운영팀 합의
- rosbag 기반 오프라인 재생 모드 지원 요구사항

