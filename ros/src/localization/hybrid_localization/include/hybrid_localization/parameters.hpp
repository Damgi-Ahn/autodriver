#ifndef HYBRID_LOCALIZATION__PARAMETERS_HPP_
#define HYBRID_LOCALIZATION__PARAMETERS_HPP_

#include <string>

#include <rclcpp/node.hpp>

#include "hybrid_localization/eskf/eskf_core.hpp"
#include "hybrid_localization/fgo/eskf_corrector.hpp"
#include "hybrid_localization/fgo/fgo_backend.hpp"
#include "hybrid_localization/fgo/imu_preintegration.hpp"
#include "hybrid_localization/fgo/keyframe_buffer.hpp"
#include "hybrid_localization/preprocess/gnss_heading_arbitrator.hpp"
#include "hybrid_localization/preprocess/imu_preprocessor.hpp"
#include "hybrid_localization/types.hpp"

namespace hybrid_localization
{

// NOTE: 노드 파라미터를 도메인별로 구조화하여 로딩/적용/참조를 단순화한다.
struct HeadingYawParams
{
  // `heading.enable_yaw_update` [-]
  // - GNSS 헤딩(GPHDT) 기반 yaw 업데이트 on/off.
  // - OFF: yaw는 IMU/기타 제약에 더 의존(드리프트↑ 가능), 점프/이상치 영향↓
  bool enable_yaw_update{true};

  // `heading.yaw_var` [rad^2]
  // - ↓: 헤딩 yaw를 더 강하게 신뢰(수렴 빠름), 헤딩 점프/멀티패스에 취약
  // - ↑: 헤딩 yaw 영향 약화(안정성↑), yaw 드리프트↑ 가능
  double yaw_var{0.012};

  // `heading.max_rate_radps` [rad/s]
  // - yaw 변화율 게이트. 너무 급격한 yaw 변화 샘플을 배제하는 용도.
  // - ↓: 더 엄격(점프 억제↑), 실제 급회전에서도 스킵↑
  // - ↑: 더 관대(급회전 추종↑), 점프 샘플 통과↑
  double max_rate_radps{1.0};

  // `heading.rate_gate_skip_max_count` [-]
  // - rate gate로 연속 스킵이 누적될 때 복구(리셋/바이패스) 트리거로 사용.
  // - <=0: count 기반 복구 비활성
  int rate_gate_skip_max_count{20};

  // `heading.rate_gate_skip_max_sec` [s]
  // - 연속 rate gate 스킵 지속 시간이 길어질 때 복구(리셋/바이패스) 트리거로 사용.
  // - <=0: time 기반 복구 비활성
  double rate_gate_skip_max_sec{2.0};

  // `heading.rate_gate_bypass_yaw_var_scale` [-]
  // - rate gate 복구 바이패스 시 yaw_var를 일시적으로 inflate하여 업데이트를 약하게 적용.
  // - <1이면 1로 클램프
  double rate_gate_bypass_yaw_var_scale{10.0};
};

struct VehicleConstraintParams
{
  // `vehicle.enable_speed_update`
  // - 차량 속도(예: OBD)로 body x-velocity를 업데이트.
  bool enable_speed_update{true};

  // `vehicle.speed_use_abs`
  // - 속도 업데이트에 절대값 사용 여부(후진/부호 처리 정책).
  bool speed_use_abs{false};

  // `vehicle.speed_var` [(m/s)^2]
  // - ↓: 속도 측정을 더 강하게 신뢰(드리프트↓), 센서 튐에 취약
  // - ↑: 속도 영향 약화(안정성↑), 속도 드리프트↑ 가능
  double speed_var{0.25};

  double speed_factor{1.0};

  // `vehicle.min_speed_mps_for_speed_update` [m/s]
  // - 저속에서 속도 업데이트를 스킵하기 위한 임계값.
  double min_speed_mps_for_speed_update{1.0};

  // `vehicle.enable_nhc`
  // - NHC(Non-Holonomic Constraint): v_base.y, v_base.z ≈ 0 제약 업데이트 on/off.
  bool enable_nhc{true};

  // `vehicle.nhc_var` [(m/s)^2]
  // - ↓: 제약 강함(횡/수직 드리프트 억제↑), 모델 부정합 시 왜곡↑
  // - ↑: 제약 약함(안정성↑), 횡/수직 드리프트 억제↓
  double nhc_var{0.04};

  // `vehicle.enable_zupt`
  // - ZUPT(Zero-velocity update): 정지 근처에서 v_base.x ≈ 0 제약 업데이트 on/off.
  bool enable_zupt{false};

  // `vehicle.zupt_speed_threshold_mps` [m/s]
  // - 이 속도 이하를 정지로 판단(너무 크면 저속 주행을 정지로 오판 가능).
  double zupt_speed_threshold_mps{0.2};

  // `vehicle.zupt_var` [(m/s)^2]
  // - ↓: 정지 시 속도를 더 강하게 0으로 끌어감(드리프트↓), 오판 시 충격↑
  // - ↑: ZUPT 영향 약화(안정성↑), 정지 드리프트↓ 효과 감소
  double zupt_var{0.01};

  // `vehicle.enable_yaw_rate_update`
  // - 조향 기반 yaw-rate로 gyro bias(z)를 제약하는 업데이트 on/off.
  bool enable_yaw_rate_update{true};
  double yaw_rate_min_speed_mps{1.0};
  double yaw_rate_var{0.0004};
};

struct GnssParams
{
  struct RecoverParams
  {
    // `gnss.recover.enable`
    // - GNSS status 개선(upgrade) 직후 transition을 부드럽게 만들기 위한 holdoff+ramp 사용 여부.
    bool enable{true};

    // `gnss.recover.holdoff_sec` [s]
    // - status 개선 직후 일정 시간 GNSS pos/vel 업데이트를 스킵(쓰레기 샘플 방어).
    double holdoff_sec{1.0};

    // `gnss.recover.ramp_sec` [s]
    // - holdoff 이후 recover inflate를 지수적으로 감쇠(peak->1).
    double ramp_sec{5.0};

    // `gnss.recover.pos_max_inflate` [-]
    // - recover peak 추정이 어려울 때 사용하는 position inflate fallback.
    double pos_max_inflate{1000.0};

    // `gnss.recover.vel_max_inflate` [-]
    // - recover peak 추정이 어려울 때 사용하는 velocity inflate fallback.
    double vel_max_inflate{50.0};
  };

  // `enable_gnss_pos_update` / `enable_gnss_vel_update`
  // - GNSS 위치/속도 업데이트 on/off.
  bool enable_gnss_pos_update{true};
  bool enable_gnss_vel_update{true};

  // `gnss.use_navsatfix_covariance`
  // - NavSatFix의 covariance 사용 여부.
  bool use_navsatfix_covariance{true};

  // `gnss.covariance_scale`
  // - GNSS covariance(공분산)를 스케일링해 과신을 방지.
  // - ↑: GNSS를 덜 신뢰(업데이트 약화), 드리프트↑ 가능
  // - ↓: GNSS를 더 신뢰(수렴 빠름), 점프/멀티패스에 취약
  double cov_scale{25.0};

  // `gnss.pos_var_fallback` [m^2]
  // - covariance가 없거나 유효하지 않을 때 사용하는 위치 분산.
  double pos_var_fallback{4.0};
  double pos_var_min{0.0025};
  double pos_var_max{1.0e7};
  bool pos_cov_diag_only{true};

  // `gnss.pos_inflate_status_fix/sbas`
  // - GNSS status가 나쁠 때 위치 분산을 추가로 inflate하여 영향력을 낮춤.
  // - ↑: 해당 status에서 위치 업데이트 영향 감소(강한 다운웨이트)
  // - ↓: 해당 status에서도 위치 업데이트를 더 사용
  double pos_inflate_status_fix{1000000.0};
  double pos_inflate_status_sbas{100.0};

  // `gnss.vel_covariance_scale`
  // - GNSS 속도 covariance 스케일링.
  double vel_cov_scale{1.0};
  double vel_var_fallback{0.49};
  double vel_var_min{0.01};
  double vel_var_max{1.0e4};

  // `gnss.min_status_for_pos_update`
  // - 이 status 미만이면 GNSS 위치 업데이트를 스킵.
  int min_status_for_pos_update{0};

  // `heading.min_gnss_status_for_yaw_update`
  // - 이 status 미만이면 GPHDT yaw 업데이트를 차단(arb에서 사용).
  int min_status_for_yaw_update{1};

  // `gnss.vel_inflate_status_fix/sbas`
  // - GNSS status가 나쁠 때 속도 분산을 inflate하여 영향력을 낮춤.
  // - ↑: 해당 status에서 속도 업데이트 영향 감소
  // - ↓: 해당 status에서도 속도 업데이트를 더 사용
  double vel_inflate_status_fix{25.0};
  double vel_inflate_status_sbas{4.0};

  // Optional GNSS pose output for pose_initializer compatibility (Case B integration).
  // This publishes a map-frame PoseWithCovarianceStamped derived from NavSatFix projection.
  // Default is OFF to avoid topic conflicts when other nodes already publish it.
  bool publish_pose_with_covariance{false};
  std::string pose_with_covariance_topic{"/sensing/gnss/pose_with_covariance"};
  bool pose_with_covariance_use_base_position{true};

  // Transition smoothing on status upgrade
  RecoverParams recover{};
};

struct OutputParams
{
  // `output.flatten_roll_pitch`
  // - 출력 자세를 평면 차량 모드로 사용(yaw 중심, roll/pitch를 약하게/평탄화).
  bool flatten_roll_pitch{true};

  // `output.roll_pitch_var` [rad^2]
  // - flatten_roll_pitch 사용 시 roll/pitch 제약 강도(분산).
  // - ↓: roll/pitch를 더 강하게 눌러 평탄하게(노면 경사 반영↓)
  // - ↑: roll/pitch 제약 약화(roll/pitch 움직임 허용↑)
  double roll_pitch_var{1000.0};
};

struct LocalizationInitParams
{
  // Initialization mode:
  // - "auto": use existing internal init logic (GNSS+heading cache, etc.)
  // - "external_initialpose": wait for external initial pose topic (Autoware /initialpose3d)
  std::string mode{"auto"};

  // If true, node runs only after being activated via SetBool trigger service.
  // This mirrors Autoware ekf_localizer behavior (pose_initializer triggers activation).
  bool require_trigger{false};

  // External initial pose topic name (internal topic name; can be remapped in launch).
  // Typically remapped to "/initialpose3d" in Autoware pipeline mode.
  std::string external_initialpose_topic{"initialpose"};

  // If true, every incoming external initial pose re-initializes the filter state.
  // If false, only the first message initializes when not yet initialized.
  bool reset_on_external_pose{true};
};

struct HybridLocalizationNodeParams
{
  // 핵심 파라미터 묶음 (기존 타입 재사용)
  ESKFParameters io{};
  TimeProcessingParams time{};
  VehicleParams vehicle{};
  ImuPreprocessParams imu{};
  EskfCoreParams eskf{};

  // 부가 동작 제어 파라미터
  double time_alignment_tolerance_sec{0.0};

  // 기능별 옵션
  HeadingYawParams heading{};
  GnssHeadingArbitratorParams heading_arbitrator{};
  VehicleConstraintParams vehicle_constraints{};
  GnssParams gnss{};
  OutputParams output{};
  LocalizationInitParams init{};

  // FGO Stage 2: IMU 사전적분 + 키프레임 선택
  ImuPreintegration::Params imu_preint{};
  KeyframeSelectionParams keyframe{};

  // FGO Stage 3: ISAM2 백엔드
  FgoBackend::Params fgo_backend{};

  // FGO Stage 4+5: EskfCorrector
  EskfCorrector::Params fgo_corrector{};

  // ImuBuffer 최대 샘플 수 (200Hz×2s=400 기본값)
  int imu_buffer_max_samples{400};

  // GNSS 불량 → DEGRADED 상태 전이 타임아웃 [s]
  double gnss_degraded_timeout_sec{5.0};

  // 캘리브레이션 관련
  bool init_imu_calibration{false};
  std::string calibration_file_path{};

  void load(rclcpp::Node & node);
};

} // namespace hybrid_localization

#endif // HYBRID_LOCALIZATION__PARAMETERS_HPP_
