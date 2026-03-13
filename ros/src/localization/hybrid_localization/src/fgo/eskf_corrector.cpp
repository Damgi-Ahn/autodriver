#include "hybrid_localization/fgo/eskf_corrector.hpp"

namespace hybrid_localization
{

int EskfCorrector::apply(
  EskfCore & eskf,
  const FgoBackend::OptimizationResult & result,
  double kf_stamp_sec,
  const ImuBuffer & imu_buffer)
{
  if (!params_.enabled || !result.valid) {
    return 0;
  }

  // Step 1: ESKF 명목 상태를 FGO 최적화 결과로 교체
  eskf.set_nominal_state(result.state);

  // Step 2: keyframe 이후 IMU 샘플 수집
  const auto imu_since = imu_buffer.get_since(kf_stamp_sec);

  int reintegrated = 0;
  for (const auto & meas : imu_since) {
    // 너무 오래된 경우 건너뜀 (버퍼 상한 방어)
    if (meas.stamp_sec - kf_stamp_sec > params_.max_reintegration_window_sec) {
      break;
    }
    // zero-accel 모드: accel 입력으로 현재 FGO-corrected b_a 사용
    // → eskf 내부에서 accel_meas - b_a = b_a - b_a = 0 으로 처리
    eskf.propagate(meas.gyro_radps, eskf.b_a(), meas.dt);
    reintegrated++;
  }

  last_correction_stamp_sec_ = kf_stamp_sec;
  last_reintegration_count_ = reintegrated;
  return reintegrated;
}

}  // namespace hybrid_localization
