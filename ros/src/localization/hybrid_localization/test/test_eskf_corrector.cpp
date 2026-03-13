#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "hybrid_localization/eskf/eskf_core.hpp"
#include "hybrid_localization/fgo/eskf_corrector.hpp"
#include "hybrid_localization/fgo/fgo_backend.hpp"
#include "hybrid_localization/fgo/imu_buffer.hpp"
#include "hybrid_localization/types.hpp"

using namespace hybrid_localization;

// ---------------------------------------------------------------------------
// 공통 헬퍼
// ---------------------------------------------------------------------------
static EskfCore make_initialized_eskf()
{
  EskfCore eskf;
  eskf.initialize(Eigen::Vector3d(1.0, 2.0, 0.0), Eigen::Quaterniond::Identity());
  return eskf;
}

static FgoBackend::OptimizationResult make_valid_result(
  const Eigen::Vector3d & pos = Eigen::Vector3d(10.0, 20.0, 0.0))
{
  FgoBackend::OptimizationResult result;
  result.valid = true;
  result.keyframe_index = 1;
  result.state.p_map = pos;
  result.state.v_map = Eigen::Vector3d(1.0, 0.0, 0.0);
  result.state.q_map_from_base = Eigen::Quaterniond::Identity();
  result.state.b_g = Eigen::Vector3d(0.001, 0.0, 0.0);
  result.state.b_a = Eigen::Vector3d(0.01, 0.0, 0.0);
  return result;
}

// ---------------------------------------------------------------------------
// 테스트: 명목 상태 교체 (재적분 없음)
// ---------------------------------------------------------------------------
TEST(EskfCorrectorTest, StateReplacementWithEmptyBuffer)
{
  EskfCore eskf = make_initialized_eskf();
  EskfCorrector corrector;
  ImuBuffer buf;
  const auto result = make_valid_result(Eigen::Vector3d(10.0, 20.0, 0.0));

  const int n = corrector.apply(eskf, result, 1.0, buf);

  EXPECT_EQ(n, 0);
  EXPECT_NEAR(eskf.p_map().x(), 10.0, 1e-9);
  EXPECT_NEAR(eskf.p_map().y(), 20.0, 1e-9);
  EXPECT_NEAR(eskf.v_map().x(), 1.0, 1e-9);
  EXPECT_NEAR(eskf.b_g().x(), 0.001, 1e-9);
  EXPECT_NEAR(eskf.b_a().x(), 0.01, 1e-9);
}

// ---------------------------------------------------------------------------
// 테스트: 비활성화 시 상태 유지
// ---------------------------------------------------------------------------
TEST(EskfCorrectorTest, DisabledModeSkipsCorrection)
{
  EskfCore eskf = make_initialized_eskf();
  Eigen::Vector3d original_p = eskf.p_map();

  EskfCorrector::Params params;
  params.enabled = false;
  EskfCorrector corrector(params);
  ImuBuffer buf;
  const auto result = make_valid_result(Eigen::Vector3d(99.0, 99.0, 0.0));

  const int n = corrector.apply(eskf, result, 1.0, buf);

  EXPECT_EQ(n, 0);
  // 상태가 바뀌지 않았는지 확인
  EXPECT_NEAR(eskf.p_map().x(), original_p.x(), 1e-9);
}

// ---------------------------------------------------------------------------
// 테스트: invalid result 스킵
// ---------------------------------------------------------------------------
TEST(EskfCorrectorTest, InvalidResultSkipsCorrection)
{
  EskfCore eskf = make_initialized_eskf();
  Eigen::Vector3d original_p = eskf.p_map();

  EskfCorrector corrector;
  ImuBuffer buf;

  FgoBackend::OptimizationResult result;
  result.valid = false;

  const int n = corrector.apply(eskf, result, 1.0, buf);

  EXPECT_EQ(n, 0);
  EXPECT_NEAR(eskf.p_map().x(), original_p.x(), 1e-9);
}

// ---------------------------------------------------------------------------
// 테스트: ImuBuffer 에 keyframe 이후 샘플이 있을 때 재적분
// ---------------------------------------------------------------------------
TEST(EskfCorrectorTest, ReintegrationWithImuSamples)
{
  EskfCore eskf = make_initialized_eskf();
  EskfCorrector corrector;
  ImuBuffer buf;

  // keyframe stamp: 1.0s
  // IMU 샘플 3개를 1.01, 1.02, 1.03s 에 추가
  const double kf_stamp = 1.0;
  for (int i = 1; i <= 3; ++i) {
    ImuMeasurement meas;
    meas.stamp_sec = kf_stamp + i * 0.01;
    meas.gyro_radps = Eigen::Vector3d(0.0, 0.0, 0.1);  // 10 deg/s yaw
    meas.accel_mps2 = Eigen::Vector3d::Zero();
    meas.dt = 0.01;
    buf.push(meas);
  }

  const auto result = make_valid_result();
  const int n = corrector.apply(eskf, result, kf_stamp, buf);

  EXPECT_EQ(n, 3);
  EXPECT_EQ(corrector.last_reintegration_count(), 3);
  EXPECT_NEAR(corrector.last_correction_stamp_sec(), kf_stamp, 1e-9);
}

// ---------------------------------------------------------------------------
// 테스트: max_reintegration_window_sec 초과 샘플 무시
// ---------------------------------------------------------------------------
TEST(EskfCorrectorTest, ReintegrationWindowCap)
{
  EskfCore eskf = make_initialized_eskf();
  EskfCorrector::Params params;
  params.max_reintegration_window_sec = 0.05;  // 50ms 제한
  EskfCorrector corrector(params);
  ImuBuffer buf;

  const double kf_stamp = 1.0;
  // 3개 샘플: 1.01, 1.06, 1.11s
  // 1.06s (dt=0.06 > 0.05) 부터는 무시
  for (int i = 0; i < 3; ++i) {
    ImuMeasurement meas;
    meas.stamp_sec = kf_stamp + (i + 1) * 0.05;
    meas.gyro_radps = Eigen::Vector3d::Zero();
    meas.accel_mps2 = Eigen::Vector3d::Zero();
    meas.dt = 0.05;
    buf.push(meas);
  }

  const auto result = make_valid_result();
  const int n = corrector.apply(eskf, result, kf_stamp, buf);

  // 첫 번째 샘플 (1.05s, diff=0.05): 윈도우 내 (0.05 <= 0.05 는 경계 포함 아님)
  // stamp_sec - kf_stamp = 0.05 > 0.05 이므로 break → 0개 재적분
  EXPECT_EQ(n, 0);
}

// ---------------------------------------------------------------------------
// 테스트: keyframe 이전 샘플은 get_since 로 필터링
// ---------------------------------------------------------------------------
TEST(EskfCorrectorTest, OldSamplesNotReintegrated)
{
  EskfCore eskf = make_initialized_eskf();
  EskfCorrector corrector;
  ImuBuffer buf;

  const double kf_stamp = 2.0;
  // keyframe 이전 샘플 3개 (stamp < kf_stamp)
  for (int i = 0; i < 3; ++i) {
    ImuMeasurement meas;
    meas.stamp_sec = 1.0 + i * 0.1;
    meas.gyro_radps = Eigen::Vector3d(0.0, 0.0, 0.5);
    meas.accel_mps2 = Eigen::Vector3d::Zero();
    meas.dt = 0.1;
    buf.push(meas);
  }

  const auto result = make_valid_result();
  const int n = corrector.apply(eskf, result, kf_stamp, buf);

  EXPECT_EQ(n, 0);
}

// ---------------------------------------------------------------------------
// 테스트: last_correction_stamp_sec 초기값
// ---------------------------------------------------------------------------
TEST(EskfCorrectorTest, InitialCorrectionStampIsNegative)
{
  EskfCorrector corrector;
  EXPECT_LT(corrector.last_correction_stamp_sec(), 0.0);
  EXPECT_EQ(corrector.last_reintegration_count(), 0);
}

// ---------------------------------------------------------------------------
// 테스트: 자세 교체 후 재적분으로 자세가 바뀌는지 확인
// ---------------------------------------------------------------------------
TEST(EskfCorrectorTest, QuaternionRestoredAfterCorrection)
{
  EskfCore eskf = make_initialized_eskf();

  // FGO 결과: 45도 yaw
  const double yaw = M_PI / 4.0;
  FgoBackend::OptimizationResult result;
  result.valid = true;
  result.keyframe_index = 1;
  result.state.p_map = Eigen::Vector3d::Zero();
  result.state.v_map = Eigen::Vector3d::Zero();
  result.state.q_map_from_base =
    Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  result.state.b_g = Eigen::Vector3d::Zero();
  result.state.b_a = Eigen::Vector3d::Zero();

  EskfCorrector corrector;
  ImuBuffer buf;
  corrector.apply(eskf, result, 0.0, buf);

  const Eigen::Quaterniond q = eskf.q_map_from_base();
  const Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
  // euler[0] = yaw (ZYX 순서)
  EXPECT_NEAR(euler[0], yaw, 1e-6);
}
