#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "hybrid_localization/fgo/fgo_backend.hpp"
#include "hybrid_localization/fgo/imu_preintegration.hpp"
#include "hybrid_localization/fgo/keyframe_buffer.hpp"
#include "hybrid_localization/types.hpp"

using namespace hybrid_localization;

// ---------------------------------------------------------------------------
// 테스트 픽스처
// ---------------------------------------------------------------------------
class FgoBackendTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ImuPreintegration::Params imu_params;
    imu_params.gyro_noise_std = 0.01;
    imu_params.accel_noise_std = 0.1;
    imu_params.gyro_bias_rw_std = 1e-4;
    imu_params.accel_bias_rw_std = 1e-3;
    imu_params.gravity_map = Eigen::Vector3d(0.0, 0.0, -9.80665);

    FgoBackend::Params backend_params;
    backend_params.window_size = 5;
    backend_params.prior_pos_noise_m = 0.1;
    backend_params.prior_vel_noise_mps = 0.1;
    backend_params.nhc_lat_noise_mps = 0.1;
    backend_params.nhc_vert_noise_mps = 0.1;

    backend_.initialize(backend_params, imu_params);
  }

  // 단순 정지 키프레임 생성 (gtsam_preint = null)
  Keyframe make_static_keyframe(uint64_t idx, const Eigen::Vector3d & pos)
  {
    Keyframe kf;
    kf.index = idx;
    kf.stamp = rclcpp::Time(static_cast<int64_t>(idx * 1e8), RCL_ROS_TIME);
    kf.state.p_map = pos;
    kf.state.v_map = Eigen::Vector3d::Zero();
    kf.state.q_map_from_base = Eigen::Quaterniond::Identity();
    kf.state.b_g = Eigen::Vector3d::Zero();
    kf.state.b_a = Eigen::Vector3d::Zero();
    kf.gtsam_preint = nullptr;  // 첫 번째 키프레임처럼 처리
    return kf;
  }

  FgoBackend backend_;
};

// ---------------------------------------------------------------------------
// Test 1: 초기화 후 is_initialized() = true
// ---------------------------------------------------------------------------
TEST_F(FgoBackendTest, InitializationSucceeds)
{
  EXPECT_TRUE(backend_.is_initialized());
  EXPECT_FALSE(backend_.latest_result().valid);
}

// ---------------------------------------------------------------------------
// Test 2: 첫 번째 키프레임 업데이트 — prior 팩터만 추가
// ---------------------------------------------------------------------------
TEST_F(FgoBackendTest, FirstKeyframeUpdate)
{
  const Keyframe kf = make_static_keyframe(0, Eigen::Vector3d(0.0, 0.0, 0.0));
  const auto result = backend_.update(kf, std::nullopt);

  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.keyframe_index, 0u);
  // 초기값이 그대로 유지되어야 함 (prior 팩터만 추가됨)
  EXPECT_NEAR(result.state.p_map.x(), 0.0, 0.1);
  EXPECT_NEAR(result.state.p_map.y(), 0.0, 0.1);
  EXPECT_NEAR(result.state.p_map.z(), 0.0, 0.1);
}

// ---------------------------------------------------------------------------
// Test 3: GNSS 위치 측정값이 있는 업데이트
// ---------------------------------------------------------------------------
TEST_F(FgoBackendTest, UpdateWithGnssMeasurement)
{
  const Keyframe kf0 = make_static_keyframe(0, Eigen::Vector3d(0.0, 0.0, 0.0));
  backend_.update(kf0, std::nullopt);

  const Keyframe kf1 = make_static_keyframe(1, Eigen::Vector3d(1.0, 0.0, 0.0));

  GnssMeasurement gnss;
  gnss.stamp_sec = 0.1;
  gnss.pos_map = Eigen::Vector3d(1.0, 0.0, 0.0);
  gnss.pos_cov = Eigen::Matrix3d::Identity() * 0.01;
  gnss.status = 2;  // GBAS/RTK

  const auto result = backend_.update(kf1, gnss);

  // gtsam_preint=null 이므로 IMU 팩터 없이 업데이트 — 초기값 근처
  EXPECT_TRUE(result.valid);
}

// ---------------------------------------------------------------------------
// Test 4: GNSS 속도 측정값 포함
// ---------------------------------------------------------------------------
TEST_F(FgoBackendTest, UpdateWithGnssVelocity)
{
  const Keyframe kf0 = make_static_keyframe(0, Eigen::Vector3d(0.0, 0.0, 0.0));
  backend_.update(kf0, std::nullopt);

  const Keyframe kf1 = make_static_keyframe(1, Eigen::Vector3d(1.0, 0.0, 0.0));

  GnssMeasurement gnss;
  gnss.pos_map = Eigen::Vector3d(1.0, 0.0, 0.0);
  gnss.pos_cov = Eigen::Matrix3d::Identity() * 0.01;
  gnss.has_velocity = true;
  gnss.vel_map = Eigen::Vector3d(5.0, 0.0, 0.0);
  gnss.vel_cov = Eigen::Matrix3d::Identity() * 0.04;
  gnss.status = 2;

  const auto result = backend_.update(kf1, gnss);
  EXPECT_TRUE(result.valid);
}

// ---------------------------------------------------------------------------
// Test 5: GNSS 헤딩 측정값 포함
// ---------------------------------------------------------------------------
TEST_F(FgoBackendTest, UpdateWithGnssHeading)
{
  const Keyframe kf0 = make_static_keyframe(0, Eigen::Vector3d(0.0, 0.0, 0.0));
  backend_.update(kf0, std::nullopt);

  const Keyframe kf1 = make_static_keyframe(1, Eigen::Vector3d(1.0, 0.0, 0.0));

  GnssMeasurement gnss;
  gnss.pos_map = Eigen::Vector3d(1.0, 0.0, 0.0);
  gnss.pos_cov = Eigen::Matrix3d::Identity() * 0.01;
  gnss.has_heading = true;
  gnss.heading_rad = 0.0;  // East
  gnss.heading_var = 0.01;
  gnss.status = 2;

  const auto result = backend_.update(kf1, gnss);
  EXPECT_TRUE(result.valid);
}

// ---------------------------------------------------------------------------
// Test 6: 슬라이딩 윈도우 — window_size 초과 시 marginalize
// ---------------------------------------------------------------------------
TEST_F(FgoBackendTest, SlidingWindowMarginalization)
{
  // window_size = 5 이므로 7개 키프레임 추가 시 marginalize 발생
  for (uint64_t i = 0; i <= 6; ++i) {
    const Keyframe kf = make_static_keyframe(
      i, Eigen::Vector3d(static_cast<double>(i), 0.0, 0.0));
    const auto result = backend_.update(kf, std::nullopt);
    EXPECT_TRUE(result.valid) << "keyframe " << i << " failed";
  }
}

// ---------------------------------------------------------------------------
// Test 7: GNSS status 필터링 — min_status > status 시 위치 팩터 스킵
// ---------------------------------------------------------------------------
TEST_F(FgoBackendTest, GnssStatusFilteringSkipsBelowMinStatus)
{
  const Keyframe kf0 = make_static_keyframe(0, Eigen::Vector3d(0.0, 0.0, 0.0));
  backend_.update(kf0, std::nullopt);

  const Keyframe kf1 = make_static_keyframe(1, Eigen::Vector3d(1.0, 0.0, 0.0));

  GnssMeasurement gnss_bad;
  gnss_bad.pos_map = Eigen::Vector3d(999.0, 999.0, 999.0);  // 큰 잘못된 위치
  gnss_bad.pos_cov = Eigen::Matrix3d::Identity() * 0.01;
  gnss_bad.status = -1;  // no fix — gnss_min_status_for_pos=0 이므로 스킵

  const auto result = backend_.update(kf1, gnss_bad);
  // 스킵되어도 업데이트는 성공 (prior 팩터만으로 유효)
  EXPECT_TRUE(result.valid);
  // 잘못된 위치가 영향을 주지 않았어야 함
  EXPECT_LT(std::abs(result.state.p_map.x()), 10.0);
}

// ---------------------------------------------------------------------------
// Test 8: gtsam_preint_params() 접근자
// ---------------------------------------------------------------------------
TEST_F(FgoBackendTest, GtsamPreintParamsAccessor)
{
  const auto params = backend_.gtsam_preint_params();
  ASSERT_NE(params, nullptr);
  // 중력 크기 검증 (≈ 9.80665 m/s²)
  EXPECT_NEAR(params->n_gravity.norm(), 9.80665, 0.01);
}
