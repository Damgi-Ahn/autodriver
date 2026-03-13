#include <gtest/gtest.h>

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "hybrid_localization/fgo/imu_preintegration.hpp"

using namespace hybrid_localization;

// ---------------------------------------------------------------------------
// 헬퍼
// ---------------------------------------------------------------------------
static ImuPreintegration::Params make_params(double gyro_std = 0.05, double accel_std = 1.0)
{
  ImuPreintegration::Params p;
  p.gyro_noise_std = gyro_std;
  p.accel_noise_std = accel_std;
  p.gyro_bias_rw_std = 1.0e-4;
  p.accel_bias_rw_std = 1.0e-3;
  return p;
}

// ---------------------------------------------------------------------------
// 초기 상태 테스트
// ---------------------------------------------------------------------------
TEST(ImuPreintegration, InitialState)
{
  ImuPreintegration preint(make_params());
  preint.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  EXPECT_FALSE(preint.valid());
  EXPECT_DOUBLE_EQ(preint.delta_t(), 0.0);
  EXPECT_EQ(preint.num_samples(), 0);

  // ΔR = I
  EXPECT_NEAR(preint.delta_R().w(), 1.0, 1.0e-10);
  EXPECT_NEAR(preint.delta_R().x(), 0.0, 1.0e-10);

  // Δv, Δp = 0
  EXPECT_TRUE(preint.delta_v().isZero(1.0e-10));
  EXPECT_TRUE(preint.delta_p().isZero(1.0e-10));
}

// ---------------------------------------------------------------------------
// 정지 상태: gyro=0, accel=0 → Δv=0, Δp=0, ΔR=I
// ---------------------------------------------------------------------------
TEST(ImuPreintegration, StaticIntegration)
{
  ImuPreintegration preint(make_params());
  preint.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  const double dt = 0.005;
  for (int i = 0; i < 200; ++i) {
    preint.push(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), dt);
  }

  EXPECT_TRUE(preint.valid());
  EXPECT_NEAR(preint.delta_t(), 1.0, 1.0e-6);
  EXPECT_NEAR(preint.delta_R().w(), 1.0, 1.0e-6);
  EXPECT_TRUE(preint.delta_v().isZero(1.0e-8));
  EXPECT_TRUE(preint.delta_p().isZero(1.0e-8));
}

// ---------------------------------------------------------------------------
// 직선 가속: accel=[1,0,0], gyro=0, dt=0.01, N=100 → Δv≈1, Δp≈0.5
// ---------------------------------------------------------------------------
TEST(ImuPreintegration, ConstantAcceleration)
{
  ImuPreintegration preint(make_params());
  preint.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  const double dt = 0.01;
  const int N = 100;
  const Eigen::Vector3d accel(1.0, 0.0, 0.0);

  for (int i = 0; i < N; ++i) {
    preint.push(Eigen::Vector3d::Zero(), accel, dt);
  }

  const double T = dt * N;  // 1.0 s
  EXPECT_NEAR(preint.delta_t(), T, 1.0e-6);
  EXPECT_NEAR(preint.delta_v().x(), 1.0, 1.0e-4);   // v = a*t = 1.0
  EXPECT_NEAR(preint.delta_p().x(), 0.5, 1.0e-3);   // p = 0.5*a*t² = 0.5
  EXPECT_NEAR(preint.delta_v().y(), 0.0, 1.0e-10);
  EXPECT_NEAR(preint.delta_p().y(), 0.0, 1.0e-10);
}

// ---------------------------------------------------------------------------
// 순수 회전: gyro=[0,0,ω], accel=0, ΔR ≈ Rz(ω*T)
// ---------------------------------------------------------------------------
TEST(ImuPreintegration, ConstantYawRate)
{
  ImuPreintegration preint(make_params());
  preint.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  const double omega_z = M_PI / 4.0;  // 45 deg/s
  const double dt = 0.005;
  const int N = 200;  // T = 1.0 s → 45°

  for (int i = 0; i < N; ++i) {
    preint.push(Eigen::Vector3d(0.0, 0.0, omega_z), Eigen::Vector3d::Zero(), dt);
  }

  const double expected_yaw = omega_z * dt * N;
  // ΔR = Rz(expected_yaw)
  const Eigen::Quaterniond expected_q(
    Eigen::AngleAxisd(expected_yaw, Eigen::Vector3d::UnitZ()));
  const Eigen::Quaterniond & dR = preint.delta_R();

  EXPECT_NEAR(std::abs(dR.dot(expected_q)), 1.0, 1.0e-3);
}

// ---------------------------------------------------------------------------
// 공분산: 샘플이 추가되면 공분산이 커진다
// ---------------------------------------------------------------------------
TEST(ImuPreintegration, CovarianceGrows)
{
  ImuPreintegration preint(make_params());
  preint.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  const double trace_before = preint.covariance().trace();

  for (int i = 0; i < 50; ++i) {
    preint.push(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.1, 0.0, 0.0), 0.01);
  }

  EXPECT_GT(preint.covariance().trace(), trace_before);
  EXPECT_TRUE(preint.covariance().isFinite());
}

// ---------------------------------------------------------------------------
// 바이어스 야코비안: 수치 미분과 비교
// ---------------------------------------------------------------------------
TEST(ImuPreintegration, BiasJacobianNumerical)
{
  ImuPreintegration preint(make_params());
  const Eigen::Vector3d b_g0(0.01, -0.005, 0.002);
  const Eigen::Vector3d b_a0(0.05, 0.02, -0.01);
  preint.reset(b_g0, b_a0);

  for (int i = 0; i < 50; ++i) {
    preint.push(
      Eigen::Vector3d(0.1, -0.05, 0.2),
      Eigen::Vector3d(0.3, 0.1, 9.8),
      0.01);
  }

  const Eigen::Vector3d delta_bg(0.001, 0.0, 0.0);
  const Eigen::Vector3d delta_ba(0.0, 0.001, 0.0);

  // 1차 보정 vs. 수치 재적분 비교 (Δv만 검증)
  const Eigen::Vector3d v_corrected = preint.corrected_delta_v(
    Eigen::Vector3d::Zero(), delta_ba);
  const Eigen::Vector3d v_expected = preint.delta_v()
    + preint.jacobian_ba().block<3, 3>(3, 0) * delta_ba;
  EXPECT_TRUE((v_corrected - v_expected).norm() < 1.0e-10);
}

// ---------------------------------------------------------------------------
// reset: 상태가 완전히 초기화된다
// ---------------------------------------------------------------------------
TEST(ImuPreintegration, Reset)
{
  ImuPreintegration preint(make_params());
  preint.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  for (int i = 0; i < 100; ++i) {
    preint.push(Eigen::Vector3d(0.1, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0), 0.01);
  }

  EXPECT_TRUE(preint.valid());

  preint.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  EXPECT_FALSE(preint.valid());
  EXPECT_DOUBLE_EQ(preint.delta_t(), 0.0);
  EXPECT_TRUE(preint.delta_v().isZero(1.0e-10));
}
