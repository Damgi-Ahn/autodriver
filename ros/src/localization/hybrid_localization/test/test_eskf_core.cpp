#include <gtest/gtest.h>
#include <cmath>
#include "hybrid_localization/eskf/eskf_core.hpp"

using hybrid_localization::EskfCore;
using hybrid_localization::EskfCoreParams;

namespace
{

constexpr double kGravity = 9.80665;

EskfCore make_initialized_eskf()
{
  EskfCoreParams params;
  EskfCore eskf(params);
  eskf.initialize(
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Quaterniond::Identity());
  return eskf;
}

}  // namespace

TEST(EskfCore, InitializedState)
{
  auto eskf = make_initialized_eskf();
  EXPECT_TRUE(eskf.initialized());
  EXPECT_TRUE(eskf.finite());
  EXPECT_NEAR(eskf.p_map().norm(), 0.0, 1e-10);
  EXPECT_NEAR(eskf.v_map().norm(), 0.0, 1e-10);
  EXPECT_NEAR(eskf.q_map_from_base().angularDistance(Eigen::Quaterniond::Identity()), 0.0, 1e-10);
}

TEST(EskfCore, PropagateWithZeroInput)
{
  auto eskf = make_initialized_eskf();
  const double dt = 0.005;  // 200 Hz
  // 입력 없음: 이상적 조건에서 상태 변화 없어야 함
  eskf.propagate(Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, kGravity), dt);
  EXPECT_TRUE(eskf.finite());
  EXPECT_NEAR(eskf.p_map().norm(), 0.0, 1e-6);
  EXPECT_NEAR(eskf.v_map().norm(), 0.0, 1e-6);
}

TEST(EskfCore, PropagateCovariance)
{
  auto eskf = make_initialized_eskf();
  const auto P_before = eskf.P();
  // propagate: 공분산이 증가해야 함 (프로세스 노이즈)
  for (int i = 0; i < 10; ++i) {
    eskf.propagate(Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, kGravity), 0.005);
  }
  const auto P_after = eskf.P();
  EXPECT_GT(P_after(0, 0), P_before(0, 0));  // 위치 분산 증가
  EXPECT_GT(P_after(3, 3), P_before(3, 3));  // 속도 분산 증가
}

TEST(EskfCore, GnssPositionUpdate)
{
  auto eskf = make_initialized_eskf();
  // 여러 번 propagate 후 GNSS 위치 업데이트
  for (int i = 0; i < 50; ++i) {
    eskf.propagate(Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, kGravity), 0.005);
  }
  const Eigen::Vector3d z_p(1.0, 2.0, 0.0);
  const Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 0.25;
  const auto dbg = eskf.update_gnss_position_3d(z_p, R);
  EXPECT_TRUE(dbg.applied);
  EXPECT_TRUE(eskf.finite());
  // 위치가 측정치 방향으로 이동해야 함
  EXPECT_GT(eskf.p_map().x(), 0.0);
  EXPECT_GT(eskf.p_map().y(), 0.0);
}

TEST(EskfCore, Reset)
{
  auto eskf = make_initialized_eskf();
  eskf.propagate(Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, kGravity), 0.005);
  eskf.reset();
  EXPECT_FALSE(eskf.initialized());
  EXPECT_NEAR(eskf.p_map().norm(), 0.0, 1e-10);
}

TEST(EskfCore, NisGatingSkip)
{
  EskfCoreParams params;
  params.nis_gate_inflate = false;  // inflate 대신 skip
  EskfCore eskf(params);
  eskf.initialize(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  for (int i = 0; i < 20; ++i) {
    eskf.propagate(Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, kGravity), 0.005);
  }
  // 매우 작은 측정 노이즈 + 큰 잔차 → NIS gate skip 발동
  const Eigen::Vector3d z_p(100.0, 0.0, 0.0);
  const Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 1e-6;
  const auto dbg = eskf.update_gnss_position_3d(z_p, R);
  EXPECT_FALSE(dbg.applied);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
