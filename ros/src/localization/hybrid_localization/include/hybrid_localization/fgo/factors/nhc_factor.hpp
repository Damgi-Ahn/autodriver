#ifndef HYBRID_LOCALIZATION__FGO__FACTORS__NHC_FACTOR_HPP_
#define HYBRID_LOCALIZATION__FGO__FACTORS__NHC_FACTOR_HPP_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/numericalDerivative.h>

namespace hybrid_localization
{

// ---------------------------------------------------------------------------
// NhcFactor (Non-Holonomic Constraint Factor)
//
// 차량 구조적 제약: body 프레임 기준 횡방향(y)·수직방향(z) 속도 ≈ 0.
//
// 관측: 없음 (제약 자체가 측정값)
// 변수: gtsam::Pose3 X(i), gtsam::Vector3 V(i)
//
// 잔차: r = [R_i^T * v_i]_{y,z}  (2D)
//   r[0] = lateral  velocity in body frame
//   r[1] = vertical velocity in body frame
//
// 야코비안:
//   H_pose (2×6): 수치 미분
//   H_vel  (2×3): R_i^T 의 1·2번째 행 (rows 1,2 of R^T)
// ---------------------------------------------------------------------------
class NhcFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3>
{
public:
  NhcFactor(
    gtsam::Key pose_key,
    gtsam::Key vel_key,
    const gtsam::SharedNoiseModel & noise)
  : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3>(noise, pose_key, vel_key)
  {}

  gtsam::Vector evaluateError(
    const gtsam::Pose3 & pose,
    const gtsam::Vector3 & vel,
    boost::optional<gtsam::Matrix &> H1 = boost::none,
    boost::optional<gtsam::Matrix &> H2 = boost::none) const override
  {
    const gtsam::Matrix3 R = pose.rotation().matrix();
    // 잔차: body 프레임 속도의 y, z 성분
    const gtsam::Vector3 v_body = R.transpose() * vel;
    const gtsam::Vector2 residual(v_body(1), v_body(2));

    if (H1) {
      // 수치 미분 (Pose3 야코비안은 복잡)
      auto fn = [&vel](const gtsam::Pose3 & p) -> gtsam::Vector2 {
        const gtsam::Matrix3 Ri = p.rotation().matrix();
        const gtsam::Vector3 vb = Ri.transpose() * vel;
        return gtsam::Vector2(vb(1), vb(2));
      };
      *H1 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Pose3>(fn, pose);
    }
    if (H2) {
      // 해석적 야코비안: d(R^T v)/dv = R^T, 행 1,2만 취함
      *H2 = R.transpose().block<2, 3>(1, 0);
    }
    return residual;
  }
};

}  // namespace hybrid_localization

#endif  // HYBRID_LOCALIZATION__FGO__FACTORS__NHC_FACTOR_HPP_
