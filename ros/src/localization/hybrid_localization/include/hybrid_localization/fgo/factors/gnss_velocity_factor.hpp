#ifndef HYBRID_LOCALIZATION__FGO__FACTORS__GNSS_VELOCITY_FACTOR_HPP_
#define HYBRID_LOCALIZATION__FGO__FACTORS__GNSS_VELOCITY_FACTOR_HPP_

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>

namespace hybrid_localization
{

// ---------------------------------------------------------------------------
// GnssVelocityFactor
//
// GNSS 도플러 기반 속도 팩터.
// 관측: map 프레임 기준 3D 속도 v_meas
// 변수: gtsam::Vector3 V(i) — map 프레임 속도
//
// 잔차: r = V - v_meas  (3D)
// 야코비안: H = I_3x3
// ---------------------------------------------------------------------------
class GnssVelocityFactor : public gtsam::NoiseModelFactor1<gtsam::Vector3>
{
public:
  GnssVelocityFactor(
    gtsam::Key vel_key,
    const gtsam::Vector3 & measured_vel,
    const gtsam::SharedNoiseModel & noise)
  : gtsam::NoiseModelFactor1<gtsam::Vector3>(noise, vel_key),
    measured_vel_(measured_vel)
  {}

  gtsam::Vector evaluateError(
    const gtsam::Vector3 & vel,
    boost::optional<gtsam::Matrix &> H = boost::none) const override
  {
    if (H) {
      *H = gtsam::I_3x3;
    }
    return vel - measured_vel_;
  }

private:
  gtsam::Vector3 measured_vel_;
};

}  // namespace hybrid_localization

#endif  // HYBRID_LOCALIZATION__FGO__FACTORS__GNSS_VELOCITY_FACTOR_HPP_
