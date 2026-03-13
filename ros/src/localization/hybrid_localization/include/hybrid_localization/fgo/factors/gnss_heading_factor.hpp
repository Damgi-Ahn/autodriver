#ifndef HYBRID_LOCALIZATION__FGO__FACTORS__GNSS_HEADING_FACTOR_HPP_
#define HYBRID_LOCALIZATION__FGO__FACTORS__GNSS_HEADING_FACTOR_HPP_

#include <cmath>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/numericalDerivative.h>

namespace hybrid_localization
{

// ---------------------------------------------------------------------------
// GnssHeadingFactor
//
// GNSS/GPHDT 기반 yaw(heading) 팩터.
// 관측: ENU yaw 각도 heading_meas [rad] (CCW from East)
// 변수: gtsam::Pose3 X(i)
//
// 잔차: r = angle_wrap(yaw(R_i) - heading_meas)  (scalar, 1D)
// 야코비안: 수치 미분 (키프레임 레이트에서 충분히 빠름)
// ---------------------------------------------------------------------------
class GnssHeadingFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
public:
  GnssHeadingFactor(
    gtsam::Key pose_key,
    double heading_meas_rad,
    const gtsam::SharedNoiseModel & noise)
  : gtsam::NoiseModelFactor1<gtsam::Pose3>(noise, pose_key),
    heading_meas_(heading_meas_rad)
  {}

  gtsam::Vector evaluateError(
    const gtsam::Pose3 & pose,
    boost::optional<gtsam::Matrix &> H = boost::none) const override
  {
    const auto error_fn = [this](const gtsam::Pose3 & p) -> gtsam::Vector1 {
      return error_scalar(p);
    };

    if (H) {
      *H = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
        error_fn, pose);
    }
    return error_scalar(pose);
  }

private:
  double heading_meas_;

  gtsam::Vector1 error_scalar(const gtsam::Pose3 & pose) const
  {
    const double yaw = pose.rotation().yaw();  // CCW from East in GTSAM ENU convention
    double diff = yaw - heading_meas_;
    // 각도 래핑: [-π, π]
    while (diff > M_PI) {diff -= 2.0 * M_PI;}
    while (diff < -M_PI) {diff += 2.0 * M_PI;}
    return gtsam::Vector1(diff);
  }
};

}  // namespace hybrid_localization

#endif  // HYBRID_LOCALIZATION__FGO__FACTORS__GNSS_HEADING_FACTOR_HPP_
