#ifndef HYBRID_LOCALIZATION__GNSS_HEADING_ARBITRATOR_HPP_
#define HYBRID_LOCALIZATION__GNSS_HEADING_ARBITRATOR_HPP_

#include <cmath>
#include <limits>

namespace hybrid_localization
{

enum class GnssHeadingSource
{
  kNone = 0,
  kGphdt = 1,
};

struct GnssHeadingArbitratorParams
{
  // `heading.min_gnss_status_for_yaw_update`의 내부 반영값.
  // - 이 status 미만이면 GPHDT yaw 후보를 사용하지 않는다.
  int min_status_for_gphdt{1};

  // When GNSS status transitions from degraded to good (e.g., 0->2), GPHDT can
  // keep outputting invalid values for a short period. Hold-off avoids using
  // those samples.
  // `heading.arb.gphdt_recover_holdoff_sec` [s]
  // - ↑: status 회복 직후 더 오래 GPHDT를 막음(안정성↑), 초기 yaw 보정 지연↑
  // - ↓: 더 빨리 사용(수렴 빠름), 회복 직후 쓰레기값 영향↑ 가능
  double gphdt_recover_holdoff_sec{2.5};

  // Max allowed time difference between "decision time" and candidate
  // measurement stamp when selecting yaw (used for init/lever-arm and to avoid
  // choosing stale candidates).
  double max_time_diff_sec{0.5};
};

struct GnssYawMeasurement
{
  GnssHeadingSource source{GnssHeadingSource::kNone};
  double yaw_rad{std::numeric_limits<double>::quiet_NaN()};
  double yaw_var_rad2{std::numeric_limits<double>::quiet_NaN()};
  double stamp_sec{std::numeric_limits<double>::quiet_NaN()};
  const char * reason{nullptr};
};

class GnssHeadingArbitrator
{
public:
  explicit GnssHeadingArbitrator(
    const GnssHeadingArbitratorParams & t_params = GnssHeadingArbitratorParams{})
  : params_(t_params) {}

  void set_params(const GnssHeadingArbitratorParams & t_params)
  {
    params_ = t_params;
  }
  const GnssHeadingArbitratorParams & params() const {return params_;}

  void reset();

  void update_gnss_status(int t_status, double t_stamp_sec);
  void update_gphdt(
    double t_yaw_rad, double t_yaw_var_rad2,
    double t_stamp_sec);

  // Check if a given GPHDT sample would be usable *now* under the current GNSS
  // status/holdoff/freshness policies, without mutating internal state.
  bool gphdt_sample_usable(
    double t_yaw_rad, double t_yaw_var_rad2,
    double t_stamp_sec, double t_now_sec,
    const char ** t_reason) const;

  bool gphdt_usable(double t_now_sec, const char ** t_reason) const;

  GnssYawMeasurement select(double t_now_sec) const;

private:
  static bool is_finite(const double x) {return std::isfinite(x);}

  bool is_fresh(double t_now_sec, double t_stamp_sec) const;

  GnssHeadingArbitratorParams params_{};

  int latest_gnss_status_{std::numeric_limits<int>::min()};
  double latest_gnss_status_stamp_sec_{std::numeric_limits<double>::quiet_NaN()};
  double gphdt_holdoff_until_sec_{0.0};

  bool have_gphdt_{false};
  double gphdt_yaw_rad_{std::numeric_limits<double>::quiet_NaN()};
  double gphdt_yaw_var_rad2_{std::numeric_limits<double>::quiet_NaN()};
  double gphdt_stamp_sec_{std::numeric_limits<double>::quiet_NaN()};

};

} // namespace hybrid_localization

#endif // HYBRID_LOCALIZATION__GNSS_HEADING_ARBITRATOR_HPP_
