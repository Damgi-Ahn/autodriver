#ifndef HYBRID_LOCALIZATION__GNSS_PREPROCESSOR_HPP_
#define HYBRID_LOCALIZATION__GNSS_PREPROCESSOR_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <string>

#include "hybrid_localization/util/map_projector.hpp"
#include "hybrid_localization/types.hpp"

namespace hybrid_localization
{

enum class GnssPreprocessStatus
{
  kOk = 0,
  kNoMapProjector,
  kNoFix,
  kInvalidLatLon,
  kProjectionFailed,
  kNonFiniteProjection,
};

struct GnssPreprocessResult
{
  geometry_msgs::msg::Point antenna_position_map{};
  geometry_msgs::msg::Point base_position_map{};
  bool altitude_sanitized{false};
  std::string projection_error{};
};

class GnssPreprocessor
{
public:
  // 전체 전처리 파이프라인: validate → project → lever-arm
  GnssPreprocessStatus preprocess(
    const sensor_msgs::msg::NavSatFix & msg,
    const MapProjector & projector,
    const ExtrinsicCache & extrinsics,
    double heading_yaw_rad,
    GnssPreprocessResult & out) const;

private:
  // ---- Pipeline steps -------------------------------------------------------

  // Step 1: fix status 및 lat/lon 유효성 검증
  static GnssPreprocessStatus validate_fix(
    const sensor_msgs::msg::NavSatFix & msg);

  // Step 2: 비정상 고도값 처리 (sentinel 값 클램프)
  static double sanitize_altitude(
    double raw_alt,
    const MapProjector & projector,
    bool & was_sanitized_out);

  // Step 3: lat/lon/alt → map 좌표 투영
  static GnssPreprocessStatus project_to_map(
    double lat, double lon, double alt,
    const MapProjector & projector,
    geometry_msgs::msg::Point & antenna_out,
    std::string & error_out);

  // Step 4: GNSS 안테나 위치 → base_link 위치 (lever-arm 보정)
  static void apply_lever_arm(
    const geometry_msgs::msg::Point & antenna_map,
    const ExtrinsicCache & extrinsics,
    double heading_yaw_rad,
    geometry_msgs::msg::Point & base_out);
};

}  // namespace hybrid_localization

#endif  // HYBRID_LOCALIZATION__GNSS_PREPROCESSOR_HPP_
