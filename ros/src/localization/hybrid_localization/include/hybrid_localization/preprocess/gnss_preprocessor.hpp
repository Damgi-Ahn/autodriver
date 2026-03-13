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
  GnssPreprocessStatus preprocess(
    const sensor_msgs::msg::NavSatFix & msg,
    const MapProjector & projector,
    const ExtrinsicCache & extrinsics,
    double heading_yaw_rad,
    GnssPreprocessResult & out) const;
};

} // namespace hybrid_localization

#endif // HYBRID_LOCALIZATION__GNSS_PREPROCESSOR_HPP_
