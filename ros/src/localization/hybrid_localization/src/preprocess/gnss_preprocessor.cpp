#include "hybrid_localization/preprocess/gnss_preprocessor.hpp"

#include <sensor_msgs/msg/nav_sat_status.hpp>

#include <cmath>

namespace hybrid_localization
{

// ---------------------------------------------------------------------------
// Pipeline step implementations
// ---------------------------------------------------------------------------

GnssPreprocessStatus GnssPreprocessor::validate_fix(
  const sensor_msgs::msg::NavSatFix & msg)
{
  if (msg.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
    return GnssPreprocessStatus::kNoFix;
  }

  if (!std::isfinite(msg.latitude) || !std::isfinite(msg.longitude) ||
    std::abs(msg.latitude) > 90.0 || std::abs(msg.longitude) > 180.0)
  {
    return GnssPreprocessStatus::kInvalidLatLon;
  }

  return GnssPreprocessStatus::kOk;
}

double GnssPreprocessor::sanitize_altitude(
  double raw_alt,
  const MapProjector & projector,
  bool & was_sanitized_out)
{
  // 일부 GNSS 드라이버는 신호 미수신 시 sentinel 값(예: -2e10) 출력
  if (!std::isfinite(raw_alt) || std::abs(raw_alt) > 1.0e6) {
    was_sanitized_out = true;
    return projector.info().map_origin.altitude;
  }
  was_sanitized_out = false;
  return raw_alt;
}

GnssPreprocessStatus GnssPreprocessor::project_to_map(
  double lat, double lon, double alt,
  const MapProjector & projector,
  geometry_msgs::msg::Point & antenna_out,
  std::string & error_out)
{
  const auto proj_status = projector.project_forward(lat, lon, alt, antenna_out, &error_out);

  // kFallbackUsed: local ENU 근사로 투영 — 허용 가능
  if (proj_status != ProjectionStatus::kOk &&
    proj_status != ProjectionStatus::kFallbackUsed)
  {
    return GnssPreprocessStatus::kProjectionFailed;
  }

  if (!std::isfinite(antenna_out.x) || !std::isfinite(antenna_out.y) ||
    !std::isfinite(antenna_out.z))
  {
    return GnssPreprocessStatus::kNonFiniteProjection;
  }

  return GnssPreprocessStatus::kOk;
}

void GnssPreprocessor::apply_lever_arm(
  const geometry_msgs::msg::Point & antenna_map,
  const ExtrinsicCache & extrinsics,
  double heading_yaw_rad,
  geometry_msgs::msg::Point & base_out)
{
  base_out = antenna_map;

  if (!extrinsics.gnss_valid) {
    return;
  }

  // base_link → antenna offset (base_link 프레임)
  const double dx = extrinsics.base_to_gnss.transform.translation.x;
  const double dy = extrinsics.base_to_gnss.transform.translation.y;
  const double dz = extrinsics.base_to_gnss.transform.translation.z;

  // heading yaw 기반 2D 회전: p_base = p_antenna - R(yaw) * t_base_to_antenna
  const double cos_yaw = std::cos(heading_yaw_rad);
  const double sin_yaw = std::sin(heading_yaw_rad);

  base_out.x = antenna_map.x - (cos_yaw * dx - sin_yaw * dy);
  base_out.y = antenna_map.y - (sin_yaw * dx + cos_yaw * dy);
  base_out.z = antenna_map.z - dz;
}

// ---------------------------------------------------------------------------
// Public preprocess() — pipeline steps를 순서대로 호출
// ---------------------------------------------------------------------------
GnssPreprocessStatus GnssPreprocessor::preprocess(
  const sensor_msgs::msg::NavSatFix & msg,
  const MapProjector & projector,
  const ExtrinsicCache & extrinsics,
  double heading_yaw_rad,
  GnssPreprocessResult & out) const
{
  out = GnssPreprocessResult{};

  if (!projector.valid()) {
    return GnssPreprocessStatus::kNoMapProjector;
  }

  // Step 1: fix/lat/lon 검증
  const auto step1_status = validate_fix(msg);
  if (step1_status != GnssPreprocessStatus::kOk) {
    return step1_status;
  }

  // Step 2: 고도 정제
  const double alt = sanitize_altitude(msg.altitude, projector, out.altitude_sanitized);

  // Step 3: map 좌표 투영
  const auto step3_status = project_to_map(
    msg.latitude, msg.longitude, alt,
    projector, out.antenna_position_map, out.projection_error);
  if (step3_status != GnssPreprocessStatus::kOk) {
    return step3_status;
  }

  // Step 4: lever-arm 보정 → base_link 위치
  apply_lever_arm(
    out.antenna_position_map, extrinsics, heading_yaw_rad, out.base_position_map);

  return GnssPreprocessStatus::kOk;
}

}  // namespace hybrid_localization
