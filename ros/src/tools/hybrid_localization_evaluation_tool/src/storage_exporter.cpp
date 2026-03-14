#include "hybrid_localization_evaluation_tool/storage_exporter.hpp"

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace autodriver::tools {

namespace {

std::string TimestampForFilename()
{
  const auto now = std::chrono::system_clock::now();
  const auto t   = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  gmtime_r(&t, &tm);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
  return oss.str();
}

template <typename T>
std::string OptToStr(const std::optional<T>& v, int precision = 6)
{
  if (!v.has_value()) return "";
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision) << *v;
  return oss.str();
}

std::string BoolToStr(bool has, bool val)
{
  if (!has) return "";
  return val ? "1" : "0";
}

}  // namespace

// ---------------------------------------------------------------------------
bool StorageExporter::Enable(const std::string& output_dir)
{
  if (output_dir.empty()) return false;

  std::error_code ec;
  std::filesystem::create_directories(output_dir, ec);
  if (ec) return false;

  const auto stamp  = TimestampForFilename();
  raw_path_         = output_dir + "/hybrid_eval_raw_"     + stamp + ".csv";
  kpi_path_         = output_dir + "/hybrid_eval_kpi_"     + stamp + ".csv";
  events_path_      = output_dir + "/hybrid_eval_events_"  + stamp + ".csv";
  session_path_     = output_dir + "/hybrid_eval_session_" + stamp + ".csv";

  raw_file_.open(raw_path_,     std::ios::out);
  kpi_file_.open(kpi_path_,     std::ios::out);
  events_file_.open(events_path_, std::ios::out);
  session_file_.open(session_path_, std::ios::out);

  if (!raw_file_.is_open() || !kpi_file_.is_open() ||
      !events_file_.is_open() || !session_file_.is_open()) {
    enabled_ = false;
    return false;
  }

  enabled_ = true;
  return true;
}

// ---------------------------------------------------------------------------
// Raw diagnostics CSV (full field set per spec)
// ---------------------------------------------------------------------------
void StorageExporter::WriteRawHeader()
{
  if (!enabled_ || raw_header_written_) return;
  raw_file_
    << "stamp_sec,is_activated,eskf_initialized,diag_level,"
    << "imu_count,gnss_count,gnss_vel_count,velocity_count,steering_count,"
    << "gnss_pos_update_applied,gnss_pos_update_reason,gnss_pos_nis,gnss_pos_residual_norm,"
    << "gnss_status,gnss_pos_R_xx,gnss_pos_R_yy,gnss_pos_R_zz,"
    << "gnss_pos_status_inflate,gnss_pos_nis_inflate,"
    << "gnss_vel_update_applied,gnss_vel_update_reason,gnss_vel_nis,gnss_vel_residual_norm,"
    << "gnss_vel_R_xx,gnss_vel_R_yy,gnss_vel_R_zz,"
    << "gnss_vel_status_inflate,gnss_vel_nis_inflate,"
    << "heading_yaw_update_applied,heading_yaw_update_reason,"
    << "heading_yaw_nis,heading_yaw_residual_rad,"
    << "heading_yaw_var,heading_yaw_var_eff,heading_yaw_var_applied,"
    << "heading_yaw_nis_inflate,heading_status_inflate,heading_recover_inflate,"
    << "heading_yaw_var_source,"
    << "vehicle_speed_var,vehicle_nhc_var,vehicle_zupt_var,vehicle_yaw_rate_var,"
    << "imu_gyro_noise_std,imu_accel_noise_std,imu_gyro_bias_noise_std,imu_accel_bias_noise_std,"
    << "P_trace,P_max_diag,P_min_diag,P_min_eig,"
    << "P_pos_max_diag,P_vel_max_diag,P_att_max_diag,P_bg_max_diag,P_ba_max_diag,"
    << "imu_dt_min_ms,imu_dt_mean_ms,imu_dt_max_ms,"
    << "gnss_delay_ms,gnss_vel_delay_ms,velocity_delay_ms,steering_delay_ms,"
    << "fgo_keyframe_count,fgo_correction_count\n";
  raw_header_written_ = true;
}

void StorageExporter::WriteRawSample(const DiagSample& s)
{
  if (!enabled_) return;
  WriteRawHeader();

  raw_file_ << std::fixed << std::setprecision(6)
    << s.stamp.seconds()            << ","
    << (s.is_activated ? "1" : "0") << ","
    << (s.eskf_initialized ? "1":"0") << ","
    << static_cast<int>(s.diag_level) << ","
    << OptToStr(s.imu_count)        << ","
    << OptToStr(s.gnss_count)       << ","
    << OptToStr(s.gnss_vel_count)   << ","
    << OptToStr(s.velocity_count)   << ","
    << OptToStr(s.steering_count)   << ","
    << BoolToStr(s.has_gnss_pos_update_applied, s.gnss_pos_update_applied) << ","
    << s.gnss_pos_update_reason     << ","
    << OptToStr(s.gnss_pos_nis)     << ","
    << OptToStr(s.gnss_pos_residual_norm) << ","
    << OptToStr(s.gnss_status, 0)   << ","
    << OptToStr(s.gnss_pos_R_xx)    << ","
    << OptToStr(s.gnss_pos_R_yy)    << ","
    << OptToStr(s.gnss_pos_R_zz)    << ","
    << OptToStr(s.gnss_pos_status_inflate) << ","
    << OptToStr(s.gnss_pos_nis_inflate) << ","
    << BoolToStr(s.has_gnss_vel_update_applied, s.gnss_vel_update_applied) << ","
    << s.gnss_vel_update_reason     << ","
    << OptToStr(s.gnss_vel_nis)     << ","
    << OptToStr(s.gnss_vel_residual_norm) << ","
    << OptToStr(s.gnss_vel_R_xx)    << ","
    << OptToStr(s.gnss_vel_R_yy)    << ","
    << OptToStr(s.gnss_vel_R_zz)    << ","
    << OptToStr(s.gnss_vel_status_inflate) << ","
    << OptToStr(s.gnss_vel_nis_inflate) << ","
    << BoolToStr(s.has_heading_yaw_update_applied, s.heading_yaw_update_applied) << ","
    << s.heading_yaw_update_reason  << ","
    << OptToStr(s.heading_yaw_nis)  << ","
    << OptToStr(s.heading_yaw_residual_rad) << ","
    << OptToStr(s.heading_yaw_var)  << ","
    << OptToStr(s.heading_yaw_var_eff) << ","
    << OptToStr(s.heading_yaw_var_applied) << ","
    << OptToStr(s.heading_yaw_nis_inflate) << ","
    << OptToStr(s.heading_status_inflate) << ","
    << OptToStr(s.heading_recover_inflate) << ","
    << s.heading_yaw_var_source     << ","
    << OptToStr(s.vehicle_speed_var) << ","
    << OptToStr(s.vehicle_nhc_var)  << ","
    << OptToStr(s.vehicle_zupt_var) << ","
    << OptToStr(s.vehicle_yaw_rate_var) << ","
    << OptToStr(s.imu_gyro_noise_std) << ","
    << OptToStr(s.imu_accel_noise_std) << ","
    << OptToStr(s.imu_gyro_bias_noise_std) << ","
    << OptToStr(s.imu_accel_bias_noise_std) << ","
    << OptToStr(s.P_trace)          << ","
    << OptToStr(s.P_max_diag)       << ","
    << OptToStr(s.P_min_diag)       << ","
    << OptToStr(s.P_min_eig)        << ","
    << OptToStr(s.P_pos_max_diag)   << ","
    << OptToStr(s.P_vel_max_diag)   << ","
    << OptToStr(s.P_att_max_diag)   << ","
    << OptToStr(s.P_bg_max_diag)    << ","
    << OptToStr(s.P_ba_max_diag)    << ","
    << OptToStr(s.imu_dt_min_ms)    << ","
    << OptToStr(s.imu_dt_mean_ms)   << ","
    << OptToStr(s.imu_dt_max_ms)    << ","
    << OptToStr(s.gnss_delay)       << ","
    << OptToStr(s.gnss_vel_delay)   << ","
    << OptToStr(s.velocity_delay)   << ","
    << OptToStr(s.steering_delay)   << ","
    << OptToStr(s.fgo_keyframe_count, 0) << ","
    << OptToStr(s.fgo_correction_count, 0) << "\n";
}

// ---------------------------------------------------------------------------
// KPI snapshot CSV
// ---------------------------------------------------------------------------
void StorageExporter::WriteKpiHeader()
{
  if (!enabled_ || kpi_header_written_) return;
  kpi_file_
    << "stamp_sec,window_sec,"
    << "output_count,output_expected,output_ratio,output_last_age_sec,"
    << "diag_rate_hz,output_rate_hz,"
    << "pos_applied_ratio,pos_applied_count,pos_total,"
    << "vel_applied_ratio,vel_applied_count,vel_total,"
    << "heading_applied_ratio,heading_applied_count,heading_total,"
    << "pos_nis_mean,pos_nis_p95,pos_nis_max,pos_nis_count,"
    << "vel_nis_mean,vel_nis_p95,vel_nis_max,vel_nis_count,"
    << "heading_nis_mean,heading_nis_p95,heading_nis_max,heading_nis_count,"
    << "pos_nis_violation_rate,vel_nis_violation_rate,heading_nis_violation_rate,"
    << "gnss_delay_mean,gnss_delay_p95,gnss_delay_max,gnss_delay_count,"
    << "gnss_vel_delay_mean,gnss_vel_delay_p95,gnss_vel_delay_max,gnss_vel_delay_count,"
    << "velocity_delay_mean,velocity_delay_p95,velocity_delay_max,velocity_delay_count,"
    << "steering_delay_mean,steering_delay_p95,steering_delay_max,steering_delay_count,"
    << "P_trace_latest,P_min_eig_latest,fgo_keyframe_count,fgo_correction_count\n";
  kpi_header_written_ = true;
}

void StorageExporter::WriteKpiSnapshot(const KpiSnapshot& k)
{
  if (!enabled_) return;
  WriteKpiHeader();

  auto ws = [this](const StatSummary& s) {
    kpi_file_ << s.mean << "," << s.p95 << "," << s.max << "," << s.count << ",";
  };

  kpi_file_ << std::fixed << std::setprecision(6)
    << k.stamp.seconds() << "," << k.window_sec << ","
    << k.output_availability.output_count << ","
    << k.output_availability.expected_count << ","
    << k.output_availability.ratio << ","
    << (k.output_availability.has_output ? std::to_string(k.output_availability.last_age_sec) : "") << ","
    << k.diag_rate_hz << "," << k.output_rate_hz << ","
    << k.gnss_pos_update.ratio << "," << k.gnss_pos_update.applied << "," << k.gnss_pos_update.total << ","
    << k.gnss_vel_update.ratio << "," << k.gnss_vel_update.applied << "," << k.gnss_vel_update.total << ","
    << k.heading_yaw_update.ratio << "," << k.heading_yaw_update.applied << "," << k.heading_yaw_update.total << ",";
  ws(k.gnss_pos_nis);
  ws(k.gnss_vel_nis);
  ws(k.heading_yaw_nis);
  kpi_file_
    << k.gnss_pos_nis_violation_rate << ","
    << k.gnss_vel_nis_violation_rate << ","
    << k.heading_nis_violation_rate  << ",";
  ws(k.gnss_delay);
  ws(k.gnss_vel_delay);
  ws(k.velocity_delay);
  kpi_file_
    << k.steering_delay.mean << "," << k.steering_delay.p95 << ","
    << k.steering_delay.max  << "," << k.steering_delay.count << ","
    << OptToStr(k.P_trace_latest)   << ","
    << OptToStr(k.P_min_eig_latest) << ","
    << (k.fgo_keyframe_count_latest.has_value()   ? std::to_string(*k.fgo_keyframe_count_latest)   : "") << ","
    << (k.fgo_correction_count_latest.has_value() ? std::to_string(*k.fgo_correction_count_latest) : "") << "\n";
}

// ---------------------------------------------------------------------------
// Events CSV
// ---------------------------------------------------------------------------
void StorageExporter::WriteEventsHeader()
{
  if (!enabled_ || events_header_written_) return;
  events_file_ << "stamp_sec,severity,event_type,message,key,value\n";
  events_header_written_ = true;
}

void StorageExporter::WriteEvents(const std::vector<AlertEvent>& events)
{
  if (!enabled_ || events.empty()) return;
  WriteEventsHeader();
  for (const auto& e : events) {
    events_file_ << std::fixed << std::setprecision(6)
      << e.stamp.seconds()          << ","
      << AlertSeverityStr(e.severity) << ","
      << AlertTypeStr(e.type)       << ","
      << "\"" << e.message << "\","
      << e.key                      << ","
      << e.value                    << "\n";
  }
}

// ---------------------------------------------------------------------------
// Session summary CSV
// ---------------------------------------------------------------------------
void StorageExporter::WriteSessionHeader()
{
  if (!enabled_ || session_header_written_) return;
  session_file_
    << "start_time_sec,end_time_sec,duration_sec,overall_status,"
    << "alerts_total,alerts_warn,alerts_error,"
    << "availability_mean,output_rate_mean,"
    << "nis_violation_rate_pos,nis_violation_rate_vel,nis_violation_rate_heading,"
    << "P_trace_latest,P_min_eig_latest\n";
  session_header_written_ = true;
}

void StorageExporter::WriteSessionSummary(
    const rclcpp::Time& start,
    const rclcpp::Time& end,
    const KpiSnapshot& final_kpi,
    size_t total_alerts,
    size_t warn_alerts,
    size_t error_alerts)
{
  if (!enabled_) return;
  WriteSessionHeader();

  const double duration = (end - start).seconds();
  const char* status =
      (error_alerts > 0) ? "ERROR" : (warn_alerts > 0) ? "WARN" : "OK";

  session_file_ << std::fixed << std::setprecision(6)
    << start.seconds()  << ","
    << end.seconds()    << ","
    << duration         << ","
    << status           << ","
    << total_alerts     << ","
    << warn_alerts      << ","
    << error_alerts     << ","
    << final_kpi.output_availability.ratio << ","
    << final_kpi.output_rate_hz            << ","
    << final_kpi.gnss_pos_nis_violation_rate << ","
    << final_kpi.gnss_vel_nis_violation_rate << ","
    << final_kpi.heading_nis_violation_rate  << ","
    << OptToStr(final_kpi.P_trace_latest)   << ","
    << OptToStr(final_kpi.P_min_eig_latest) << "\n";
}

}  // namespace autodriver::tools
