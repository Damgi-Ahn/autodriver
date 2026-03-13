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
  const auto t = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  gmtime_r(&t, &tm);

  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
  return oss.str();
}

std::string OptionalToString(const std::optional<double>& value)
{
  if (!value.has_value()) return "";
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6) << value.value();
  return oss.str();
}

std::string OptionalBoolString(bool has_value, bool value)
{
  if (!has_value) return "";
  return value ? "1" : "0";
}

}  // namespace

bool StorageExporter::Enable(const std::string& output_dir)
{
  if (output_dir.empty()) return false;

  std::error_code ec;
  std::filesystem::create_directories(output_dir, ec);
  if (ec) return false;

  const auto stamp = TimestampForFilename();
  raw_path_ = output_dir + "/hybrid_eval_raw_" + stamp + ".csv";
  kpi_path_ = output_dir + "/hybrid_eval_kpi_" + stamp + ".csv";

  raw_file_.open(raw_path_, std::ios::out);
  kpi_file_.open(kpi_path_, std::ios::out);

  if (!raw_file_.is_open() || !kpi_file_.is_open()) {
    enabled_ = false;
    return false;
  }

  enabled_ = true;
  return true;
}

void StorageExporter::WriteRawHeader()
{
  if (!enabled_ || raw_header_written_) return;
  raw_file_
      << "stamp_sec,is_activated,eskf_initialized,"
      << "gnss_pos_update_applied,gnss_pos_update_reason,gnss_pos_nis,"
      << "gnss_vel_update_applied,gnss_vel_update_reason,gnss_vel_nis,"
      << "heading_yaw_update_applied,heading_yaw_update_reason,heading_yaw_nis,"
      << "gnss_delay,gnss_vel_delay,velocity_delay,steering_delay\n";
  raw_header_written_ = true;
}

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
      << "gnss_delay_mean,gnss_delay_p95,gnss_delay_max,gnss_delay_count,"
      << "gnss_vel_delay_mean,gnss_vel_delay_p95,gnss_vel_delay_max,gnss_vel_delay_count,"
      << "velocity_delay_mean,velocity_delay_p95,velocity_delay_max,velocity_delay_count,"
      << "steering_delay_mean,steering_delay_p95,steering_delay_max,steering_delay_count\n";
  kpi_header_written_ = true;
}

void StorageExporter::WriteRawSample(const DiagSample& sample)
{
  if (!enabled_) return;
  WriteRawHeader();

  raw_file_ << std::fixed << std::setprecision(6);
  raw_file_
      << sample.stamp.seconds() << ","
      << (sample.is_activated ? "1" : "0") << ","
      << (sample.eskf_initialized ? "1" : "0") << ","
      << OptionalBoolString(sample.has_gnss_pos_update_applied,
                            sample.gnss_pos_update_applied) << ","
      << sample.gnss_pos_update_reason << ","
      << OptionalToString(sample.gnss_pos_nis) << ","
      << OptionalBoolString(sample.has_gnss_vel_update_applied,
                            sample.gnss_vel_update_applied) << ","
      << sample.gnss_vel_update_reason << ","
      << OptionalToString(sample.gnss_vel_nis) << ","
      << OptionalBoolString(sample.has_heading_yaw_update_applied,
                            sample.heading_yaw_update_applied) << ","
      << sample.heading_yaw_update_reason << ","
      << OptionalToString(sample.heading_yaw_nis) << ","
      << OptionalToString(sample.gnss_delay) << ","
      << OptionalToString(sample.gnss_vel_delay) << ","
      << OptionalToString(sample.velocity_delay) << ","
      << OptionalToString(sample.steering_delay) << "\n";
}

void StorageExporter::WriteKpiSnapshot(const KpiSnapshot& snapshot)
{
  if (!enabled_) return;
  WriteKpiHeader();

  auto write_stat = [this](const StatSummary& stat) {
    kpi_file_ << stat.mean << "," << stat.p95 << "," << stat.max << "," << stat.count << ",";
  };

  kpi_file_ << std::fixed << std::setprecision(6);
  kpi_file_ << snapshot.stamp.seconds() << "," << snapshot.window_sec << ",";
  kpi_file_ << snapshot.output_availability.output_count << ","
            << snapshot.output_availability.expected_count << ","
            << snapshot.output_availability.ratio << ",";
  if (snapshot.output_availability.has_output) {
    kpi_file_ << snapshot.output_availability.last_age_sec << ",";
  } else {
    kpi_file_ << ",";
  }
  kpi_file_ << snapshot.diag_rate_hz << "," << snapshot.output_rate_hz << ",";

  kpi_file_ << snapshot.gnss_pos_update.ratio << ","
            << snapshot.gnss_pos_update.applied << ","
            << snapshot.gnss_pos_update.total << ",";
  kpi_file_ << snapshot.gnss_vel_update.ratio << ","
            << snapshot.gnss_vel_update.applied << ","
            << snapshot.gnss_vel_update.total << ",";
  kpi_file_ << snapshot.heading_yaw_update.ratio << ","
            << snapshot.heading_yaw_update.applied << ","
            << snapshot.heading_yaw_update.total << ",";

  write_stat(snapshot.gnss_pos_nis);
  write_stat(snapshot.gnss_vel_nis);
  write_stat(snapshot.heading_yaw_nis);
  write_stat(snapshot.gnss_delay);
  write_stat(snapshot.gnss_vel_delay);
  write_stat(snapshot.velocity_delay);

  kpi_file_ << snapshot.steering_delay.mean << ","
            << snapshot.steering_delay.p95 << ","
            << snapshot.steering_delay.max << ","
            << snapshot.steering_delay.count << "\n";
}

}  // namespace autodriver::tools
