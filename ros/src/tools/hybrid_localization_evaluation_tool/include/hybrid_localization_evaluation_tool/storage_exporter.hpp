#pragma once

#include "hybrid_localization_evaluation_tool/alert_engine.hpp"
#include "hybrid_localization_evaluation_tool/diagnostic_parser.hpp"
#include "hybrid_localization_evaluation_tool/kpi_engine.hpp"

#include <fstream>
#include <string>
#include <vector>

namespace autodriver::tools {

class StorageExporter {
 public:
  StorageExporter() = default;

  bool Enable(const std::string& output_dir);
  bool enabled() const { return enabled_; }

  // Called per diagnostic sample
  void WriteRawSample(const DiagSample& sample);
  // Called per KPI timer tick
  void WriteKpiSnapshot(const KpiSnapshot& snapshot);
  // Called when new alerts are generated
  void WriteEvents(const std::vector<AlertEvent>& events);
  // Called once at session end (or on-demand export)
  void WriteSessionSummary(const rclcpp::Time& start,
                           const rclcpp::Time& end,
                           const KpiSnapshot& final_kpi,
                           size_t total_alerts,
                           size_t warn_alerts,
                           size_t error_alerts);

  const std::string& raw_path()     const { return raw_path_; }
  const std::string& kpi_path()     const { return kpi_path_; }
  const std::string& events_path()  const { return events_path_; }
  const std::string& session_path() const { return session_path_; }

 private:
  void WriteRawHeader();
  void WriteKpiHeader();
  void WriteEventsHeader();
  void WriteSessionHeader();

  bool enabled_ = false;

  bool raw_header_written_     = false;
  bool kpi_header_written_     = false;
  bool events_header_written_  = false;
  bool session_header_written_ = false;

  std::string raw_path_;
  std::string kpi_path_;
  std::string events_path_;
  std::string session_path_;

  std::ofstream raw_file_;
  std::ofstream kpi_file_;
  std::ofstream events_file_;
  std::ofstream session_file_;
};

}  // namespace autodriver::tools
