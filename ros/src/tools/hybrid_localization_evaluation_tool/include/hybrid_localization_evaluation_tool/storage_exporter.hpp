#pragma once

#include "hybrid_localization_evaluation_tool/diagnostic_parser.hpp"
#include "hybrid_localization_evaluation_tool/kpi_engine.hpp"

#include <fstream>
#include <string>

namespace autodriver::tools {

class StorageExporter {
 public:
  StorageExporter() = default;

  bool Enable(const std::string& output_dir);
  bool enabled() const { return enabled_; }

  void WriteRawSample(const DiagSample& sample);
  void WriteKpiSnapshot(const KpiSnapshot& snapshot);

  const std::string& raw_path() const { return raw_path_; }
  const std::string& kpi_path() const { return kpi_path_; }

 private:
  void WriteRawHeader();
  void WriteKpiHeader();

  bool enabled_ = false;
  bool raw_header_written_ = false;
  bool kpi_header_written_ = false;

  std::string raw_path_;
  std::string kpi_path_;
  std::ofstream raw_file_;
  std::ofstream kpi_file_;
};

}  // namespace autodriver::tools
