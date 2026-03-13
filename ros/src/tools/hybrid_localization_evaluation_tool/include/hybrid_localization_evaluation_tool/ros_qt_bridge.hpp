#pragma once

#include "hybrid_localization_evaluation_tool/diagnostic_parser.hpp"
#include "hybrid_localization_evaluation_tool/kpi_engine.hpp"

#include <mutex>

namespace autodriver::tools {

class RosQtBridge {
 public:
  void UpdateSample(const DiagSample& sample);
  void UpdateKpi(const KpiSnapshot& snapshot);

  bool GetLatestSample(DiagSample* out_sample) const;
  bool GetLatestKpi(KpiSnapshot* out_snapshot) const;

 private:
  mutable std::mutex mutex_;
  bool has_sample_ = false;
  bool has_kpi_ = false;
  DiagSample latest_sample_;
  KpiSnapshot latest_kpi_;
};

}  // namespace autodriver::tools
