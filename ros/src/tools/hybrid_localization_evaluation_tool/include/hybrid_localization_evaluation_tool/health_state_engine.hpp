#pragma once

#include "hybrid_localization_evaluation_tool/kpi_engine.hpp"

#include <cstdint>
#include <string>

namespace autodriver::tools {

// ---------------------------------------------------------------------------
// Localization health state machine
//   OK → DEGRADED → FAILURE → RECOVERING → OK
// ---------------------------------------------------------------------------
enum class HealthState : uint8_t {
  OK         = 0,
  DEGRADED   = 1,
  FAILURE    = 2,
  RECOVERING = 3,
};

inline const char* HealthStateStr(HealthState s)
{
  switch (s) {
    case HealthState::OK:         return "OK";
    case HealthState::DEGRADED:   return "DEGRADED";
    case HealthState::FAILURE:    return "FAILURE";
    case HealthState::RECOVERING: return "RECOVERING";
  }
  return "UNKNOWN";
}

inline const char* HealthStateColor(HealthState s)
{
  switch (s) {
    case HealthState::OK:         return "#27ae60";
    case HealthState::DEGRADED:   return "#f39c12";
    case HealthState::FAILURE:    return "#e74c3c";
    case HealthState::RECOVERING: return "#3498db";
  }
  return "#7f8c8d";
}

// ---------------------------------------------------------------------------
// Configurable health transition thresholds
// ---------------------------------------------------------------------------
struct HealthThresholds {
  double nis_violation_warn    = 0.20;  // > 20% violations → DEGRADED
  double nis_violation_failure = 0.50;  // > 50% violations → FAILURE
  double availability_warn     = 0.90;  // < 90% → DEGRADED
  double availability_failure  = 0.70;  // < 70% → FAILURE
  double p_trace_warn          = 100.0;
  double p_trace_failure       = 1000.0;
  int    recover_stable_steps  = 5;    // consecutive OK updates to leave RECOVERING
};

// ---------------------------------------------------------------------------
// HealthStateEngine — transition logic called from KPI timer (1 Hz)
// ---------------------------------------------------------------------------
class HealthStateEngine {
 public:
  explicit HealthStateEngine(const HealthThresholds& thr = HealthThresholds{});

  // Call once per KPI timer tick; returns current state after transition
  HealthState Update(const KpiSnapshot& kpi, bool eskf_initialized);

  HealthState   state()      const { return state_; }
  const char*   state_str()  const { return HealthStateStr(state_); }
  const char*   state_color()const { return HealthStateColor(state_); }
  bool          transitioned()const { return transitioned_; }

  void set_thresholds(const HealthThresholds& t) { thr_ = t; }

 private:
  HealthThresholds thr_;
  HealthState      state_          = HealthState::OK;
  bool             transitioned_   = false;
  bool             prev_eskf_init_ = false;
  int              stable_count_   = 0;
};

}  // namespace autodriver::tools
