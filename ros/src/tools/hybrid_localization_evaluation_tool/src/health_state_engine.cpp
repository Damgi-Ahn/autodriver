#include "hybrid_localization_evaluation_tool/health_state_engine.hpp"

#include <algorithm>

namespace autodriver::tools {

HealthStateEngine::HealthStateEngine(const HealthThresholds& thr) : thr_(thr) {}

HealthState HealthStateEngine::Update(const KpiSnapshot& kpi, bool eskf_initialized)
{
  transitioned_ = false;

  // ---- Compute condition flags -----------------------------------------------
  const double max_nis_viol = std::max({kpi.gnss_pos_nis_violation_rate,
                                        kpi.gnss_vel_nis_violation_rate,
                                        kpi.heading_nis_violation_rate});
  const double avail        = kpi.output_availability.ratio;
  const bool p_trace_warn   = kpi.P_trace_latest.has_value()
                                  && *kpi.P_trace_latest > thr_.p_trace_warn;
  const bool p_trace_fail   = kpi.P_trace_latest.has_value()
                                  && *kpi.P_trace_latest > thr_.p_trace_failure;

  const bool is_degraded = !eskf_initialized
                           || max_nis_viol > thr_.nis_violation_warn
                           || avail        < thr_.availability_warn
                           || p_trace_warn;

  const bool is_failure  = !eskf_initialized
                           || max_nis_viol > thr_.nis_violation_failure
                           || avail        < thr_.availability_failure
                           || p_trace_fail;

  // ---- ESKF re-init edge → RECOVERING ----------------------------------------
  if (!prev_eskf_init_ && eskf_initialized && state_ == HealthState::FAILURE) {
    state_        = HealthState::RECOVERING;
    stable_count_ = 0;
    transitioned_ = true;
  }
  prev_eskf_init_ = eskf_initialized;

  // ---- State transitions ------------------------------------------------------
  const HealthState prev = state_;

  switch (state_) {
    case HealthState::OK:
      if (is_failure)       { state_ = HealthState::FAILURE;   stable_count_ = 0; }
      else if (is_degraded) { state_ = HealthState::DEGRADED;  stable_count_ = 0; }
      break;

    case HealthState::DEGRADED:
      if (is_failure)       { state_ = HealthState::FAILURE;   stable_count_ = 0; }
      else if (!is_degraded){ state_ = HealthState::OK; }
      break;

    case HealthState::FAILURE:
      if (!is_failure)      { state_ = HealthState::RECOVERING; stable_count_ = 0; }
      break;

    case HealthState::RECOVERING:
      if (is_failure)       { state_ = HealthState::FAILURE;   stable_count_ = 0; }
      else if (is_degraded) { state_ = HealthState::DEGRADED;  stable_count_ = 0; }
      else {
        ++stable_count_;
        if (stable_count_ >= thr_.recover_stable_steps) {
          state_ = HealthState::OK;
          stable_count_ = 0;
        }
      }
      break;
  }

  if (state_ != prev) transitioned_ = true;
  return state_;
}

}  // namespace autodriver::tools
