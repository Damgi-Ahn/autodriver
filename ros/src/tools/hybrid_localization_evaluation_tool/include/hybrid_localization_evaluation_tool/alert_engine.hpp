#pragma once

#include "hybrid_localization_evaluation_tool/diagnostic_parser.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace autodriver::tools {

// ---------------------------------------------------------------------------
// Alert severity
// ---------------------------------------------------------------------------
enum class AlertSeverity : uint8_t { INFO = 0, WARN = 1, ERROR = 2 };

inline const char* AlertSeverityStr(AlertSeverity s)
{
  switch (s) {
    case AlertSeverity::INFO:  return "INFO";
    case AlertSeverity::WARN:  return "WARN";
    case AlertSeverity::ERROR: return "ERROR";
  }
  return "INFO";
}

// ---------------------------------------------------------------------------
// Alert event types
// ---------------------------------------------------------------------------
enum class AlertType : uint8_t {
  NIS_GATE_VIOLATION,
  SENSOR_DELAY,
  COVARIANCE_DRIFT,
  IMU_DT_JITTER,
  OUTPUT_UNAVAILABLE,
  ACTIVATION_CHANGE,
  GNSS_STATUS_CHANGE,
};

inline const char* AlertTypeStr(AlertType t)
{
  switch (t) {
    case AlertType::NIS_GATE_VIOLATION:  return "NIS_GATE_VIOLATION";
    case AlertType::SENSOR_DELAY:        return "SENSOR_DELAY";
    case AlertType::COVARIANCE_DRIFT:    return "COVARIANCE_DRIFT";
    case AlertType::IMU_DT_JITTER:       return "IMU_DT_JITTER";
    case AlertType::OUTPUT_UNAVAILABLE:  return "OUTPUT_UNAVAILABLE";
    case AlertType::ACTIVATION_CHANGE:   return "ACTIVATION_CHANGE";
    case AlertType::GNSS_STATUS_CHANGE:  return "GNSS_STATUS_CHANGE";
  }
  return "UNKNOWN";
}

// ---------------------------------------------------------------------------
// A single alert event
// ---------------------------------------------------------------------------
struct AlertEvent {
  rclcpp::Time stamp;
  AlertSeverity severity = AlertSeverity::INFO;
  AlertType type = AlertType::NIS_GATE_VIOLATION;
  std::string message;
  std::string key;
  double value = 0.0;
};

// ---------------------------------------------------------------------------
// Configurable thresholds
// ---------------------------------------------------------------------------
struct AlertThresholds {
  // NIS gates (should match hybrid_localization params)
  double nis_gate_gnss_pos  = 11.34;
  double nis_gate_gnss_vel  = 11.34;
  double nis_gate_heading   = 6.63;

  // Delay thresholds [ms]
  double delay_warn_ms  = 200.0;
  double delay_error_ms = 500.0;

  // IMU dt jitter [ms]
  double imu_dt_jitter_warn_ms  = 2.0;   // max - min > warn
  double imu_dt_jitter_error_ms = 5.0;

  // P trace thresholds
  double P_trace_warn  = 100.0;
  double P_trace_error = 1000.0;

  // P min_eig thresholds (negative = diverging)
  double P_min_eig_warn  = 0.0;  // negative triggers WARN
  double P_min_eig_error = -1.0; // more negative triggers ERROR

  // Output availability (ratio below threshold)
  double output_avail_warn  = 0.90;
  double output_avail_error = 0.70;
};

// ---------------------------------------------------------------------------
// AlertEngine: evaluates one DiagSample and emits zero or more AlertEvents
// ---------------------------------------------------------------------------
class AlertEngine {
 public:
  explicit AlertEngine(const AlertThresholds& thresholds = AlertThresholds{});

  // Evaluate a new sample; returns any new alerts generated this cycle
  std::vector<AlertEvent> Evaluate(const DiagSample& sample);

  // Evaluate output availability ratio (called from KPI timer)
  std::vector<AlertEvent> EvaluateOutputAvailability(
      const rclcpp::Time& stamp, double availability_ratio);

  const AlertThresholds& thresholds() const { return thresholds_; }
  void set_thresholds(const AlertThresholds& t) { thresholds_ = t; }

 private:
  AlertThresholds thresholds_;

  // State for edge-triggered events
  bool prev_is_activated_ = true;
  bool prev_eskf_initialized_ = false;
  int  prev_gnss_status_ = -99;
};

}  // namespace autodriver::tools
