#include "hybrid_localization_evaluation_tool/alert_engine.hpp"

#include <cmath>
#include <sstream>

namespace autodriver::tools {

AlertEngine::AlertEngine(const AlertThresholds& thresholds)
    : thresholds_(thresholds)
{
}

std::vector<AlertEvent> AlertEngine::Evaluate(const DiagSample& sample)
{
  std::vector<AlertEvent> events;

  const auto make = [&](AlertSeverity sev, AlertType type,
                        const std::string& key, double value,
                        const std::string& msg) {
    AlertEvent e;
    e.stamp    = sample.stamp;
    e.severity = sev;
    e.type     = type;
    e.key      = key;
    e.value    = value;
    e.message  = msg;
    events.push_back(e);
  };

  // ---- Activation / ESKF init changes (edge-triggered) -------------------
  if (sample.is_activated != prev_is_activated_) {
    std::ostringstream oss;
    oss << "is_activated changed: " << prev_is_activated_
        << " → " << sample.is_activated;
    make(AlertSeverity::WARN, AlertType::ACTIVATION_CHANGE,
         "is_activated", sample.is_activated ? 1.0 : 0.0, oss.str());
    prev_is_activated_ = sample.is_activated;
  }
  if (sample.eskf_initialized != prev_eskf_initialized_) {
    std::ostringstream oss;
    oss << "eskf_initialized changed: " << prev_eskf_initialized_
        << " → " << sample.eskf_initialized;
    make(AlertSeverity::INFO, AlertType::ACTIVATION_CHANGE,
         "eskf_initialized", sample.eskf_initialized ? 1.0 : 0.0, oss.str());
    prev_eskf_initialized_ = sample.eskf_initialized;
  }

  // ---- GNSS status change (edge-triggered) --------------------------------
  if (sample.gnss_status.has_value()) {
    const int cur = *sample.gnss_status;
    if (prev_gnss_status_ != -99 && cur != prev_gnss_status_) {
      std::ostringstream oss;
      oss << "gnss_status changed: " << prev_gnss_status_ << " → " << cur;
      const auto sev = (cur < prev_gnss_status_) ? AlertSeverity::WARN : AlertSeverity::INFO;
      make(sev, AlertType::GNSS_STATUS_CHANGE, "gnss_status",
           static_cast<double>(cur), oss.str());
    }
    prev_gnss_status_ = cur;
  }

  // ---- NIS gate violations ------------------------------------------------
  const auto check_nis = [&](const std::optional<double>& nis,
                              double gate,
                              const std::string& key) {
    if (!nis.has_value() || !std::isfinite(*nis)) return;
    if (*nis > gate) {
      std::ostringstream oss;
      oss << key << " NIS=" << *nis << " > gate=" << gate;
      make(AlertSeverity::WARN, AlertType::NIS_GATE_VIOLATION, key, *nis, oss.str());
    }
  };
  check_nis(sample.gnss_pos_nis,     thresholds_.nis_gate_gnss_pos, "gnss_pos_nis");
  check_nis(sample.gnss_vel_nis,     thresholds_.nis_gate_gnss_vel, "gnss_vel_nis");
  check_nis(sample.heading_yaw_nis,  thresholds_.nis_gate_heading,  "heading_yaw_nis");

  // ---- Sensor delays ------------------------------------------------------
  const auto check_delay = [&](const std::optional<double>& delay_ms,
                                const std::string& key) {
    if (!delay_ms.has_value() || !std::isfinite(*delay_ms)) return;
    const double v = *delay_ms;
    if (v > thresholds_.delay_error_ms) {
      std::ostringstream oss;
      oss << key << " delay=" << v << " ms (ERROR threshold=" << thresholds_.delay_error_ms << ")";
      make(AlertSeverity::ERROR, AlertType::SENSOR_DELAY, key, v, oss.str());
    } else if (v > thresholds_.delay_warn_ms) {
      std::ostringstream oss;
      oss << key << " delay=" << v << " ms (WARN threshold=" << thresholds_.delay_warn_ms << ")";
      make(AlertSeverity::WARN, AlertType::SENSOR_DELAY, key, v, oss.str());
    }
  };
  check_delay(sample.gnss_delay,     "gnss_delay");
  check_delay(sample.gnss_vel_delay, "gnss_vel_delay");
  check_delay(sample.velocity_delay, "velocity_delay");
  check_delay(sample.steering_delay, "steering_delay");

  // ---- IMU dt jitter ------------------------------------------------------
  if (sample.imu_dt_min_ms.has_value() && sample.imu_dt_max_ms.has_value()) {
    const double jitter = *sample.imu_dt_max_ms - *sample.imu_dt_min_ms;
    if (std::isfinite(jitter)) {
      if (jitter > thresholds_.imu_dt_jitter_error_ms) {
        std::ostringstream oss;
        oss << "IMU dt jitter=" << jitter << " ms (ERROR)";
        make(AlertSeverity::ERROR, AlertType::IMU_DT_JITTER, "imu_dt_jitter", jitter, oss.str());
      } else if (jitter > thresholds_.imu_dt_jitter_warn_ms) {
        std::ostringstream oss;
        oss << "IMU dt jitter=" << jitter << " ms (WARN)";
        make(AlertSeverity::WARN, AlertType::IMU_DT_JITTER, "imu_dt_jitter", jitter, oss.str());
      }
    }
  }

  // ---- Covariance health --------------------------------------------------
  if (sample.P_trace.has_value() && std::isfinite(*sample.P_trace)) {
    const double v = *sample.P_trace;
    if (v > thresholds_.P_trace_error) {
      std::ostringstream oss;
      oss << "P_trace=" << v << " > " << thresholds_.P_trace_error << " (ERROR)";
      make(AlertSeverity::ERROR, AlertType::COVARIANCE_DRIFT, "P_trace", v, oss.str());
    } else if (v > thresholds_.P_trace_warn) {
      std::ostringstream oss;
      oss << "P_trace=" << v << " > " << thresholds_.P_trace_warn << " (WARN)";
      make(AlertSeverity::WARN, AlertType::COVARIANCE_DRIFT, "P_trace", v, oss.str());
    }
  }

  if (sample.P_min_eig.has_value() && std::isfinite(*sample.P_min_eig)) {
    const double v = *sample.P_min_eig;
    if (v < thresholds_.P_min_eig_error) {
      std::ostringstream oss;
      oss << "P_min_eig=" << v << " < " << thresholds_.P_min_eig_error << " (ERROR)";
      make(AlertSeverity::ERROR, AlertType::COVARIANCE_DRIFT, "P_min_eig", v, oss.str());
    } else if (v < thresholds_.P_min_eig_warn) {
      std::ostringstream oss;
      oss << "P_min_eig=" << v << " < 0 (WARN)";
      make(AlertSeverity::WARN, AlertType::COVARIANCE_DRIFT, "P_min_eig", v, oss.str());
    }
  }

  return events;
}

std::vector<AlertEvent> AlertEngine::EvaluateOutputAvailability(
    const rclcpp::Time& stamp, double availability_ratio)
{
  std::vector<AlertEvent> events;

  if (!std::isfinite(availability_ratio)) return events;

  AlertSeverity sev;
  if (availability_ratio < thresholds_.output_avail_error) {
    sev = AlertSeverity::ERROR;
  } else if (availability_ratio < thresholds_.output_avail_warn) {
    sev = AlertSeverity::WARN;
  } else {
    return events;
  }

  std::ostringstream oss;
  oss << "output availability=" << static_cast<int>(availability_ratio * 100.0) << "%";
  AlertEvent e;
  e.stamp    = stamp;
  e.severity = sev;
  e.type     = AlertType::OUTPUT_UNAVAILABLE;
  e.key      = "output_availability";
  e.value    = availability_ratio;
  e.message  = oss.str();
  events.push_back(e);
  return events;
}

}  // namespace autodriver::tools
