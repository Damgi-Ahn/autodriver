#pragma once

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

namespace autodriver::system {

// ---------------------------------------------------------------------------
// GPU health alert levels
// ---------------------------------------------------------------------------
enum class AlertLevel : uint8_t { kOk = 0, kWarn = 1, kError = 2 };

// ---------------------------------------------------------------------------
// GpuSnapshot — raw hardware readings from one poll cycle.
// ---------------------------------------------------------------------------
struct GpuSnapshot {
  float    gpu_load_pct{0.0f};      ///< GR3D engine load [0, 100]
  float    gpu_temp_c{0.0f};        ///< GPU thermal zone (°C)
  float    cpu_temp_c{0.0f};        ///< CPU thermal zone (°C)
  uint64_t gpu_mem_used_kb{0};      ///< CUDA device memory in use
  uint64_t gpu_mem_total_kb{0};     ///< Total CUDA device memory
  bool     throttling{false};       ///< Thermal throttling active
};

// ---------------------------------------------------------------------------
// GpuWatchdog — ROS 2 node
//
// Polls Jetson Orin GPU health from sysfs + CUDA at configurable rate.
//
// Detections:
//   GPU stall   : gpu_load_pct == 0 for > stall_threshold_s
//   Memory OOM  : used > mem_warn_pct of total
//   Thermal     : gpu_temp_c > throttle_temp_c  OR  throttling == true
//
// Publishes:
//   /system/watchdog/status  (diagnostic_msgs/DiagnosticArray) every poll
//   /system/watchdog/alert   (std_msgs/String) only on AlertLevel change
//
// Sysfs paths (Jetson Orin / JetPack 6):
//   GPU load   : /sys/devices/gpu.0/load                 (0-100)
//   GPU temp   : /sys/class/thermal/thermal_zone1/temp   (millidegrees C)
//   CPU temp   : /sys/class/thermal/thermal_zone0/temp   (millidegrees C)
// ---------------------------------------------------------------------------
class GpuWatchdog : public rclcpp::Node {
 public:
  explicit GpuWatchdog(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
  ~GpuWatchdog() override;

  // ── Testable helpers (public for unit testing) ────────────────────────────
  /// Pure evaluation logic — no I/O. Testable without hardware.
  AlertLevel Evaluate(const GpuSnapshot& snap) noexcept;

  /// Reset stall accumulator (used in tests to clear state between cases).
  void ResetStallAccumulator() noexcept { stall_accumulator_s_ = 0.0f; }

  float stall_accumulator_s() const noexcept { return stall_accumulator_s_; }

 private:
  void PollLoop();
  GpuSnapshot ReadSnapshot() noexcept;
  void Publish(const GpuSnapshot& snap, AlertLevel level);

  // Sysfs path constants (Jetson Orin)
  static constexpr const char* kGpuLoadPath = "/sys/devices/gpu.0/load";
  static constexpr const char* kGpuTempPath =
      "/sys/class/thermal/thermal_zone1/temp";
  static constexpr const char* kCpuTempPath =
      "/sys/class/thermal/thermal_zone0/temp";

  // Config
  float poll_rate_hz_{2.0f};
  float stall_threshold_s_{5.0f};
  float mem_warn_pct_{90.0f};
  float throttle_temp_c_{85.0f};

  // Runtime state
  std::thread        poll_thread_;
  std::atomic<bool>  running_{false};
  AlertLevel         last_level_{AlertLevel::kOk};
  float              stall_accumulator_s_{0.0f};

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                 alert_pub_;
};

}  // namespace autodriver::system
