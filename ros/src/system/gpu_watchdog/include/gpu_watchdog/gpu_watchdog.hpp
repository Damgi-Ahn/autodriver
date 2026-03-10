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
enum class AlertLevel { OK, WARN, ERROR };

// ---------------------------------------------------------------------------
// GpuSnapshot — raw readings from a single poll cycle
// ---------------------------------------------------------------------------
struct GpuSnapshot {
    float gpu_load_pct{0.0f};       ///< GR3D load from tegrastats / sysfs
    float emc_load_pct{0.0f};       ///< EMC (memory controller) load
    float gpu_temp_c{0.0f};         ///< GPU thermal zone temperature
    float cpu_temp_c{0.0f};
    uint64_t gpu_mem_used_kb{0};
    uint64_t gpu_mem_total_kb{0};
    bool throttling{false};         ///< Thermal throttling active
};

// ---------------------------------------------------------------------------
// GpuWatchdog — ROS2 node
//
// Polls GPU health from sysfs / tegrastats at configurable rate.
// Detects:
//   - GPU stall   : load == 0 for > stall_threshold_s seconds
//   - Memory OOM  : used > mem_warn_pct of total
//   - Thermal     : temp > throttle_temp_c OR throttling flag set
//
// Publishes:
//   /system/watchdog/status  (diagnostic_msgs/DiagnosticArray) at poll rate
//   /system/watchdog/alert   (std_msgs/String) on level change
// ---------------------------------------------------------------------------
class GpuWatchdog : public rclcpp::Node {
public:
    explicit GpuWatchdog(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
    ~GpuWatchdog() override;

private:
    void poll_loop();
    GpuSnapshot read_snapshot();
    AlertLevel  evaluate(const GpuSnapshot& snap);
    void        publish(const GpuSnapshot& snap, AlertLevel level);

    // sysfs paths (Jetson Orin / JetPack 6.0)
    static constexpr const char* kGpuLoadPath  = "/sys/devices/gpu.0/load";
    static constexpr const char* kGpuTempPath  = "/sys/class/thermal/thermal_zone1/temp";
    static constexpr const char* kCpuTempPath  = "/sys/class/thermal/thermal_zone0/temp";
    static constexpr const char* kEmcLoadPath  = "/sys/kernel/debug/bpmp/debug/clk/emc/rate";

    // Config (loaded from ROS params)
    float    poll_rate_hz_{2.0f};
    float    stall_threshold_s_{5.0f};
    float    mem_warn_pct_{90.0f};
    float    throttle_temp_c_{85.0f};

    // State
    std::thread           poll_thread_;
    std::atomic<bool>     running_{false};
    AlertLevel            last_level_{AlertLevel::OK};
    float                 stall_accumulator_s_{0.0f};

    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                 alert_pub_;
};

} // namespace autodriver::system
