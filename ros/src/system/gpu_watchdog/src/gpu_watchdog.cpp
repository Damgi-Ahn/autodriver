#include "gpu_watchdog/gpu_watchdog.hpp"

#include <fstream>
#include <sstream>
#include <chrono>

namespace autodriver::system {

GpuWatchdog::GpuWatchdog(const rclcpp::NodeOptions& options)
    : rclcpp::Node("gpu_watchdog", options)
{
    declare_parameter("poll_rate_hz",       2.0);
    declare_parameter("stall_threshold_s",  5.0);
    declare_parameter("mem_warn_pct",       90.0);
    declare_parameter("throttle_temp_c",    85.0);

    poll_rate_hz_      = get_parameter("poll_rate_hz").as_double();
    stall_threshold_s_ = get_parameter("stall_threshold_s").as_double();
    mem_warn_pct_      = get_parameter("mem_warn_pct").as_double();
    throttle_temp_c_   = get_parameter("throttle_temp_c").as_double();

    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/system/watchdog/status", 10);
    alert_pub_ = create_publisher<std_msgs::msg::String>(
        "/system/watchdog/alert", 10);

    running_.store(true);
    poll_thread_ = std::thread([this]{ poll_loop(); });
}

GpuWatchdog::~GpuWatchdog()
{
    running_.store(false);
    if (poll_thread_.joinable()) poll_thread_.join();
}

void GpuWatchdog::poll_loop()
{
    const auto period = std::chrono::duration<double>(1.0 / poll_rate_hz_);

    while (running_.load()) {
        const auto next = std::chrono::steady_clock::now() + period;

        GpuSnapshot snap = read_snapshot();
        AlertLevel  level = evaluate(snap);
        publish(snap, level);

        std::this_thread::sleep_until(next);
    }
}

GpuSnapshot GpuWatchdog::read_snapshot()
{
    GpuSnapshot snap{};

    // ── GPU load ──────────────────────────────────────────────────────────
    {
        std::ifstream f(kGpuLoadPath);
        if (f) f >> snap.gpu_load_pct;
    }

    // ── GPU temperature ───────────────────────────────────────────────────
    {
        std::ifstream f(kGpuTempPath);
        if (f) {
            int milli_c = 0;
            f >> milli_c;
            snap.gpu_temp_c = static_cast<float>(milli_c) / 1000.0f;
        }
    }

    // ── CPU temperature ───────────────────────────────────────────────────
    {
        std::ifstream f(kCpuTempPath);
        if (f) {
            int milli_c = 0;
            f >> milli_c;
            snap.cpu_temp_c = static_cast<float>(milli_c) / 1000.0f;
        }
    }

    // ── GPU memory via CUDA ───────────────────────────────────────────────
    size_t free_b = 0, total_b = 0;
    if (cudaMemGetInfo(&free_b, &total_b) == cudaSuccess) {
        snap.gpu_mem_total_kb = total_b / 1024;
        snap.gpu_mem_used_kb  = (total_b - free_b) / 1024;
    }

    return snap;
}

AlertLevel GpuWatchdog::evaluate(const GpuSnapshot& snap)
{
    const float period_s = 1.0f / poll_rate_hz_;

    // Stall detection: GPU active but load == 0 for threshold duration
    if (snap.gpu_load_pct < 1.0f) {
        stall_accumulator_s_ += period_s;
    } else {
        stall_accumulator_s_ = 0.0f;
    }

    if (stall_accumulator_s_ >= stall_threshold_s_) return AlertLevel::ERROR;

    // Memory OOM
    if (snap.gpu_mem_total_kb > 0) {
        const float used_pct =
            100.0f * static_cast<float>(snap.gpu_mem_used_kb) /
            static_cast<float>(snap.gpu_mem_total_kb);
        if (used_pct >= mem_warn_pct_) return AlertLevel::WARN;
    }

    // Thermal throttling
    if (snap.gpu_temp_c >= throttle_temp_c_ || snap.throttling)
        return AlertLevel::WARN;

    return AlertLevel::OK;
}

void GpuWatchdog::publish(const GpuSnapshot& snap, AlertLevel level)
{
    // ── DiagnosticArray ───────────────────────────────────────────────────
    diagnostic_msgs::msg::DiagnosticArray diag;
    diag.header.stamp = rclcpp::Clock().now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "GPU/Watchdog";
    status.hardware_id = "jetson_orin";

    switch (level) {
    case AlertLevel::OK:    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;   break;
    case AlertLevel::WARN:  status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN; break;
    case AlertLevel::ERROR: status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR; break;
    }

    auto add_kv = [&](const std::string& k, const std::string& v) {
        diagnostic_msgs::msg::KeyValue kval;
        kval.key = k; kval.value = v;
        status.values.push_back(kval);
    };
    add_kv("gpu_load_pct",   std::to_string(snap.gpu_load_pct));
    add_kv("gpu_temp_c",     std::to_string(snap.gpu_temp_c));
    add_kv("cpu_temp_c",     std::to_string(snap.cpu_temp_c));
    add_kv("gpu_mem_used_kb",std::to_string(snap.gpu_mem_used_kb));
    add_kv("gpu_mem_total_kb",std::to_string(snap.gpu_mem_total_kb));
    add_kv("stall_accum_s",  std::to_string(stall_accumulator_s_));

    diag.status.push_back(status);
    diag_pub_->publish(diag);

    // ── Alert on level change ─────────────────────────────────────────────
    if (level != last_level_) {
        last_level_ = level;
        std_msgs::msg::String alert;
        if (level == AlertLevel::ERROR)
            alert.data = "ERROR: GPU stall detected";
        else if (level == AlertLevel::WARN)
            alert.data = "WARN: GPU thermal/memory pressure";
        else
            alert.data = "OK: GPU nominal";
        alert_pub_->publish(alert);
    }
}

} // namespace autodriver::system
