#include "gpu_watchdog/gpu_watchdog.hpp"

#include <cuda_runtime.h>

#include <fstream>
#include <chrono>
#include <string>

namespace autodriver::system {

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------

GpuWatchdog::GpuWatchdog(const rclcpp::NodeOptions& options)
    : rclcpp::Node("gpu_watchdog", options)
{
  declare_parameter("poll_rate_hz",      2.0);
  declare_parameter("stall_threshold_s", 5.0);
  declare_parameter("mem_warn_pct",      90.0);
  declare_parameter("throttle_temp_c",   85.0);

  poll_rate_hz_      = static_cast<float>(get_parameter("poll_rate_hz").as_double());
  stall_threshold_s_ = static_cast<float>(get_parameter("stall_threshold_s").as_double());
  mem_warn_pct_      = static_cast<float>(get_parameter("mem_warn_pct").as_double());
  throttle_temp_c_   = static_cast<float>(get_parameter("throttle_temp_c").as_double());

  diag_pub_  = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/system/watchdog/status", rclcpp::QoS{10});
  alert_pub_ = create_publisher<std_msgs::msg::String>(
      "/system/watchdog/alert", rclcpp::QoS{10});

  running_.store(true);
  poll_thread_ = std::thread([this] { PollLoop(); });

  RCLCPP_INFO(get_logger(),
              "GpuWatchdog started — poll=%.1f Hz, stall=%.1f s, "
              "mem_warn=%.0f%%, throttle=%.1f°C",
              poll_rate_hz_, stall_threshold_s_, mem_warn_pct_, throttle_temp_c_);
}

GpuWatchdog::~GpuWatchdog()
{
  running_.store(false);
  if (poll_thread_.joinable()) poll_thread_.join();
}

// ---------------------------------------------------------------------------
// PollLoop
// ---------------------------------------------------------------------------

void GpuWatchdog::PollLoop()
{
  const auto period = std::chrono::duration<double>(1.0 / poll_rate_hz_);

  while (running_.load()) {
    const auto next = std::chrono::steady_clock::now() + period;

    GpuSnapshot snap  = ReadSnapshot();
    AlertLevel  level = Evaluate(snap);
    Publish(snap, level);

    std::this_thread::sleep_until(next);
  }
}

// ---------------------------------------------------------------------------
// ReadSnapshot — reads hardware state from sysfs + CUDA
// ---------------------------------------------------------------------------

GpuSnapshot GpuWatchdog::ReadSnapshot() noexcept
{
  GpuSnapshot snap{};

  // ── GPU engine load ───────────────────────────────────────────────────────
  // /sys/devices/gpu.0/load reports 0–1000 (per-mille) on Jetson Orin.
  {
    std::ifstream f(kGpuLoadPath);
    if (f) {
      int raw = 0;
      f >> raw;
      snap.gpu_load_pct = static_cast<float>(raw) / 10.0f;  // 0–100
    }
  }

  // ── GPU temperature ───────────────────────────────────────────────────────
  {
    std::ifstream f(kGpuTempPath);
    if (f) {
      int milli_c = 0;
      f >> milli_c;
      snap.gpu_temp_c = static_cast<float>(milli_c) / 1000.0f;
    }
  }

  // ── CPU temperature ───────────────────────────────────────────────────────
  {
    std::ifstream f(kCpuTempPath);
    if (f) {
      int milli_c = 0;
      f >> milli_c;
      snap.cpu_temp_c = static_cast<float>(milli_c) / 1000.0f;
    }
  }

  // ── CUDA device memory ────────────────────────────────────────────────────
  size_t free_b = 0, total_b = 0;
  if (cudaMemGetInfo(&free_b, &total_b) == cudaSuccess) {
    snap.gpu_mem_total_kb = total_b / 1024;
    snap.gpu_mem_used_kb  = (total_b - free_b) / 1024;
  }

  // ── Thermal throttle flag (Tegra power mode) ──────────────────────────────
  // /sys/kernel/debug/bpmp/debug/clk/emc/mrq_rate_locked — only accessible
  // in debug builds. Use temperature threshold as proxy.
  snap.throttling = (snap.gpu_temp_c > throttle_temp_c_);

  return snap;
}

// ---------------------------------------------------------------------------
// Evaluate — pure logic, no I/O. Exposed publicly for unit tests.
// ---------------------------------------------------------------------------

AlertLevel GpuWatchdog::Evaluate(const GpuSnapshot& snap) noexcept
{
  const float period_s = 1.0f / poll_rate_hz_;

  // ── Stall detection ───────────────────────────────────────────────────────
  // GPU stall: load near zero for longer than stall_threshold_s.
  // Note: during node startup the GPU load is legitimately 0 — the stall
  // accumulator only fires an ERROR if the threshold is consistently exceeded.
  if (snap.gpu_load_pct < 1.0f) {
    stall_accumulator_s_ += period_s;
  } else {
    stall_accumulator_s_ = 0.0f;
  }

  if (stall_accumulator_s_ >= stall_threshold_s_) {
    return AlertLevel::kError;
  }

  // ── Thermal ───────────────────────────────────────────────────────────────
  if (snap.throttling || snap.gpu_temp_c >= throttle_temp_c_) {
    return AlertLevel::kWarn;
  }

  // ── Memory pressure ───────────────────────────────────────────────────────
  if (snap.gpu_mem_total_kb > 0) {
    const float used_pct =
        100.0f * static_cast<float>(snap.gpu_mem_used_kb) /
                 static_cast<float>(snap.gpu_mem_total_kb);
    if (used_pct >= mem_warn_pct_) {
      return AlertLevel::kWarn;
    }
  }

  return AlertLevel::kOk;
}

// ---------------------------------------------------------------------------
// Publish
// ---------------------------------------------------------------------------

void GpuWatchdog::Publish(const GpuSnapshot& snap, AlertLevel level)
{
  // ── DiagnosticArray ───────────────────────────────────────────────────────
  diagnostic_msgs::msg::DiagnosticArray diag;
  diag.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name        = "GPU/Watchdog";
  status.hardware_id = "jetson_orin";

  using DS = diagnostic_msgs::msg::DiagnosticStatus;
  switch (level) {
    case AlertLevel::kOk:    status.level = DS::OK;    break;
    case AlertLevel::kWarn:  status.level = DS::WARN;  break;
    case AlertLevel::kError: status.level = DS::ERROR; break;
  }

  auto add_kv = [&](const std::string& k, const std::string& v) {
    diagnostic_msgs::msg::KeyValue kval;
    kval.key   = k;
    kval.value = v;
    status.values.push_back(kval);
  };
  add_kv("gpu_load_pct",    std::to_string(snap.gpu_load_pct));
  add_kv("gpu_temp_c",      std::to_string(snap.gpu_temp_c));
  add_kv("cpu_temp_c",      std::to_string(snap.cpu_temp_c));
  add_kv("gpu_mem_used_kb", std::to_string(snap.gpu_mem_used_kb));
  add_kv("gpu_mem_total_kb",std::to_string(snap.gpu_mem_total_kb));
  add_kv("stall_accum_s",   std::to_string(stall_accumulator_s_));
  add_kv("throttling",      snap.throttling ? "true" : "false");

  diag.status.push_back(status);
  diag_pub_->publish(diag);

  // ── Alert on level transition ─────────────────────────────────────────────
  if (level != last_level_) {
    last_level_ = level;
    std_msgs::msg::String alert;
    switch (level) {
      case AlertLevel::kError:
        alert.data = "ERROR: GPU stall detected (accumulator=" +
                     std::to_string(stall_accumulator_s_) + "s)";
        RCLCPP_ERROR(get_logger(), "%s", alert.data.c_str());
        break;
      case AlertLevel::kWarn:
        alert.data = "WARN: GPU thermal/memory pressure (temp=" +
                     std::to_string(snap.gpu_temp_c) + "°C)";
        RCLCPP_WARN(get_logger(), "%s", alert.data.c_str());
        break;
      case AlertLevel::kOk:
        alert.data = "OK: GPU nominal";
        RCLCPP_INFO(get_logger(), "%s", alert.data.c_str());
        break;
    }
    alert_pub_->publish(alert);
  }
}

}  // namespace autodriver::system
