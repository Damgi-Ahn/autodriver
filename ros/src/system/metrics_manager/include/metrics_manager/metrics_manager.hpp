#pragma once

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace autodriver::system {

// ---------------------------------------------------------------------------
// CameraMetrics — per-camera snapshot updated externally.
// ---------------------------------------------------------------------------
struct CameraMetrics {
  uint32_t camera_id{0};
  float    fps{0.0f};
  uint64_t frames_captured{0};
  uint64_t frames_dropped{0};
};

// ---------------------------------------------------------------------------
// ModelMetrics — per-model inference performance snapshot.
// Auto-populated from /perception/{model}/results subscriptions,
// or manually updated via UpdateModelMetrics().
// ---------------------------------------------------------------------------
struct ModelMetrics {
  std::string model_name;
  float       avg_latency_ms{0.0f};
  uint64_t    batches_run{0};
  float       avg_batch_size{0.0f};
};

// ---------------------------------------------------------------------------
// GpuMetrics — hardware snapshot from sysfs + CUDA.
// ---------------------------------------------------------------------------
struct GpuMetrics {
  float gpu_utilization_pct{0.0f};
  float gpu_memory_used_mb{0.0f};
  float gpu_memory_total_mb{0.0f};
  float temperature_c{0.0f};
};

// ---------------------------------------------------------------------------
// TopicHealth — stall detection for a single topic.
// ---------------------------------------------------------------------------
struct TopicHealth {
  std::string topic;
  bool        healthy{false};     ///< false until first message received
  uint64_t    last_msg_ns{0};     ///< monotonic ns of last received message
  uint64_t    stall_count{0};     ///< consecutive stall intervals detected
};

// ---------------------------------------------------------------------------
// MetricsManager — ROS 2 node
//
// Aggregates all pipeline health data and publishes at 1 Hz on:
//   /system/metrics  (diagnostic_msgs/DiagnosticArray)
//
// Auto-subscriptions (configurable via model_names parameter):
//   /perception/{name}/results  → model latency/batch metrics
//
// Manual update API (thread-safe):
//   UpdateCameraMetrics(CameraMetrics)
//   UpdateModelMetrics(ModelMetrics)
//
// Topic stall detection:
//   Every publish cycle, topics in watched_topics_ that have not received
//   a message within stall_threshold_s_ are flagged in diagnostics.
// ---------------------------------------------------------------------------
class MetricsManager : public rclcpp::Node {
 public:
  explicit MetricsManager(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
  ~MetricsManager() override = default;

  // ── Thread-safe manual update API ────────────────────────────────────────
  void UpdateCameraMetrics(const CameraMetrics& m);
  void UpdateModelMetrics(const ModelMetrics& m);

 private:
  void CollectAndPublish();
  GpuMetrics ReadGpuMetrics() noexcept;

  // Subscribe to /perception/{name}/results and parse latency from JSON string.
  void SubscribeToModelTopics(const std::vector<std::string>& model_names);

  // Callback for model result topics — parses latency_ms and frames from payload.
  void OnModelResult(const std_msgs::msg::String& msg,
                     const std::string&           model_name);

  // Evaluate topic stall state; updates TopicHealth::healthy and stall_count.
  void CheckTopicHealth();

  // Build DiagnosticStatus for GPU, cameras, models, topic health.
  void AppendGpuStatus(diagnostic_msgs::msg::DiagnosticArray& out,
                       const GpuMetrics& gpu);
  void AppendCameraStatus(diagnostic_msgs::msg::DiagnosticArray& out);
  void AppendModelStatus(diagnostic_msgs::msg::DiagnosticArray& out);
  void AppendTopicHealthStatus(diagnostic_msgs::msg::DiagnosticArray& out);

  // sysfs paths (Jetson Orin)
  static constexpr const char* kGpuLoadPath =
      "/sys/devices/gpu.0/load";
  static constexpr const char* kGpuTempPath =
      "/sys/class/thermal/thermal_zone1/temp";

  // Config
  float       publish_rate_hz_{1.0f};
  float       stall_threshold_s_{5.0f};

  // Protected by metrics_mutex_
  mutable std::mutex                                  metrics_mutex_;
  std::unordered_map<uint32_t, CameraMetrics>         camera_metrics_;
  std::unordered_map<std::string, ModelMetrics>       model_metrics_;
  std::unordered_map<std::string, TopicHealth>        topic_health_;

  // Model result subscriptions (one per subscribed model)
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> model_subs_;

  rclcpp::TimerBase::SharedPtr                                    timer_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_;
};

}  // namespace autodriver::system
