#include "metrics_manager/metrics_manager.hpp"

#include <cuda_runtime.h>

#include <fstream>
#include <string>

namespace autodriver::system {

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

MetricsManager::MetricsManager(const rclcpp::NodeOptions& options)
    : rclcpp::Node("metrics_manager", options)
{
  declare_parameter("publish_rate_hz",    1.0);
  declare_parameter("stall_threshold_s",  5.0);
  declare_parameter("model_names",
                    std::vector<std::string>{"yolo", "lane", "segmentation"});

  publish_rate_hz_   = static_cast<float>(get_parameter("publish_rate_hz").as_double());
  stall_threshold_s_ = static_cast<float>(get_parameter("stall_threshold_s").as_double());

  const auto model_names =
      get_parameter("model_names").as_string_array();

  pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/system/metrics", rclcpp::QoS{10});

  SubscribeToModelTopics(model_names);

  const auto period_ms =
      static_cast<int>(1000.0f / publish_rate_hz_);
  timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      [this] { CollectAndPublish(); });

  RCLCPP_INFO(get_logger(),
              "MetricsManager started — %.1f Hz, monitoring %zu model(s)",
              publish_rate_hz_, model_names.size());
}

// ---------------------------------------------------------------------------
// Thread-safe update API
// ---------------------------------------------------------------------------

void MetricsManager::UpdateCameraMetrics(const CameraMetrics& m)
{
  std::lock_guard<std::mutex> lock(metrics_mutex_);
  camera_metrics_[m.camera_id] = m;
}

void MetricsManager::UpdateModelMetrics(const ModelMetrics& m)
{
  std::lock_guard<std::mutex> lock(metrics_mutex_);
  model_metrics_[m.model_name] = m;
}

// ---------------------------------------------------------------------------
// Topic subscriptions
// ---------------------------------------------------------------------------

void MetricsManager::SubscribeToModelTopics(
    const std::vector<std::string>& model_names)
{
  for (const auto& name : model_names) {
    const std::string topic = "/perception/" + name + "/results";

    // Initialise TopicHealth entry.
    {
      std::lock_guard<std::mutex> lock(metrics_mutex_);
      TopicHealth th;
      th.topic   = topic;
      th.healthy = false;
      topic_health_[topic] = th;
    }

    auto sub = create_subscription<std_msgs::msg::String>(
        topic, rclcpp::QoS{10},
        [this, name](const std_msgs::msg::String& msg) {
          OnModelResult(msg, name);
        });
    model_subs_.push_back(sub);

    RCLCPP_DEBUG(get_logger(), "Subscribed to %s", topic.c_str());
  }
}

// ---------------------------------------------------------------------------
// Model result callback — parses JSON string produced by InferenceManager
//
// Expected format:
//   {"model":"yolo","frames":4,"latency_ms":3.14,"output_floats":4000}
// ---------------------------------------------------------------------------

void MetricsManager::OnModelResult(const std_msgs::msg::String& msg,
                                    const std::string&           model_name)
{
  const std::string& data = msg.data;

  // Simple key-value extraction without a JSON library.
  auto extract_float = [&](const std::string& key) -> float {
    const std::string needle = "\"" + key + "\":";
    const size_t pos = data.find(needle);
    if (pos == std::string::npos) return 0.0f;
    try {
      return std::stof(data.substr(pos + needle.size()));
    } catch (...) { return 0.0f; }
  };
  auto extract_int = [&](const std::string& key) -> uint64_t {
    const std::string needle = "\"" + key + "\":";
    const size_t pos = data.find(needle);
    if (pos == std::string::npos) return 0;
    try {
      return static_cast<uint64_t>(std::stoull(data.substr(pos + needle.size())));
    } catch (...) { return 0; }
  };

  const float    latency_ms  = extract_float("latency_ms");
  const uint64_t frames      = extract_int("frames");

  const uint64_t now_ns = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());

  std::lock_guard<std::mutex> lock(metrics_mutex_);

  // Update model metrics (exponential moving average for latency).
  auto& mm = model_metrics_[model_name];
  mm.model_name = model_name;
  mm.batches_run++;
  // EMA alpha = 0.1 for latency smoothing
  mm.avg_latency_ms = (mm.batches_run == 1)
                      ? latency_ms
                      : 0.9f * mm.avg_latency_ms + 0.1f * latency_ms;
  mm.avg_batch_size  = (mm.batches_run == 1)
                      ? static_cast<float>(frames)
                      : 0.9f * mm.avg_batch_size + 0.1f * static_cast<float>(frames);

  // Update topic health.
  const std::string topic = "/perception/" + model_name + "/results";
  auto& th = topic_health_[topic];
  th.last_msg_ns = now_ns;
  th.healthy     = true;
  th.stall_count = 0;
}

// ---------------------------------------------------------------------------
// CheckTopicHealth — called each publish cycle
// ---------------------------------------------------------------------------

void MetricsManager::CheckTopicHealth()
{
  // Caller holds metrics_mutex_.
  const uint64_t now_ns = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());
  const uint64_t threshold_ns =
      static_cast<uint64_t>(stall_threshold_s_ * 1e9f);

  for (auto& [topic, th] : topic_health_) {
    if (!th.healthy) continue;  // Never received a message yet — not a stall
    if (th.last_msg_ns == 0) continue;

    if ((now_ns - th.last_msg_ns) > threshold_ns) {
      th.stall_count++;
      if (th.stall_count == 1) {
        RCLCPP_WARN(get_logger(),
                    "Topic stall detected: %s (no message for %.1f s)",
                    topic.c_str(), stall_threshold_s_);
      }
    } else {
      th.stall_count = 0;
    }
  }
}

// ---------------------------------------------------------------------------
// CollectAndPublish — timer callback (1 Hz)
// ---------------------------------------------------------------------------

void MetricsManager::CollectAndPublish()
{
  GpuMetrics gpu = ReadGpuMetrics();

  diagnostic_msgs::msg::DiagnosticArray msg;
  msg.header.stamp = this->now();

  AppendGpuStatus(msg, gpu);

  {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    CheckTopicHealth();
    AppendCameraStatus(msg);
    AppendModelStatus(msg);
    AppendTopicHealthStatus(msg);
  }

  pub_->publish(msg);
}

// ---------------------------------------------------------------------------
// AppendGpuStatus
// ---------------------------------------------------------------------------

void MetricsManager::AppendGpuStatus(
    diagnostic_msgs::msg::DiagnosticArray& out, const GpuMetrics& gpu)
{
  using DS = diagnostic_msgs::msg::DiagnosticStatus;
  DS status;
  status.name        = "GPU";
  status.hardware_id = "jetson_orin_gpu";
  status.level       = DS::OK;

  auto kv = [](const std::string& k, const std::string& v) {
    diagnostic_msgs::msg::KeyValue kval;
    kval.key = k; kval.value = v;
    return kval;
  };
  status.values.push_back(kv("utilization_pct", std::to_string(gpu.gpu_utilization_pct)));
  status.values.push_back(kv("memory_used_mb",  std::to_string(gpu.gpu_memory_used_mb)));
  status.values.push_back(kv("memory_total_mb", std::to_string(gpu.gpu_memory_total_mb)));
  status.values.push_back(kv("temperature_c",   std::to_string(gpu.temperature_c)));
  out.status.push_back(status);
}

// ---------------------------------------------------------------------------
// AppendCameraStatus
// ---------------------------------------------------------------------------

void MetricsManager::AppendCameraStatus(diagnostic_msgs::msg::DiagnosticArray& out)
{
  using DS = diagnostic_msgs::msg::DiagnosticStatus;
  for (const auto& [id, m] : camera_metrics_) {
    DS status;
    status.name  = "Camera/" + std::to_string(id);
    status.level = (m.frames_dropped == 0) ? DS::OK : DS::WARN;

    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "fps";             kv.value = std::to_string(m.fps);
    status.values.push_back(kv);
    kv.key = "frames_captured"; kv.value = std::to_string(m.frames_captured);
    status.values.push_back(kv);
    kv.key = "frames_dropped";  kv.value = std::to_string(m.frames_dropped);
    status.values.push_back(kv);

    out.status.push_back(status);
  }
}

// ---------------------------------------------------------------------------
// AppendModelStatus
// ---------------------------------------------------------------------------

void MetricsManager::AppendModelStatus(diagnostic_msgs::msg::DiagnosticArray& out)
{
  using DS = diagnostic_msgs::msg::DiagnosticStatus;
  for (const auto& [name, m] : model_metrics_) {
    DS status;
    status.name  = "Model/" + name;
    status.level = DS::OK;

    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "avg_latency_ms"; kv.value = std::to_string(m.avg_latency_ms);
    status.values.push_back(kv);
    kv.key = "avg_batch_size"; kv.value = std::to_string(m.avg_batch_size);
    status.values.push_back(kv);
    kv.key = "batches_run";    kv.value = std::to_string(m.batches_run);
    status.values.push_back(kv);

    out.status.push_back(status);
  }
}

// ---------------------------------------------------------------------------
// AppendTopicHealthStatus
// ---------------------------------------------------------------------------

void MetricsManager::AppendTopicHealthStatus(
    diagnostic_msgs::msg::DiagnosticArray& out)
{
  using DS = diagnostic_msgs::msg::DiagnosticStatus;
  for (const auto& [topic, th] : topic_health_) {
    DS status;
    status.name  = "Topic" + topic;
    status.level = (th.stall_count > 0) ? DS::ERROR : DS::OK;

    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "healthy";     kv.value = th.healthy ? "true" : "false";
    status.values.push_back(kv);
    kv.key = "stall_count"; kv.value = std::to_string(th.stall_count);
    status.values.push_back(kv);

    out.status.push_back(status);
  }
}

// ---------------------------------------------------------------------------
// ReadGpuMetrics
// ---------------------------------------------------------------------------

GpuMetrics MetricsManager::ReadGpuMetrics() noexcept
{
  GpuMetrics m{};

  // GPU load (0–1000 per-mille on Jetson Orin)
  {
    std::ifstream f(kGpuLoadPath);
    if (f) {
      int raw = 0;
      f >> raw;
      m.gpu_utilization_pct = static_cast<float>(raw) / 10.0f;
    }
  }

  // Temperature
  {
    std::ifstream f(kGpuTempPath);
    if (f) {
      int milli_c = 0;
      f >> milli_c;
      m.temperature_c = static_cast<float>(milli_c) / 1000.0f;
    }
  }

  // CUDA memory
  size_t free_b = 0, total_b = 0;
  if (cudaMemGetInfo(&free_b, &total_b) == cudaSuccess) {
    const float kMb = 1024.0f * 1024.0f;
    m.gpu_memory_total_mb = static_cast<float>(total_b) / kMb;
    m.gpu_memory_used_mb  = m.gpu_memory_total_mb -
                            static_cast<float>(free_b) / kMb;
  }

  return m;
}

}  // namespace autodriver::system
