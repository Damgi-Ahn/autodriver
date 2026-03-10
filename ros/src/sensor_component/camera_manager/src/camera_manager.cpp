#include "camera_manager/camera_manager.hpp"

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <filesystem>
#include <stdexcept>
#include <thread>

namespace autodriver::camera {

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------

CameraManager::CameraManager(const rclcpp::NodeOptions& options)
    : rclcpp::Node("camera_manager", options) {
  declare_parameter("cameras_yaml",
      (std::filesystem::path(get_package_share_directory("camera_manager")) /
       "config" / "cameras.yaml").string());
  declare_parameter("ipc_socket_path", "/tmp/autodriver/frames.sock");
  declare_parameter("debug_stream",    false);

  ipc_socket_path_     = get_parameter("ipc_socket_path").as_string();
  debug_stream_enabled_ = get_parameter("debug_stream").as_bool();

  LoadConfig();
  ConnectToInferenceManager();
  StartCameras();

  metrics_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      [this] { PublishMetrics(); });

  RCLCPP_INFO(get_logger(), "CameraManager ready — %zu camera(s)",
              config_.cameras.size());
}

CameraManager::~CameraManager() {
  metrics_timer_.reset();
  ShutdownCameras();
  ipc::CloseSocket(ipc_fd_);
  ipc_fd_ = -1;
}

// ---------------------------------------------------------------------------
// LoadConfig
// ---------------------------------------------------------------------------

void CameraManager::LoadConfig() {
  const auto yaml_path = get_parameter("cameras_yaml").as_string();

  std::string err;
  auto maybe_cfg = LoadCameraConfig(yaml_path, &err);
  if (!maybe_cfg) {
    throw std::runtime_error("CameraManager: " + err);
  }
  config_ = std::move(*maybe_cfg);

  // Build drop-state table.
  for (const auto& cam : config_.cameras) {
    DropState ds;
    const auto& policy = config_.frame_drop_policy;
    const bool is_high = [&] {
      for (auto id : policy.high_priority_ids)
        if (id == cam.id) return true;
      return false;
    }();

    if (!is_high && policy.max_fps > 0) {
      ds.min_interval_ns = 1'000'000'000ULL / policy.max_fps;
    }
    drop_states_[cam.id] = ds;
  }
}

// ---------------------------------------------------------------------------
// ConnectToInferenceManager
//
// camera_manager is the IPC CLIENT.
// tensorrt_inference_manager binds the server socket first.
// ---------------------------------------------------------------------------

void CameraManager::ConnectToInferenceManager() {
  RCLCPP_INFO(get_logger(), "Connecting to inference_manager IPC at %s …",
              ipc_socket_path_.c_str());

  ipc::IpcStatus status;
  ipc_fd_ = ipc::CreateClientSocket(
      ipc_socket_path_, &status,
      /*max_retries=*/60,
      /*retry_delay_ms=*/500);

  if (ipc_fd_ < 0) {
    throw std::runtime_error(
        std::string("Failed to connect to inference_manager IPC: ") +
        ipc::StatusString(status));
  }

  // Enlarge socket buffer: 12 cameras × 2 MB / camera ≈ 24 MB.
  ipc::SetSocketBufferSize(ipc_fd_, 24 * 1024 * 1024);

  RCLCPP_INFO(get_logger(), "IPC connected to inference_manager");
}

// ---------------------------------------------------------------------------
// StartCameras
// ---------------------------------------------------------------------------

void CameraManager::StartCameras() {
  for (const auto& spec : config_.cameras) {
    // Calibration
    if (spec.calibration.enabled) LoadCalibration(spec);

    // ROS2 publishers
    const std::string base = "/camera/" + spec.name;
    camera_info_pubs_[spec.id] =
        create_publisher<sensor_msgs::msg::CameraInfo>(
            base + "/camera_info", rclcpp::QoS(10));

    if (debug_stream_enabled_) {
      debug_image_pubs_[spec.id] =
          create_publisher<sensor_msgs::msg::CompressedImage>(
              base + "/image_raw/compressed",
              rclcpp::QoS(5).best_effort());
    }

    // Pipeline
    auto pipeline = std::make_unique<CameraPipeline>(spec);
    pipeline->SetDebugStreamEnabled(debug_stream_enabled_);

    pipeline->SetFrameCallback(
        [this](int fd, uint64_t ts, uint32_t id,
               uint32_t w, uint32_t h, uint32_t fmt) {
          OnFrame(fd, ts, id, w, h, fmt);
        });

    if (debug_stream_enabled_) {
      pipeline->SetDebugFrameCallback(
          [this](uint32_t id, const uint8_t* data,
                 size_t sz, uint64_t ts) {
            PublishDebugImage(id, data, sz, ts);
          });
    }

    if (!pipeline->Start()) {
      RCLCPP_ERROR(get_logger(),
                   "Camera %u (%s): pipeline failed to start — skipped",
                   spec.id, spec.name.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Camera %u (%s): started",
                  spec.id, spec.name.c_str());
    }

    pipelines_[spec.id] = std::move(pipeline);

    // 100 ms stagger — prevents I2C congestion when multiple MIPI cameras
    // initialise simultaneously (Jetson Orin GMSL / CSI arbiter).
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

// ---------------------------------------------------------------------------
// Per-frame hot path
// ---------------------------------------------------------------------------

void CameraManager::OnFrame(int dmabuf_fd, uint64_t timestamp,
                               uint32_t camera_id,
                               uint32_t width, uint32_t height,
                               uint32_t format) {
  if (!ShouldSendFrame(camera_id)) {
    // Frame dropped by policy — do NOT close dmabuf_fd here.
    // GStreamer owns it; we only hold a transient reference.
    auto it = pipelines_.find(camera_id);
    if (it != pipelines_.end()) {
      // dropped counter is incremented inside CameraPipeline; nothing to do.
    }
    return;
  }

  const ipc::FrameMeta meta{camera_id, timestamp, width, height, format};

  // Serialise concurrent SendFd calls from multiple appsink threads.
  {
    std::lock_guard<std::mutex> lock(ipc_mutex_);
    const ipc::IpcStatus s = ipc::SendFd(ipc_fd_, dmabuf_fd, meta);
    if (!ipc::IsOk(s) && s != ipc::IpcStatus::kWouldBlock) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "IPC SendFd failed for camera %u: %s",
                           camera_id, ipc::StatusString(s));
    }
  }

  // CameraInfo at frame rate (non-blocking — uses QoS depth for backpressure).
  PublishCameraInfo(camera_id, timestamp);
}

bool CameraManager::ShouldSendFrame(uint32_t camera_id) noexcept {
  auto it = drop_states_.find(camera_id);
  if (it == drop_states_.end()) return true;

  DropState& ds = it->second;
  if (ds.min_interval_ns == 0) return true;  // High-priority: always send

  const uint64_t now = ipc::NowNs();
  if (now - ds.last_sent_ns >= ds.min_interval_ns) {
    ds.last_sent_ns = now;
    return true;
  }
  return false;
}

// ---------------------------------------------------------------------------
// Calibration loading
// ---------------------------------------------------------------------------

void CameraManager::LoadCalibration(const CameraSpec& spec) {
  if (spec.calibration.file.empty()) return;

  YAML::Node root;
  try {
    root = YAML::LoadFile(spec.calibration.file);
  } catch (const YAML::Exception& e) {
    RCLCPP_WARN(get_logger(),
                "Camera %u: calibration file load failed: %s", spec.id, e.what());
    return;
  }

  CalibData cal;
  cal.width    = root["image_width"].as<uint32_t>(spec.width);
  cal.height   = root["image_height"].as<uint32_t>(spec.height);
  cal.frame_id = root["camera_name"].as<std::string>(spec.name);

  // OpenCV YAML calibration format.
  auto read_mat = [&](const char* key, auto& arr) {
    if (!root[key]) return;
    const auto data = root[key]["data"].as<std::vector<double>>();
    for (size_t i = 0; i < std::min(data.size(), arr.size()); ++i)
      arr[i] = data[i];
  };
  auto read_vec = [&](const char* key, std::vector<double>& vec) {
    if (!root[key]) return;
    vec = root[key]["data"].as<std::vector<double>>();
  };

  read_mat("camera_matrix",         cal.K);
  read_mat("rectification_matrix",  cal.R);
  read_mat("projection_matrix",     cal.P);
  read_vec("distortion_coefficients", cal.D);

  calibrations_[spec.id] = std::move(cal);
  RCLCPP_INFO(get_logger(), "Camera %u: calibration loaded from %s",
              spec.id, spec.calibration.file.c_str());
}

// ---------------------------------------------------------------------------
// ROS2 publishing
// ---------------------------------------------------------------------------

void CameraManager::PublishCameraInfo(uint32_t camera_id,
                                        uint64_t timestamp_ns) {
  auto pub_it = camera_info_pubs_.find(camera_id);
  if (pub_it == camera_info_pubs_.end()) return;

  sensor_msgs::msg::CameraInfo msg;
  msg.header.stamp    = rclcpp::Time(static_cast<int64_t>(timestamp_ns));
  msg.header.frame_id = "camera_" + std::to_string(camera_id);

  auto cal_it = calibrations_.find(camera_id);
  if (cal_it != calibrations_.end()) {
    const CalibData& cal = cal_it->second;
    msg.width      = cal.width;
    msg.height     = cal.height;
    msg.header.frame_id = cal.frame_id;
    msg.distortion_model = "plumb_bob";

    for (size_t i = 0; i < 9;  ++i) msg.k[i] = cal.K[i];
    for (size_t i = 0; i < 9;  ++i) msg.r[i] = cal.R[i];
    for (size_t i = 0; i < 12; ++i) msg.p[i] = cal.P[i];
    msg.d = cal.D;
  }

  pub_it->second->publish(msg);
}

void CameraManager::PublishDebugImage(uint32_t       camera_id,
                                        const uint8_t* jpeg_data,
                                        size_t         jpeg_size,
                                        uint64_t       timestamp_ns) {
  auto pub_it = debug_image_pubs_.find(camera_id);
  if (pub_it == debug_image_pubs_.end()) return;

  sensor_msgs::msg::CompressedImage msg;
  msg.header.stamp    = rclcpp::Time(static_cast<int64_t>(timestamp_ns));
  msg.header.frame_id = "camera_" + std::to_string(camera_id);
  msg.format = "jpeg";
  msg.data.assign(jpeg_data, jpeg_data + jpeg_size);

  pub_it->second->publish(std::move(msg));
}

// ---------------------------------------------------------------------------
// Metrics (1 Hz)
// ---------------------------------------------------------------------------

void CameraManager::PublishMetrics() {
  for (const auto& [id, pipeline] : pipelines_) {
    RCLCPP_DEBUG(get_logger(),
                 "Camera %u: captured=%" PRIu64 " dropped=%" PRIu64
                 " healthy=%d",
                 id,
                 pipeline->frames_captured(),
                 pipeline->frames_dropped(),
                 static_cast<int>(pipeline->IsHealthy()));
  }
}

// ---------------------------------------------------------------------------
// Shutdown
// ---------------------------------------------------------------------------

void CameraManager::ShutdownCameras() {
  for (auto& [id, pipeline] : pipelines_) {
    pipeline->Stop();
  }
  pipelines_.clear();
}

}  // namespace autodriver::camera
