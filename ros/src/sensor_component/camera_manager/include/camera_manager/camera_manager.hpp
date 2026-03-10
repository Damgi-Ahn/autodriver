#pragma once

#include "camera_config.hpp"
#include "camera_pipeline.hpp"

#include <ipc_unix_socket/ipc_unix_socket.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <mutex>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autodriver::camera {

// ---------------------------------------------------------------------------
// CameraManager — ROS2 node
//
// Parameter declarations (set via launch file or ROS2 CLI):
//   cameras_yaml    (string) — path to cameras.yaml
//   ipc_socket_path (string) — Unix socket path to inference_manager server
//   debug_stream    (bool)   — enable per-camera NVJPEG CompressedImage stream
//
// Responsibilities:
//   1. Load cameras.yaml (via LoadCameraConfig)
//   2. Connect to tensorrt_inference_manager IPC server (client role)
//   3. Start one CameraPipeline per camera (100ms stagger for I2C stability)
//   4. On each frame: apply drop policy, then SendFd to inference_manager
//   5. Publish sensor_msgs/CameraInfo (if calibration is available)
//   6. Publish sensor_msgs/CompressedImage debug stream (if enabled)
//   7. 1 Hz metrics log via create_wall_timer
// ---------------------------------------------------------------------------
class CameraManager : public rclcpp::Node {
 public:
  explicit CameraManager(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
  ~CameraManager() override;

  // Non-copyable, non-movable (owns GStreamer + IPC resources)
  CameraManager(const CameraManager&)            = delete;
  CameraManager& operator=(const CameraManager&) = delete;

 private:
  // ── Initialisation (called in constructor) ────────────────────────────
  void LoadConfig();
  void ConnectToInferenceManager();
  void StartCameras();

  // ── Per-frame hot path (called from CameraPipeline appsink thread) ────
  void OnFrame(int      dmabuf_fd,
               uint64_t timestamp,
               uint32_t camera_id,
               uint32_t width,
               uint32_t height,
               uint32_t format);

  // Returns true if this frame passes the drop policy for camera_id.
  [[nodiscard]] bool ShouldSendFrame(uint32_t camera_id) noexcept;

  // ── ROS2 publishing ───────────────────────────────────────────────────
  void PublishCameraInfo(uint32_t camera_id, uint64_t timestamp_ns);
  void PublishDebugImage(uint32_t       camera_id,
                          const uint8_t* jpeg_data,
                          size_t         jpeg_size,
                          uint64_t       timestamp_ns);

  // ── Calibration ───────────────────────────────────────────────────────
  void LoadCalibration(const CameraSpec& spec);

  // ── Metrics (1 Hz timer) ──────────────────────────────────────────────
  void PublishMetrics();

  // ── Shutdown ──────────────────────────────────────────────────────────
  void ShutdownCameras();

  // ── Configuration ────────────────────────────────────────────────────
  CameraManagerConfig config_;
  std::string         ipc_socket_path_;
  bool                debug_stream_enabled_{false};

  // ── IPC (camera_manager is the CLIENT) ───────────────────────────────
  int ipc_fd_{-1};  ///< Connected socket to tensorrt_inference_manager

  // ── Pipelines ─────────────────────────────────────────────────────────
  std::unordered_map<uint32_t, std::unique_ptr<CameraPipeline>> pipelines_;

  // ── ROS2 publishers (per camera) ─────────────────────────────────────
  std::unordered_map<
      uint32_t,
      rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr>
      camera_info_pubs_;

  std::unordered_map<
      uint32_t,
      rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr>
      debug_image_pubs_;

  // ── Calibration data (optional, per camera) ──────────────────────────
  struct CalibData {
    std::array<double, 9>  K{};   ///< 3x3 intrinsic, row-major
    std::vector<double>    D;     ///< distortion coeffs (4 or 5)
    std::array<double, 9>  R{};   ///< rectification matrix
    std::array<double, 12> P{};   ///< 3x4 projection matrix
    uint32_t width{0};
    uint32_t height{0};
    std::string frame_id;
  };
  std::unordered_map<uint32_t, CalibData> calibrations_;

  // ── Frame-drop state (one per camera) ────────────────────────────────
  struct DropState {
    uint64_t last_sent_ns{0};
    uint64_t min_interval_ns{0};  ///< 0 = always send (high priority)
  };
  std::unordered_map<uint32_t, DropState> drop_states_;

  // ── IPC write mutex ───────────────────────────────────────────────────
  // Multiple CameraPipeline appsink threads call OnFrame() concurrently.
  // Serialise SendFd calls on the single ipc_fd_.
  std::mutex ipc_mutex_;

  rclcpp::TimerBase::SharedPtr metrics_timer_;
};

}  // namespace autodriver::camera
