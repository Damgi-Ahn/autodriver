#pragma once

#include "camera_config.hpp"
#include "camera_pipeline.hpp"

#include <ipc_unix_socket/ipc_unix_socket.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <atomic>
#include <memory>
#include <thread>
#include <unordered_map>
#include <vector>

namespace autodriver::camera {

// ---------------------------------------------------------------------------
// CameraManager — ROS2 node
//
// Responsibilities:
//   1. Load cameras.yaml
//   2. Open cameras sequentially (100ms stagger, prevent I2C congestion)
//   3. Start one CameraPipeline per camera
//   4. Relay DMABUF FDs + FrameMeta to inference_manager via Unix socket
//   5. Publish sensor_msgs/CameraInfo (if calibration enabled)
//   6. Publish sensor_msgs/CompressedImage debug stream (optional)
//   7. Apply frame-drop policy for lower-priority cameras
// ---------------------------------------------------------------------------
class CameraManager : public rclcpp::Node {
public:
    explicit CameraManager(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
    ~CameraManager() override;

private:
    // ── Initialisation ──────────────────────────────────────────────────────
    void load_config();
    void connect_ipc_server();
    void start_cameras();

    // ── Per-frame path ───────────────────────────────────────────────────────
    /// Called from CameraPipeline appsink thread.
    void on_frame(int dmabuf_fd,
                  uint64_t timestamp,
                  uint32_t camera_id,
                  uint32_t width,
                  uint32_t height,
                  uint32_t format);

    /// Determine whether a frame from `camera_id` should be sent or dropped.
    bool should_send_frame(uint32_t camera_id);

    // ── Calibration / CameraInfo ─────────────────────────────────────────────
    void load_calibration(const CameraConfig& cam);
    void publish_camera_info(uint32_t camera_id, uint64_t timestamp_ns);

    // ── Debug image ──────────────────────────────────────────────────────────
    void publish_debug_image(uint32_t camera_id,
                             const uint8_t* jpeg_data,
                             size_t jpeg_size,
                             uint64_t timestamp_ns);

    // ── Metrics timer ────────────────────────────────────────────────────────
    void publish_metrics();

    // ── Shutdown ─────────────────────────────────────────────────────────────
    void shutdown_cameras();

    // ── Data members ─────────────────────────────────────────────────────────
    CameraManagerConfig config_;

    // IPC
    int ipc_server_fd_{-1};  ///< Listening socket
    int ipc_peer_fd_{-1};    ///< Connected peer (inference_manager)

    // Pipelines (indexed by camera_id)
    std::unordered_map<uint32_t, std::unique_ptr<CameraPipeline>> pipelines_;

    // ROS2 publishers (per camera)
    std::unordered_map<uint32_t,
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_pubs_;
    std::unordered_map<uint32_t,
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr> debug_image_pubs_;

    // Calibration data (per camera, optional)
    struct CalibrationData {
        std::vector<double> K;   ///< 3×3 intrinsic matrix, row-major
        std::vector<double> D;   ///< Distortion coefficients
        std::vector<double> R;   ///< Rectification matrix
        std::vector<double> P;   ///< Projection matrix
        uint32_t width{0};
        uint32_t height{0};
    };
    std::unordered_map<uint32_t, CalibrationData> calibrations_;

    // Frame-drop state per camera
    struct DropState {
        uint64_t last_sent_ns{0};
        uint64_t min_interval_ns{0};  ///< Derived from min/max FPS in policy
    };
    std::unordered_map<uint32_t, DropState> drop_states_;

    rclcpp::TimerBase::SharedPtr metrics_timer_;

    std::string ipc_socket_path_;
    bool        debug_stream_enabled_{false};
};

} // namespace autodriver::camera
