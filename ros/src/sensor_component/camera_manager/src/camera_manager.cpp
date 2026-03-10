#include "camera_manager/camera_manager.hpp"

#include <chrono>
#include <filesystem>
#include <stdexcept>
#include <thread>

namespace autodriver::camera {

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------

CameraManager::CameraManager(const rclcpp::NodeOptions& options)
    : rclcpp::Node("camera_manager", options)
{
    // Declare ROS2 parameters
    declare_parameter("cameras_yaml",    "config/cameras.yaml");
    declare_parameter("ipc_socket_path", "/tmp/autodriver/frames.sock");
    declare_parameter("debug_stream",    false);

    ipc_socket_path_     = get_parameter("ipc_socket_path").as_string();
    debug_stream_enabled_ = get_parameter("debug_stream").as_bool();

    load_config();
    connect_ipc_server();
    start_cameras();

    metrics_timer_ = create_wall_timer(
        std::chrono::seconds(1),
        [this]{ publish_metrics(); });

    RCLCPP_INFO(get_logger(), "CameraManager started with %zu camera(s)",
                config_.cameras.size());
}

CameraManager::~CameraManager()
{
    shutdown_cameras();
    if (ipc_peer_fd_ >= 0)   ::close(ipc_peer_fd_);
    if (ipc_server_fd_ >= 0) ipc::close_server_socket(ipc_server_fd_, ipc_socket_path_);
}

// ---------------------------------------------------------------------------
// Initialisation
// ---------------------------------------------------------------------------

void CameraManager::load_config()
{
    const auto yaml_path = get_parameter("cameras_yaml").as_string();
    config_ = load_camera_config(yaml_path);

    // Initialise drop states for normal-priority cameras
    for (const auto& cam : config_.cameras) {
        const auto& policy = config_.frame_drop_policy;
        bool is_high = false;
        for (auto id : policy.high_priority_cameras)
            if (id == cam.id) { is_high = true; break; }

        DropState ds;
        if (!is_high && policy.normal_priority_max_fps > 0) {
            ds.min_interval_ns =
                1'000'000'000ULL / policy.normal_priority_max_fps;
        }
        drop_states_[cam.id] = ds;
    }
}

void CameraManager::connect_ipc_server()
{
    // Ensure socket directory exists
    std::filesystem::create_directories(
        std::filesystem::path(ipc_socket_path_).parent_path());

    ipc_server_fd_ = ipc::create_server_socket(ipc_socket_path_);
    if (ipc_server_fd_ < 0)
        throw std::runtime_error("Failed to create IPC server socket: " + ipc_socket_path_);

    RCLCPP_INFO(get_logger(), "IPC server listening on %s", ipc_socket_path_.c_str());

    // Wait for inference_manager to connect (blocking — runs in constructor thread)
    ipc_peer_fd_ = ipc::accept_connection(ipc_server_fd_);
    if (ipc_peer_fd_ < 0)
        throw std::runtime_error("IPC accept failed");

    ipc::set_socket_buffer_size(ipc_peer_fd_, 4 * 1024 * 1024);
    RCLCPP_INFO(get_logger(), "IPC peer connected");
}

void CameraManager::start_cameras()
{
    // Sequential open — 100ms stagger to prevent I2C bus congestion
    for (const auto& cam_cfg : config_.cameras) {
        // Calibration
        if (cam_cfg.calibration.enabled) load_calibration(cam_cfg);

        // ROS2 publishers
        const std::string info_topic = "/camera/" + cam_cfg.name + "/camera_info";
        camera_info_pubs_[cam_cfg.id] =
            create_publisher<sensor_msgs::msg::CameraInfo>(info_topic, 10);

        if (debug_stream_enabled_) {
            const std::string dbg_topic = "/camera/" + cam_cfg.name + "/compressed";
            debug_image_pubs_[cam_cfg.id] =
                create_publisher<sensor_msgs::msg::CompressedImage>(dbg_topic, 5);
        }

        // Create pipeline
        auto pipeline = std::make_unique<CameraPipeline>(cam_cfg);
        pipeline->set_frame_callback(
            [this](int fd, uint64_t ts, uint32_t id, uint32_t w, uint32_t h, uint32_t fmt) {
                on_frame(fd, ts, id, w, h, fmt);
            });

        if (!pipeline->start()) {
            RCLCPP_ERROR(get_logger(), "Camera %u (%s) failed to start — skipping",
                         cam_cfg.id, cam_cfg.name.c_str());
        }

        pipelines_[cam_cfg.id] = std::move(pipeline);

        // 100ms stagger
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// ---------------------------------------------------------------------------
// Per-frame hot path
// ---------------------------------------------------------------------------

void CameraManager::on_frame(int dmabuf_fd, uint64_t timestamp,
                               uint32_t camera_id,
                               uint32_t width, uint32_t height, uint32_t format)
{
    if (!should_send_frame(camera_id)) {
        ::close(dmabuf_fd);
        if (auto it = pipelines_.find(camera_id); it != pipelines_.end())
            ; // dropped — pipeline counter already incremented
        return;
    }

    ipc::FrameMeta meta{camera_id, timestamp, width, height, format};
    if (!ipc::send_fd(ipc_peer_fd_, dmabuf_fd, meta)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "IPC send failed for camera %u", camera_id);
    }

    // Publish calibration info at frame rate
    publish_camera_info(camera_id, timestamp);
}

bool CameraManager::should_send_frame(uint32_t camera_id)
{
    auto& ds = drop_states_[camera_id];
    if (ds.min_interval_ns == 0) return true;  // High priority — always send

    const uint64_t now = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());

    if (now - ds.last_sent_ns >= ds.min_interval_ns) {
        ds.last_sent_ns = now;
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Calibration helpers
// ---------------------------------------------------------------------------

void CameraManager::load_calibration(const CameraConfig& cam)
{
    // TODO(Stage 4): parse OpenCV-format YAML and populate calibrations_[cam.id]
    RCLCPP_INFO(get_logger(), "Loading calibration for camera %s from %s",
                cam.name.c_str(), cam.calibration.file.c_str());
}

void CameraManager::publish_camera_info(uint32_t camera_id, uint64_t timestamp_ns)
{
    auto it = camera_info_pubs_.find(camera_id);
    if (it == camera_info_pubs_.end()) return;

    // TODO(Stage 4): fill msg from calibrations_[camera_id]
    sensor_msgs::msg::CameraInfo msg;
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp_ns));
    it->second->publish(msg);
}

// ---------------------------------------------------------------------------
// Debug image
// ---------------------------------------------------------------------------

void CameraManager::publish_debug_image(uint32_t camera_id,
                                         const uint8_t* jpeg_data,
                                         size_t jpeg_size,
                                         uint64_t timestamp_ns)
{
    auto it = debug_image_pubs_.find(camera_id);
    if (it == debug_image_pubs_.end()) return;

    sensor_msgs::msg::CompressedImage msg;
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp_ns));
    msg.format = "jpeg";
    msg.data.assign(jpeg_data, jpeg_data + jpeg_size);
    it->second->publish(msg);
}

// ---------------------------------------------------------------------------
// Metrics
// ---------------------------------------------------------------------------

void CameraManager::publish_metrics()
{
    for (const auto& [id, pipeline] : pipelines_) {
        RCLCPP_DEBUG(get_logger(),
                     "Camera %u: captured=%lu dropped=%lu healthy=%d",
                     id,
                     pipeline->frames_captured(),
                     pipeline->frames_dropped(),
                     pipeline->is_healthy());
    }
}

// ---------------------------------------------------------------------------
// Shutdown
// ---------------------------------------------------------------------------

void CameraManager::shutdown_cameras()
{
    for (auto& [id, pipeline] : pipelines_) {
        pipeline->stop();
    }
    pipelines_.clear();
}

} // namespace autodriver::camera
