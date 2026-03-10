#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace autodriver::camera {

// ---------------------------------------------------------------------------
// Per-camera configuration (mirrors cameras.yaml schema)
// ---------------------------------------------------------------------------

enum class Priority { HIGH, NORMAL };

struct CalibrationConfig {
    bool        enabled{false};
    std::string file;           ///< Path to OpenCV-format calibration YAML
};

struct CameraConfig {
    uint32_t         id;
    std::string      name;
    std::string      device;          ///< e.g. /dev/video0
    uint32_t         fps{30};
    uint32_t         width{1920};
    uint32_t         height{1080};
    std::string      pixel_format{"UYVY"};
    Priority         priority{Priority::NORMAL};
    CalibrationConfig calibration;
};

// ---------------------------------------------------------------------------
// Frame-drop policy
// ---------------------------------------------------------------------------

struct FrameDropPolicy {
    std::vector<uint32_t> high_priority_cameras;  ///< Always full FPS
    uint32_t              normal_priority_min_fps{10};
    uint32_t              normal_priority_max_fps{20};
};

// ---------------------------------------------------------------------------
// Top-level config container
// ---------------------------------------------------------------------------

struct CameraManagerConfig {
    std::vector<CameraConfig> cameras;
    FrameDropPolicy           frame_drop_policy;
};

/// Load and validate cameras.yaml.
/// Throws std::runtime_error on parse failure or missing mandatory fields.
CameraManagerConfig load_camera_config(const std::string& yaml_path);

} // namespace autodriver::camera
