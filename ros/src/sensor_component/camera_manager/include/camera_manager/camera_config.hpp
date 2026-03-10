#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace autodriver::camera {

// ---------------------------------------------------------------------------
// Priority — controls frame-drop policy for normal-priority cameras.
// ---------------------------------------------------------------------------
enum class Priority : uint8_t { kHigh, kNormal };

// ---------------------------------------------------------------------------
// CalibrationConfig — per-camera lens calibration (optional).
// File path is resolved relative to cameras.yaml by load_camera_config().
// ---------------------------------------------------------------------------
struct CalibrationConfig {
  bool        enabled{false};
  std::string file;  ///< Absolute path after resolution (empty if disabled)
};

// ---------------------------------------------------------------------------
// CameraSpec — per-camera hardware and pipeline configuration.
//
// `sensor_id` maps to nvarguscamerasrc sensor-id= property (CSI camera index).
// `gst_source` overrides sensor_id if non-empty, allowing any GStreamer source
//   element (e.g. "v4l2src device=/dev/video0" for USB cameras in simulation).
// ---------------------------------------------------------------------------
struct CameraSpec {
  uint32_t          id;
  std::string       name;
  uint32_t          sensor_id{0};   ///< nvarguscamerasrc sensor-id (default path)
  std::string       gst_source;     ///< Custom source override (optional)
  uint32_t          fps{30};
  uint32_t          width{1920};
  uint32_t          height{1080};
  Priority          priority{Priority::kNormal};
  CalibrationConfig calibration;
};

// ---------------------------------------------------------------------------
// FrameDropPolicy — rate-limits normal-priority cameras to save IPC bandwidth.
// High-priority cameras are never dropped.
// ---------------------------------------------------------------------------
struct FrameDropPolicy {
  std::vector<uint32_t> high_priority_ids;  ///< camera_id list — always full FPS
  uint32_t              min_fps{10};         ///< Floor FPS for normal cameras
  uint32_t              max_fps{20};         ///< Ceiling FPS for normal cameras
};

// ---------------------------------------------------------------------------
// CameraManagerConfig — top-level config loaded from cameras.yaml.
// ---------------------------------------------------------------------------
struct CameraManagerConfig {
  std::vector<CameraSpec> cameras;
  FrameDropPolicy         frame_drop_policy;
};

// ---------------------------------------------------------------------------
// LoadCameraConfig
//
// Parses `yaml_path` and returns a validated CameraManagerConfig.
// Calibration file paths in the YAML are treated as relative to yaml_path's
// parent directory and resolved to absolute paths.
//
// Returns std::nullopt on parse or validation failure; places a description
// in `*error_msg` if non-null.
// ---------------------------------------------------------------------------
#include <optional>
[[nodiscard]] std::optional<CameraManagerConfig> LoadCameraConfig(
    const std::string& yaml_path,
    std::string* error_msg = nullptr) noexcept;

}  // namespace autodriver::camera
