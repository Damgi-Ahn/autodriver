#include "camera_manager/camera_config.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <optional>
#include <string>

namespace autodriver::camera {

namespace {

Priority ParsePriority(const std::string& s) noexcept {
  if (s == "HIGH" || s == "high") return Priority::kHigh;
  return Priority::kNormal;
}

}  // namespace

std::optional<CameraManagerConfig> LoadCameraConfig(
    const std::string& yaml_path,
    std::string* error_msg) noexcept {
  const std::filesystem::path yaml_dir =
      std::filesystem::path(yaml_path).parent_path();

  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception& e) {
    if (error_msg) *error_msg = std::string("YAML parse error: ") + e.what();
    return std::nullopt;
  }

  CameraManagerConfig cfg;

  // ── Camera list ───────────────────────────────────────────────────────────
  const auto& cameras_node = root["cameras"];
  if (!cameras_node || !cameras_node.IsSequence()) {
    if (error_msg) *error_msg = "Missing or invalid 'cameras' key";
    return std::nullopt;
  }

  for (const auto& node : cameras_node) {
    CameraSpec cam;

    if (!node["id"] || !node["name"]) {
      if (error_msg) *error_msg = "Camera entry missing 'id' or 'name'";
      return std::nullopt;
    }

    cam.id   = node["id"].as<uint32_t>();
    cam.name = node["name"].as<std::string>();

    // Source: prefer explicit gst_source, otherwise build from sensor_id.
    if (node["gst_source"]) {
      cam.gst_source = node["gst_source"].as<std::string>();
    } else if (node["sensor_id"]) {
      cam.sensor_id = node["sensor_id"].as<uint32_t>();
    }

    if (node["fps"])    cam.fps    = node["fps"].as<uint32_t>();
    if (node["width"])  cam.width  = node["width"].as<uint32_t>();
    if (node["height"]) cam.height = node["height"].as<uint32_t>();

    // Legacy "resolution" key for backwards compatibility.
    if (node["resolution"] && node["resolution"].IsSequence() &&
        node["resolution"].size() == 2) {
      cam.width  = node["resolution"][0].as<uint32_t>();
      cam.height = node["resolution"][1].as<uint32_t>();
    }

    if (node["priority"])
      cam.priority = ParsePriority(node["priority"].as<std::string>());

    // Calibration: resolve relative paths against yaml_dir.
    if (node["calibration"]) {
      cam.calibration.enabled =
          node["calibration"]["enabled"].as<bool>(false);
      if (cam.calibration.enabled && node["calibration"]["file"]) {
        const std::filesystem::path rel_file =
            node["calibration"]["file"].as<std::string>();
        cam.calibration.file =
            rel_file.is_absolute()
                ? rel_file.string()
                : (yaml_dir / rel_file).lexically_normal().string();
      }
    }

    cfg.cameras.push_back(std::move(cam));
  }

  if (cfg.cameras.empty()) {
    if (error_msg) *error_msg = "cameras list is empty";
    return std::nullopt;
  }

  // ── Frame drop policy ─────────────────────────────────────────────────────
  if (root["frame_drop_policy"]) {
    const auto& p = root["frame_drop_policy"];
    if (p["high_priority_cameras"])
      cfg.frame_drop_policy.high_priority_ids =
          p["high_priority_cameras"].as<std::vector<uint32_t>>();
    if (p["normal_priority_min_fps"])
      cfg.frame_drop_policy.min_fps = p["normal_priority_min_fps"].as<uint32_t>();
    if (p["normal_priority_max_fps"])
      cfg.frame_drop_policy.max_fps = p["normal_priority_max_fps"].as<uint32_t>();
  }

  return cfg;
}

}  // namespace autodriver::camera
