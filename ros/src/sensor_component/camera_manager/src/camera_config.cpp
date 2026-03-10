#include "camera_manager/camera_config.hpp"

#include <yaml-cpp/yaml.h>
#include <stdexcept>

namespace autodriver::camera {

static Priority parse_priority(const std::string& s)
{
    if (s == "HIGH") return Priority::HIGH;
    return Priority::NORMAL;
}

CameraManagerConfig load_camera_config(const std::string& yaml_path)
{
    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error(std::string("Failed to load cameras.yaml: ") + e.what());
    }

    CameraManagerConfig cfg;

    // ── Parse camera list ─────────────────────────────────────────────────
    for (const auto& node : root["cameras"]) {
        CameraConfig cam;
        cam.id     = node["id"].as<uint32_t>();
        cam.name   = node["name"].as<std::string>();
        cam.device = node["device"].as<std::string>();

        if (node["fps"])        cam.fps    = node["fps"].as<uint32_t>();
        if (node["resolution"]) {
            cam.width  = node["resolution"][0].as<uint32_t>();
            cam.height = node["resolution"][1].as<uint32_t>();
        }
        if (node["pixel_format"])
            cam.pixel_format = node["pixel_format"].as<std::string>();
        if (node["priority"])
            cam.priority = parse_priority(node["priority"].as<std::string>());

        if (node["calibration"]) {
            cam.calibration.enabled = node["calibration"]["enabled"].as<bool>(false);
            if (cam.calibration.enabled)
                cam.calibration.file = node["calibration"]["file"].as<std::string>();
        }

        cfg.cameras.push_back(cam);
    }

    // ── Parse frame drop policy ───────────────────────────────────────────
    if (root["frame_drop_policy"]) {
        auto& policy_node = root["frame_drop_policy"];
        if (policy_node["high_priority_cameras"])
            cfg.frame_drop_policy.high_priority_cameras =
                policy_node["high_priority_cameras"].as<std::vector<uint32_t>>();
        if (policy_node["normal_priority_min_fps"])
            cfg.frame_drop_policy.normal_priority_min_fps =
                policy_node["normal_priority_min_fps"].as<uint32_t>();
        if (policy_node["normal_priority_max_fps"])
            cfg.frame_drop_policy.normal_priority_max_fps =
                policy_node["normal_priority_max_fps"].as<uint32_t>();
    }

    return cfg;
}

} // namespace autodriver::camera
