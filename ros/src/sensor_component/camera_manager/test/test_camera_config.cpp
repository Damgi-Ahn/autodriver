// test_camera_config.cpp
//
// Unit tests for LoadCameraConfig() and CameraSpec/FrameDropPolicy structs.
// Tests run without hardware — only YAML parsing and validation logic.
//
// Test categories:
//   Happy path    — valid cameras.yaml parses correctly
//   Camera fields — id, name, priority, fps, resolution, calibration
//   FrameDrop     — high_priority_ids, min/max fps
//   Error paths   — missing keys, malformed YAML, empty cameras list
//   Edge cases    — gst_source override, legacy resolution key, duplicate ids

#include <gtest/gtest.h>
#include "camera_manager/camera_config.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

using namespace autodriver::camera;

// ---------------------------------------------------------------------------
// YAML-writing helper — writes a temp file, auto-deletes on scope exit.
// ---------------------------------------------------------------------------
class TempYaml {
 public:
  explicit TempYaml(const std::string& content) {
    char tmpl[] = "/tmp/test_cam_XXXXXX.yaml";
    int fd = mkstemps(tmpl, 5);
    if (fd < 0) throw std::runtime_error("mkstemps failed");
    path_ = tmpl;
    ::write(fd, content.c_str(), content.size());
    ::close(fd);
  }
  ~TempYaml() { std::filesystem::remove(path_); }
  const std::string& path() const { return path_; }

 private:
  std::string path_;
};

// ---------------------------------------------------------------------------
// Happy path
// ---------------------------------------------------------------------------

TEST(CameraConfigTest, ParsesValidYaml)
{
  TempYaml yaml(R"(
cameras:
  - id: 0
    name: front_center
    sensor_id: 0
    fps: 30
    width: 1920
    height: 1080
    priority: HIGH
  - id: 1
    name: rear_center
    sensor_id: 1
    fps: 30
    width: 1920
    height: 1080
    priority: NORMAL
frame_drop_policy:
  high_priority_cameras: [0]
  normal_priority_min_fps: 10
  normal_priority_max_fps: 20
)");

  std::string err;
  auto cfg = LoadCameraConfig(yaml.path(), &err);
  ASSERT_TRUE(cfg.has_value()) << "Parse failed: " << err;
  EXPECT_EQ(cfg->cameras.size(), 2u);
  EXPECT_EQ(cfg->cameras[0].id,   0u);
  EXPECT_EQ(cfg->cameras[0].name, "front_center");
  EXPECT_EQ(cfg->cameras[1].id,   1u);
}

// ---------------------------------------------------------------------------
// Camera field parsing
// ---------------------------------------------------------------------------

TEST(CameraConfigTest, PriorityHighParsed)
{
  TempYaml yaml(R"(
cameras:
  - id: 0
    name: cam0
    sensor_id: 0
    priority: HIGH
)");
  auto cfg = LoadCameraConfig(yaml.path());
  ASSERT_TRUE(cfg.has_value());
  EXPECT_EQ(cfg->cameras[0].priority, Priority::kHigh);
}

TEST(CameraConfigTest, PriorityNormalDefault)
{
  TempYaml yaml(R"(
cameras:
  - id: 0
    name: cam0
    sensor_id: 0
    priority: NORMAL
)");
  auto cfg = LoadCameraConfig(yaml.path());
  ASSERT_TRUE(cfg.has_value());
  EXPECT_EQ(cfg->cameras[0].priority, Priority::kNormal);
}

TEST(CameraConfigTest, PriorityMissingDefaultsToNormal)
{
  TempYaml yaml(R"(
cameras:
  - id: 0
    name: cam0
    sensor_id: 0
)");
  auto cfg = LoadCameraConfig(yaml.path());
  ASSERT_TRUE(cfg.has_value());
  EXPECT_EQ(cfg->cameras[0].priority, Priority::kNormal);
}

TEST(CameraConfigTest, ResolutionDefaultValues)
{
  TempYaml yaml(R"(
cameras:
  - id: 0
    name: cam0
    sensor_id: 0
)");
  auto cfg = LoadCameraConfig(yaml.path());
  ASSERT_TRUE(cfg.has_value());
  // Default values from struct
  EXPECT_EQ(cfg->cameras[0].width,  1920u);
  EXPECT_EQ(cfg->cameras[0].height, 1080u);
  EXPECT_EQ(cfg->cameras[0].fps,    30u);
}

TEST(CameraConfigTest, CustomResolutionAndFps)
{
  TempYaml yaml(R"(
cameras:
  - id: 0
    name: cam0
    sensor_id: 0
    fps: 60
    width: 3840
    height: 2160
)");
  auto cfg = LoadCameraConfig(yaml.path());
  ASSERT_TRUE(cfg.has_value());
  EXPECT_EQ(cfg->cameras[0].fps,    60u);
  EXPECT_EQ(cfg->cameras[0].width,  3840u);
  EXPECT_EQ(cfg->cameras[0].height, 2160u);
}

TEST(CameraConfigTest, LegacyResolutionKey)
{
  // Backward-compatibility: resolution: [W, H] array
  TempYaml yaml(R"(
cameras:
  - id: 0
    name: cam0
    sensor_id: 0
    resolution: [640, 480]
)");
  auto cfg = LoadCameraConfig(yaml.path());
  ASSERT_TRUE(cfg.has_value());
  EXPECT_EQ(cfg->cameras[0].width,  640u);
  EXPECT_EQ(cfg->cameras[0].height, 480u);
}

TEST(CameraConfigTest, GstSourceOverride)
{
  TempYaml yaml(R"(
cameras:
  - id: 0
    name: usb_cam
    gst_source: "v4l2src device=/dev/video0"
)");
  auto cfg = LoadCameraConfig(yaml.path());
  ASSERT_TRUE(cfg.has_value());
  EXPECT_EQ(cfg->cameras[0].gst_source, "v4l2src device=/dev/video0");
}

TEST(CameraConfigTest, CalibrationEnabledWithRelativePath)
{
  TempYaml yaml(R"(
cameras:
  - id: 0
    name: cam0
    sensor_id: 0
    calibration:
      enabled: true
      file: calibration/cam0.yaml
)");
  auto cfg = LoadCameraConfig(yaml.path());
  ASSERT_TRUE(cfg.has_value());
  EXPECT_TRUE(cfg->cameras[0].calibration.enabled);
  // Resolved path must be absolute
  EXPECT_TRUE(std::filesystem::path(cfg->cameras[0].calibration.file).is_absolute());
  // Must contain the relative part
  EXPECT_NE(cfg->cameras[0].calibration.file.find("cam0.yaml"), std::string::npos);
}

TEST(CameraConfigTest, CalibrationDisabled)
{
  TempYaml yaml(R"(
cameras:
  - id: 0
    name: cam0
    sensor_id: 0
    calibration:
      enabled: false
)");
  auto cfg = LoadCameraConfig(yaml.path());
  ASSERT_TRUE(cfg.has_value());
  EXPECT_FALSE(cfg->cameras[0].calibration.enabled);
  EXPECT_TRUE(cfg->cameras[0].calibration.file.empty());
}

// ---------------------------------------------------------------------------
// FrameDropPolicy
// ---------------------------------------------------------------------------

TEST(CameraConfigTest, FrameDropPolicyParsed)
{
  TempYaml yaml(R"(
cameras:
  - id: 0
    name: cam0
    sensor_id: 0
frame_drop_policy:
  high_priority_cameras: [0, 1, 2, 3]
  normal_priority_min_fps: 10
  normal_priority_max_fps: 20
)");
  auto cfg = LoadCameraConfig(yaml.path());
  ASSERT_TRUE(cfg.has_value());
  const auto& p = cfg->frame_drop_policy;
  ASSERT_EQ(p.high_priority_ids.size(), 4u);
  EXPECT_EQ(p.high_priority_ids[0], 0u);
  EXPECT_EQ(p.high_priority_ids[3], 3u);
  EXPECT_EQ(p.min_fps, 10u);
  EXPECT_EQ(p.max_fps, 20u);
}

TEST(CameraConfigTest, FrameDropPolicyMissingUsesDefaults)
{
  TempYaml yaml(R"(
cameras:
  - id: 0
    name: cam0
    sensor_id: 0
)");
  auto cfg = LoadCameraConfig(yaml.path());
  ASSERT_TRUE(cfg.has_value());
  // No frame_drop_policy block → struct defaults
  EXPECT_TRUE(cfg->frame_drop_policy.high_priority_ids.empty());
  EXPECT_EQ(cfg->frame_drop_policy.min_fps, 10u);
  EXPECT_EQ(cfg->frame_drop_policy.max_fps, 20u);
}

// ---------------------------------------------------------------------------
// Error paths
// ---------------------------------------------------------------------------

TEST(CameraConfigTest, MissingFileReturnsNullopt)
{
  std::string err;
  auto cfg = LoadCameraConfig("/nonexistent/path.yaml", &err);
  EXPECT_FALSE(cfg.has_value());
  EXPECT_FALSE(err.empty());
}

TEST(CameraConfigTest, MalformedYamlReturnsNullopt)
{
  TempYaml yaml(R"(
cameras: [this: is: not: valid: yaml
)");
  std::string err;
  auto cfg = LoadCameraConfig(yaml.path(), &err);
  EXPECT_FALSE(cfg.has_value());
}

TEST(CameraConfigTest, EmptyCamerasListReturnsNullopt)
{
  TempYaml yaml(R"(
cameras: []
)");
  std::string err;
  auto cfg = LoadCameraConfig(yaml.path(), &err);
  EXPECT_FALSE(cfg.has_value());
  EXPECT_FALSE(err.empty());
}

TEST(CameraConfigTest, CamerasMissingIdReturnsNullopt)
{
  TempYaml yaml(R"(
cameras:
  - name: cam_no_id
    sensor_id: 0
)");
  std::string err;
  auto cfg = LoadCameraConfig(yaml.path(), &err);
  EXPECT_FALSE(cfg.has_value());
}

TEST(CameraConfigTest, CamerasMissingNameReturnsNullopt)
{
  TempYaml yaml(R"(
cameras:
  - id: 0
    sensor_id: 0
)");
  std::string err;
  auto cfg = LoadCameraConfig(yaml.path(), &err);
  EXPECT_FALSE(cfg.has_value());
}

TEST(CameraConfigTest, NoCamerasKeyReturnsNullopt)
{
  TempYaml yaml(R"(
not_cameras:
  - id: 0
    name: cam0
)");
  std::string err;
  auto cfg = LoadCameraConfig(yaml.path(), &err);
  EXPECT_FALSE(cfg.has_value());
}

// ---------------------------------------------------------------------------
// Autonomous driving edge cases
// ---------------------------------------------------------------------------

TEST(CameraConfigTest, TwelveCamerasMaxLoad)
{
  // Jetson Orin supports up to 12 CSI cameras
  std::string content = "cameras:\n";
  for (int i = 0; i < 12; ++i) {
    content += "  - id: " + std::to_string(i) + "\n";
    content += "    name: cam" + std::to_string(i) + "\n";
    content += "    sensor_id: " + std::to_string(i) + "\n";
    content += "    priority: " + std::string(i < 4 ? "HIGH" : "NORMAL") + "\n";
  }
  TempYaml yaml(content);
  auto cfg = LoadCameraConfig(yaml.path());
  ASSERT_TRUE(cfg.has_value());
  EXPECT_EQ(cfg->cameras.size(), 12u);
  EXPECT_EQ(cfg->cameras[0].priority,  Priority::kHigh);
  EXPECT_EQ(cfg->cameras[4].priority,  Priority::kNormal);
  EXPECT_EQ(cfg->cameras[11].id,       11u);
}

TEST(CameraConfigTest, VideoTestSrcForSimulation)
{
  TempYaml yaml(R"(
cameras:
  - id: 0
    name: sim_front
    gst_source: "videotestsrc pattern=smpte is-live=true"
    fps: 30
    width: 1920
    height: 1080
)");
  auto cfg = LoadCameraConfig(yaml.path());
  ASSERT_TRUE(cfg.has_value());
  EXPECT_NE(cfg->cameras[0].gst_source.find("videotestsrc"), std::string::npos);
}

TEST(CameraConfigTest, AllHighPriorityNormalMinFpsHigherThanMax)
{
  // min_fps > max_fps is a configuration error we tolerate (node handles it)
  TempYaml yaml(R"(
cameras:
  - id: 0
    name: cam0
    sensor_id: 0
frame_drop_policy:
  high_priority_cameras: [0]
  normal_priority_min_fps: 30
  normal_priority_max_fps: 10
)");
  // LoadCameraConfig should not reject this — semantic validation is node's job
  auto cfg = LoadCameraConfig(yaml.path());
  ASSERT_TRUE(cfg.has_value());
  EXPECT_EQ(cfg->frame_drop_policy.min_fps, 30u);
  EXPECT_EQ(cfg->frame_drop_policy.max_fps, 10u);
}

TEST(CameraConfigTest, NullErrorMsgDoesNotCrash)
{
  // Pass nullptr for error_msg — should not segfault
  auto cfg = LoadCameraConfig("/nonexistent.yaml", nullptr);
  EXPECT_FALSE(cfg.has_value());
}

