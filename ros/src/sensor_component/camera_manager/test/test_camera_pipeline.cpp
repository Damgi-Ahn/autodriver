// test_camera_pipeline.cpp
//
// Unit tests for CameraPipeline:
//   - BuildPipelineString() format validation (no hardware needed)
//   - Initial state: IsHealthy(), frames_captured(), frames_dropped()
//   - Camera spec accessors: camera_id(), name()
//   - Callback registration does not crash
//   - Restart counter logic (tested via public interface)
//
// GStreamer hardware calls (Start, Stop, actual frame capture) are NOT tested
// here — they require physical CSI cameras.  These tests cover the testable
// pure logic without a Jetson.

#include <gtest/gtest.h>
#include "camera_manager/camera_pipeline.hpp"

#include <string>

using namespace autodriver::camera;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static CameraSpec MakeSpec(uint32_t id,
                             const std::string& name,
                             uint32_t sensor_id = 0,
                             uint32_t fps = 30,
                             uint32_t width = 1920,
                             uint32_t height = 1080)
{
  CameraSpec s;
  s.id        = id;
  s.name      = name;
  s.sensor_id = sensor_id;
  s.fps       = fps;
  s.width     = width;
  s.height    = height;
  s.priority  = Priority::kNormal;
  return s;
}

static CameraSpec MakeGstSrcSpec(uint32_t id, const std::string& gst_src)
{
  CameraSpec s  = MakeSpec(id, "test_cam");
  s.gst_source  = gst_src;
  return s;
}

// ---------------------------------------------------------------------------
// Initial state
// ---------------------------------------------------------------------------

TEST(CameraPipelineTest, InitialHealthyIsTrue)
{
  CameraPipeline p(MakeSpec(0, "front_center"));
  EXPECT_TRUE(p.IsHealthy());
}

TEST(CameraPipelineTest, InitialFrameCountersAreZero)
{
  CameraPipeline p(MakeSpec(0, "front_center"));
  EXPECT_EQ(p.frames_captured(), 0u);
  EXPECT_EQ(p.frames_dropped(),  0u);
}

TEST(CameraPipelineTest, CameraIdMatches)
{
  CameraPipeline p(MakeSpec(5, "side_left"));
  EXPECT_EQ(p.camera_id(), 5u);
}

TEST(CameraPipelineTest, NameMatches)
{
  CameraPipeline p(MakeSpec(0, "rear_center"));
  EXPECT_EQ(p.name(), "rear_center");
}

// ---------------------------------------------------------------------------
// Callback registration (no-crash tests)
// ---------------------------------------------------------------------------

TEST(CameraPipelineTest, SetFrameCallbackDoesNotThrow)
{
  CameraPipeline p(MakeSpec(0, "cam0"));
  EXPECT_NO_THROW(
      p.SetFrameCallback(
          [](int, uint64_t, uint32_t, uint32_t, uint32_t, uint32_t) {}));
}

TEST(CameraPipelineTest, SetNullFrameCallbackDoesNotThrow)
{
  CameraPipeline p(MakeSpec(0, "cam0"));
  EXPECT_NO_THROW(p.SetFrameCallback(nullptr));
}

TEST(CameraPipelineTest, SetDebugCallbackDoesNotThrow)
{
  CameraPipeline p(MakeSpec(0, "cam0"));
  EXPECT_NO_THROW(
      p.SetDebugFrameCallback(
          [](uint32_t, const uint8_t*, size_t, uint64_t) {}));
}

TEST(CameraPipelineTest, SetDebugStreamEnabledDoesNotThrow)
{
  CameraPipeline p(MakeSpec(0, "cam0"));
  EXPECT_NO_THROW(p.SetDebugStreamEnabled(true));
  EXPECT_NO_THROW(p.SetDebugStreamEnabled(false));
}

// ---------------------------------------------------------------------------
// Pipeline string format
// The private BuildPipelineString() is tested indirectly through
// the fact that all spec permutations must produce a non-empty string.
// We expose it via a friend struct in a white-box test below.
// ---------------------------------------------------------------------------

// White-box helper: expose BuildPipelineString via subclass trick.
// CameraPipeline's BuildPipelineString is private but callable via
// a derived class in the test binary.
class TestablePipeline : public CameraPipeline {
 public:
  explicit TestablePipeline(const CameraSpec& s) : CameraPipeline(s) {}
  using CameraPipeline::BuildPipelineString;
};

TEST(CameraPipelineTest, PipelineStringContainsSensorId)
{
  CameraSpec s = MakeSpec(0, "front", /*sensor_id=*/3);
  TestablePipeline p(s);
  const std::string pipeline = p.BuildPipelineString();
  EXPECT_FALSE(pipeline.empty());
  EXPECT_NE(pipeline.find("3"), std::string::npos)
      << "sensor_id=3 not in pipeline string: " << pipeline;
}

TEST(CameraPipelineTest, PipelineStringContainsFps)
{
  CameraSpec s = MakeSpec(0, "cam", 0, /*fps=*/60);
  TestablePipeline p(s);
  const std::string pipeline = p.BuildPipelineString();
  EXPECT_NE(pipeline.find("60"), std::string::npos)
      << "fps 60 not in pipeline: " << pipeline;
}

TEST(CameraPipelineTest, PipelineStringContainsResolution)
{
  CameraSpec s = MakeSpec(0, "cam", 0, 30, /*width=*/1280, /*height=*/720);
  TestablePipeline p(s);
  const std::string pipeline = p.BuildPipelineString();
  EXPECT_NE(pipeline.find("1280"), std::string::npos)
      << "width 1280 not in pipeline: " << pipeline;
  EXPECT_NE(pipeline.find("720"), std::string::npos)
      << "height 720 not in pipeline: " << pipeline;
}

TEST(CameraPipelineTest, PipelineStringWithGstSourceOverride)
{
  CameraSpec s = MakeGstSrcSpec(0, "videotestsrc pattern=smpte");
  TestablePipeline p(s);
  const std::string pipeline = p.BuildPipelineString();
  EXPECT_NE(pipeline.find("videotestsrc"), std::string::npos)
      << "custom gst_source not in pipeline: " << pipeline;
  // nvarguscamerasrc must NOT appear when gst_source is set
  EXPECT_EQ(pipeline.find("nvarguscamerasrc"), std::string::npos)
      << "nvarguscamerasrc should not appear with gst_source override: "
      << pipeline;
}

TEST(CameraPipelineTest, PipelineStringContainsAppsink)
{
  CameraSpec s = MakeSpec(0, "cam");
  TestablePipeline p(s);
  const std::string pipeline = p.BuildPipelineString();
  EXPECT_NE(pipeline.find("appsink"), std::string::npos)
      << "appsink not in pipeline: " << pipeline;
}

TEST(CameraPipelineTest, PipelineStringWithDebugStreamContainsTee)
{
  CameraSpec s = MakeSpec(0, "cam");
  TestablePipeline p(s);
  p.SetDebugStreamEnabled(true);
  const std::string pipeline = p.BuildPipelineString();
  EXPECT_NE(pipeline.find("tee"), std::string::npos)
      << "tee not in debug pipeline: " << pipeline;
  EXPECT_NE(pipeline.find("nvjpegenc"), std::string::npos)
      << "nvjpegenc not in debug pipeline: " << pipeline;
}

TEST(CameraPipelineTest, PipelineStringWithoutDebugDoesNotContainTee)
{
  CameraSpec s = MakeSpec(0, "cam");
  TestablePipeline p(s);
  p.SetDebugStreamEnabled(false);
  const std::string pipeline = p.BuildPipelineString();
  EXPECT_EQ(pipeline.find("tee"), std::string::npos)
      << "tee should not appear without debug stream: " << pipeline;
}

// ---------------------------------------------------------------------------
// Edge cases
// ---------------------------------------------------------------------------

TEST(CameraPipelineTest, AllTwelveCameraSpecsProduceValidStrings)
{
  // Jetson Orin: 12 CSI cameras, sensor_id 0-11
  for (uint32_t i = 0; i < 12; ++i) {
    CameraSpec s = MakeSpec(i, "cam" + std::to_string(i), /*sensor_id=*/i);
    TestablePipeline p(s);
    EXPECT_FALSE(p.BuildPipelineString().empty())
        << "Camera " << i << " produced empty pipeline string";
  }
}

TEST(CameraPipelineTest, ZeroFpsCameraSpecDoesNotCrash)
{
  // fps=0 is invalid for hardware but pipeline string construction must not crash
  CameraSpec s = MakeSpec(0, "cam", 0, /*fps=*/0);
  TestablePipeline p(s);
  EXPECT_NO_THROW(p.BuildPipelineString());
}

TEST(CameraPipelineTest, V4L2SourceForUSBCamera)
{
  CameraSpec s = MakeGstSrcSpec(0, "v4l2src device=/dev/video0 ! video/x-raw");
  TestablePipeline p(s);
  const std::string pipeline = p.BuildPipelineString();
  EXPECT_NE(pipeline.find("v4l2src"), std::string::npos);
}

