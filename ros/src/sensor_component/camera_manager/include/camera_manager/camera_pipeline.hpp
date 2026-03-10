#pragma once

#include "camera_config.hpp"
#include <ipc_unix_socket/ipc_unix_socket.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <atomic>
#include <functional>
#include <string>
#include <thread>

namespace autodriver::camera {

// ---------------------------------------------------------------------------
// FrameCallback — called from GStreamer appsink thread on every captured frame.
//
// Parameters match ipc::FrameMeta so the caller can forward directly.
// `dmabuf_fd` ownership stays with CameraPipeline — do NOT close it.
// The callback runs inline; keep it fast (typically one SendFd call).
// ---------------------------------------------------------------------------
using FrameCallback = std::function<void(
    int     dmabuf_fd,
    uint64_t timestamp,
    uint32_t camera_id,
    uint32_t width,
    uint32_t height,
    uint32_t format)>;

// ---------------------------------------------------------------------------
// DebugFrameCallback — called from the debug appsink thread with JPEG data.
// `data` is valid only for the duration of the callback.
// ---------------------------------------------------------------------------
using DebugFrameCallback = std::function<void(
    uint32_t       camera_id,
    const uint8_t* data,
    size_t         size,
    uint64_t       timestamp)>;

// ---------------------------------------------------------------------------
// CameraPipeline
//
// Manages one GStreamer pipeline for one CSI/USB camera on Jetson Orin.
//
// Pipeline topology (debug stream disabled):
//   nvarguscamerasrc sensor-id=N  [or custom gst_source]
//     → capsfilter (NVMM NV12 @ WxH fps/1)
//     → appsink name=inference_sink  ← FrameCallback
//
// Pipeline topology (debug stream enabled):
//   nvarguscamerasrc sensor-id=N
//     → capsfilter (NVMM NV12)
//     → tee name=t
//   t. → queue(2, leaky=downstream) → appsink name=inference_sink
//   t. → queue(1, leaky=downstream) → nvjpegenc → appsink name=debug_sink
//
// Lifecycle (not thread-safe):
//   pipeline.Start()   // spawns pipeline_thread_ + optional bus_thread_
//   ...                // FrameCallback invoked from streaming thread
//   pipeline.Stop()    // EOS → joins threads → null state
//
// Automatic restart (from bus_thread_):
//   On GST_MESSAGE_ERROR: rebuild pipeline and restart up to kMaxRestarts.
//   If all attempts exhausted: marks healthy=false, stops.
// ---------------------------------------------------------------------------
class CameraPipeline {
 public:
  static constexpr int kMaxRestarts    = 5;
  static constexpr int kRestartDelayMs = 500;

  explicit CameraPipeline(const CameraSpec& spec);
  ~CameraPipeline();

  CameraPipeline(const CameraPipeline&)            = delete;
  CameraPipeline& operator=(const CameraPipeline&) = delete;

  void SetFrameCallback(FrameCallback cb);
  void SetDebugFrameCallback(DebugFrameCallback cb);
  void SetDebugStreamEnabled(bool enabled);

  /// Build GStreamer pipeline and start streaming.
  /// Returns false if the pipeline cannot be created.
  [[nodiscard]] bool Start() noexcept;

  /// Send EOS and wait for all threads to exit.
  void Stop() noexcept;

  [[nodiscard]] bool     IsHealthy()      const noexcept;
  [[nodiscard]] uint32_t camera_id()      const noexcept { return spec_.id; }
  [[nodiscard]] const std::string& name() const noexcept { return spec_.name; }

  [[nodiscard]] uint64_t frames_captured() const noexcept;
  [[nodiscard]] uint64_t frames_dropped()  const noexcept;

 private:
  // Pipeline construction
  [[nodiscard]] bool        BuildPipeline()  noexcept;
  void                      DestroyPipeline() noexcept;
  [[nodiscard]] std::string BuildPipelineString() const noexcept;

  // Bus monitoring thread — handles ERROR / EOS / WARNING messages.
  void RunBusLoop() noexcept;

  // appsink callbacks — called from GStreamer streaming thread.
  static GstFlowReturn OnInferenceSample(GstAppSink* sink,
                                          gpointer   user_data) noexcept;
  GstFlowReturn HandleInferenceSample(GstAppSink* sink) noexcept;

  static GstFlowReturn OnDebugSample(GstAppSink* sink,
                                      gpointer   user_data) noexcept;
  GstFlowReturn HandleDebugSample(GstAppSink* sink) noexcept;

  // Restart (called from bus thread on error).
  [[nodiscard]] bool Restart() noexcept;

  // ── Data members ──────────────────────────────────────────────────────────
  CameraSpec           spec_;
  FrameCallback        frame_callback_;
  DebugFrameCallback   debug_frame_callback_;
  bool                 debug_stream_enabled_{false};

  GstElement* pipeline_{nullptr};
  GstElement* inference_appsink_{nullptr};
  GstElement* debug_appsink_{nullptr};

  std::thread      bus_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> healthy_{true};
  int               restart_count_{0};

  std::atomic<uint64_t> frames_captured_{0};
  std::atomic<uint64_t> frames_dropped_{0};
};

}  // namespace autodriver::camera
