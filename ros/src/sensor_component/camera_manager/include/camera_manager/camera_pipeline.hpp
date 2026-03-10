#pragma once

#include "camera_config.hpp"

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>

struct NvBufSurface;  // forward-declare to avoid pulling in full nvbufsurface.h

namespace autodriver::camera {

// ---------------------------------------------------------------------------
// Callback type invoked from the appsink new-sample handler.
// `dmabuf_fd`  : exported DMABUF fd (caller must close after use)
// `meta`       : filled frame metadata
// ---------------------------------------------------------------------------
using FrameCallback = std::function<void(int dmabuf_fd,
                                         uint64_t timestamp,
                                         uint32_t camera_id,
                                         uint32_t width,
                                         uint32_t height,
                                         uint32_t format)>;

// ---------------------------------------------------------------------------
// Single-camera GStreamer NVMM pipeline.
//
// Lifecycle:
//   CameraPipeline p{config};
//   p.set_frame_callback(cb);
//   p.start();           // spawns GStreamer thread
//   ...
//   p.stop();            // signals EOS and joins thread
//
// Automatic restart:
//   On pipeline error the thread calls restart() internally up to
//   max_restart_attempts times before giving up.
// ---------------------------------------------------------------------------
class CameraPipeline {
public:
    static constexpr int kMaxRestartAttempts = 5;
    static constexpr int kRestartDelayMs     = 500;

    explicit CameraPipeline(const CameraConfig& config);
    ~CameraPipeline();

    // Non-copyable, non-movable (owns GStreamer resources)
    CameraPipeline(const CameraPipeline&)            = delete;
    CameraPipeline& operator=(const CameraPipeline&) = delete;

    void set_frame_callback(FrameCallback cb);

    /// Start the GStreamer pipeline in a dedicated thread.
    bool start();

    /// Signal EOS, wait for thread to join. Safe to call from any thread.
    void stop();

    /// Returns false if the pipeline has failed permanently.
    bool is_healthy() const { return healthy_.load(std::memory_order_relaxed); }

    uint32_t camera_id() const { return config_.id; }
    const std::string& name() const { return config_.name; }

    // Stats (updated from appsink callback, read from metrics thread)
    uint64_t frames_captured() const { return frames_captured_.load(); }
    uint64_t frames_dropped()  const { return frames_dropped_.load();  }

private:
    // GStreamer pipeline construction
    bool build_pipeline();
    void destroy_pipeline();
    void run_loop();           ///< Executed inside pipeline_thread_

    // appsink callbacks (inference branch)
    static GstFlowReturn on_inference_sample(GstAppSink* sink, gpointer user_data);
    GstFlowReturn handle_inference_sample(GstAppSink* sink);

    // appsink callbacks (debug branch — NVJPEG compressed image)
    static GstFlowReturn on_debug_sample(GstAppSink* sink, gpointer user_data);
    GstFlowReturn handle_debug_sample(GstAppSink* sink);

    // Restart logic
    bool restart();

    CameraConfig    config_;
    FrameCallback   frame_callback_;

    GstElement*     pipeline_{nullptr};
    GstElement*     inference_appsink_{nullptr};
    GstElement*     debug_appsink_{nullptr};

    std::thread          pipeline_thread_;
    std::atomic<bool>    running_{false};
    std::atomic<bool>    healthy_{true};
    int                  restart_count_{0};

    std::atomic<uint64_t> frames_captured_{0};
    std::atomic<uint64_t> frames_dropped_{0};
};

} // namespace autodriver::camera
