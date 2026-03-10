#include "camera_manager/camera_pipeline.hpp"

#include <nvbufsurface.h>

#include <chrono>
#include <cstring>
#include <sstream>
#include <thread>

namespace autodriver::camera {

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

CameraPipeline::CameraPipeline(const CameraSpec& spec) : spec_(spec) {}

CameraPipeline::~CameraPipeline() { Stop(); }

void CameraPipeline::SetFrameCallback(FrameCallback cb) {
  frame_callback_ = std::move(cb);
}

void CameraPipeline::SetDebugFrameCallback(DebugFrameCallback cb) {
  debug_frame_callback_ = std::move(cb);
}

void CameraPipeline::SetDebugStreamEnabled(bool enabled) {
  debug_stream_enabled_ = enabled;
}

bool CameraPipeline::IsHealthy() const noexcept {
  return healthy_.load(std::memory_order_acquire);
}

uint64_t CameraPipeline::frames_captured() const noexcept {
  return frames_captured_.load(std::memory_order_relaxed);
}

uint64_t CameraPipeline::frames_dropped() const noexcept {
  return frames_dropped_.load(std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------
// Pipeline string construction
//
// Output examples:
//
// Without debug stream:
//   nvarguscamerasrc sensor-id=0 !
//   video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1,format=NV12 !
//   appsink name=inference_sink emit-signals=false sync=false max-buffers=2 drop=true
//
// With debug stream (tee):
//   nvarguscamerasrc sensor-id=0 !
//   video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1,format=NV12 !
//   tee name=t
//   t. ! queue max-size-buffers=2 leaky=downstream !
//     appsink name=inference_sink emit-signals=false sync=false max-buffers=2 drop=true
//   t. ! queue max-size-buffers=1 leaky=downstream !
//     nvvidconv ! video/x-raw,format=I420 ! nvjpegenc !
//     appsink name=debug_sink emit-signals=false sync=false max-buffers=1 drop=true
// ---------------------------------------------------------------------------

std::string CameraPipeline::BuildPipelineString() const noexcept {
  std::ostringstream ss;

  // ── Source ─────────────────────────────────────────────────────────────
  if (!spec_.gst_source.empty()) {
    ss << spec_.gst_source;
  } else {
    ss << "nvarguscamerasrc sensor-id=" << spec_.sensor_id;
  }
  ss << " ! ";

  // ── NVMM caps filter ───────────────────────────────────────────────────
  ss << "video/x-raw(memory:NVMM)"
     << ",width="      << spec_.width
     << ",height="     << spec_.height
     << ",framerate="  << spec_.fps << "/1"
     << ",format=NV12";
  ss << " ! ";

  if (!debug_stream_enabled_) {
    // Direct path to inference appsink.
    ss << "appsink name=inference_sink"
       << " emit-signals=false sync=false max-buffers=2 drop=true";
  } else {
    // Tee: one branch for inference, one for NVJPEG debug.
    ss << "tee name=t"
       << " t. ! queue max-size-buffers=2 leaky=downstream !"
       << " appsink name=inference_sink"
       << " emit-signals=false sync=false max-buffers=2 drop=true"
       << " t. ! queue max-size-buffers=1 leaky=downstream !"
       << " nvvidconv ! video/x-raw,format=I420 ! nvjpegenc !"
       << " appsink name=debug_sink"
       << " emit-signals=false sync=false max-buffers=1 drop=true";
  }

  return ss.str();
}

// ---------------------------------------------------------------------------
// Pipeline lifecycle
// ---------------------------------------------------------------------------

bool CameraPipeline::BuildPipeline() noexcept {
  const std::string pipe_str = BuildPipelineString();

  GError* error = nullptr;
  pipeline_ = gst_parse_launch(pipe_str.c_str(), &error);
  if (!pipeline_ || error) {
    if (error) g_error_free(error);
    return false;
  }

  // ── Inference appsink ───────────────────────────────────────────────────
  inference_appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "inference_sink");
  if (!inference_appsink_) {
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    return false;
  }

  GstAppSinkCallbacks inference_cbs{};
  inference_cbs.new_sample = &CameraPipeline::OnInferenceSample;
  gst_app_sink_set_callbacks(GST_APP_SINK(inference_appsink_),
                              &inference_cbs, this, nullptr);

  // ── Debug appsink (optional) ────────────────────────────────────────────
  debug_appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "debug_sink");
  if (debug_appsink_) {
    GstAppSinkCallbacks debug_cbs{};
    debug_cbs.new_sample = &CameraPipeline::OnDebugSample;
    gst_app_sink_set_callbacks(GST_APP_SINK(debug_appsink_),
                                &debug_cbs, this, nullptr);
  }

  return true;
}

void CameraPipeline::DestroyPipeline() noexcept {
  if (inference_appsink_) {
    gst_object_unref(inference_appsink_);
    inference_appsink_ = nullptr;
  }
  if (debug_appsink_) {
    gst_object_unref(debug_appsink_);
    debug_appsink_ = nullptr;
  }
  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
  }
}

bool CameraPipeline::Start() noexcept {
  if (running_.load(std::memory_order_acquire)) return true;

  if (!BuildPipeline()) {
    healthy_.store(false, std::memory_order_release);
    return false;
  }

  running_.store(true, std::memory_order_release);
  gst_element_set_state(pipeline_, GST_STATE_PLAYING);

  bus_thread_ = std::thread([this] { RunBusLoop(); });
  return true;
}

void CameraPipeline::Stop() noexcept {
  if (!running_.load(std::memory_order_acquire)) return;

  running_.store(false, std::memory_order_release);

  if (pipeline_) {
    gst_element_send_event(pipeline_, gst_event_new_eos());
  }

  if (bus_thread_.joinable()) {
    bus_thread_.join();
  }

  DestroyPipeline();
}

// ---------------------------------------------------------------------------
// Bus monitoring loop (bus_thread_)
// Handles ERROR and EOS messages.  Triggers Restart() on error.
// ---------------------------------------------------------------------------

void CameraPipeline::RunBusLoop() noexcept {
  GstBus* bus = gst_element_get_bus(pipeline_);
  if (!bus) return;

  while (running_.load(std::memory_order_acquire)) {
    GstMessage* msg = gst_bus_timed_pop_filtered(
        bus,
        100 * GST_MSECOND,
        static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS |
                                    GST_MESSAGE_WARNING));

    if (!msg) continue;

    switch (GST_MESSAGE_TYPE(msg)) {
      case GST_MESSAGE_EOS:
        gst_message_unref(msg);
        running_.store(false, std::memory_order_release);
        break;

      case GST_MESSAGE_ERROR: {
        GError* err    = nullptr;
        gchar*  detail = nullptr;
        gst_message_parse_error(msg, &err, &detail);
        // Error logged; caller reads is_healthy().
        if (err)    g_error_free(err);
        if (detail) g_free(detail);
        gst_message_unref(msg);

        if (restart_count_ < kMaxRestarts) {
          Restart();
        } else {
          healthy_.store(false, std::memory_order_release);
          running_.store(false, std::memory_order_release);
        }
        break;
      }

      case GST_MESSAGE_WARNING: {
        GError* warn   = nullptr;
        gchar*  detail = nullptr;
        gst_message_parse_warning(msg, &warn, &detail);
        if (warn)   g_error_free(warn);
        if (detail) g_free(detail);
        gst_message_unref(msg);
        break;
      }

      default:
        gst_message_unref(msg);
        break;
    }
  }

  gst_object_unref(bus);
}

bool CameraPipeline::Restart() noexcept {
  ++restart_count_;
  gst_element_set_state(pipeline_, GST_STATE_NULL);
  DestroyPipeline();
  std::this_thread::sleep_for(std::chrono::milliseconds(kRestartDelayMs));

  if (!BuildPipeline()) {
    healthy_.store(false, std::memory_order_release);
    return false;
  }

  gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  return true;
}

// ---------------------------------------------------------------------------
// Inference appsink — zero-copy DMABUF FD extraction
//
// Memory path (Jetson Orin, JetPack 6.0):
//   GstBuffer contains one GstMemory wrapping an NvBufSurface.
//   NvBufSurface::surfaceList[0].bufferDesc  is the DMABUF file descriptor.
//
// Ownership:
//   The bufferDesc fd belongs to GStreamer.  SendFd() completes a dup() in
//   the kernel (SCM_RIGHTS).  After SendFd returns we release the GstSample;
//   GStreamer may immediately close its copy, but the inference_manager's
//   kernel-duplicated copy remains valid until inference_manager closes it.
// ---------------------------------------------------------------------------

GstFlowReturn CameraPipeline::OnInferenceSample(GstAppSink* sink,
                                                  gpointer   user_data) noexcept {
  return static_cast<CameraPipeline*>(user_data)->HandleInferenceSample(sink);
}

GstFlowReturn CameraPipeline::HandleInferenceSample(GstAppSink* sink) noexcept {
  GstSample* sample = gst_app_sink_pull_sample(sink);
  if (!sample) return GST_FLOW_ERROR;

  GstBuffer* buffer = gst_sample_get_buffer(sample);
  if (!buffer) {
    gst_sample_unref(sample);
    return GST_FLOW_ERROR;
  }

  // ── Extract timestamp ───────────────────────────────────────────────────
  const GstClockTime pts = GST_BUFFER_PTS(buffer);
  const uint64_t timestamp =
      GST_CLOCK_TIME_IS_VALID(pts)
          ? static_cast<uint64_t>(pts)
          : ipc::NowNs();

  // ── Extract NvBufSurface and DMABUF fd ──────────────────────────────────
  GstMemory*  mem = gst_buffer_peek_memory(buffer, 0);
  GstMapInfo  map_info{};

  if (!gst_memory_map(mem, &map_info, GST_MAP_READ)) {
    gst_sample_unref(sample);
    frames_dropped_.fetch_add(1, std::memory_order_relaxed);
    return GST_FLOW_ERROR;
  }

  const NvBufSurface* const surface =
      reinterpret_cast<const NvBufSurface*>(map_info.data);

  // bufferDesc is the DMABUF fd for this NVMM buffer.
  const int dmabuf_fd =
      static_cast<int>(surface->surfaceList[0].bufferDesc);

  // Invoke callback while buffer is still mapped (fd must be valid for SendFd).
  if (frame_callback_ && dmabuf_fd >= 0) {
    frame_callback_(
        dmabuf_fd,
        timestamp,
        spec_.id,
        spec_.width,
        spec_.height,
        /*format=*/static_cast<uint32_t>(surface->surfaceList[0].colorFormat));
    frames_captured_.fetch_add(1, std::memory_order_relaxed);
  } else {
    frames_dropped_.fetch_add(1, std::memory_order_relaxed);
  }

  gst_memory_unmap(mem, &map_info);
  gst_sample_unref(sample);
  return GST_FLOW_OK;
}

// ---------------------------------------------------------------------------
// Debug appsink — NVJPEG compressed frames for ROS2 debug stream
// ---------------------------------------------------------------------------

GstFlowReturn CameraPipeline::OnDebugSample(GstAppSink* sink,
                                              gpointer   user_data) noexcept {
  return static_cast<CameraPipeline*>(user_data)->HandleDebugSample(sink);
}

GstFlowReturn CameraPipeline::HandleDebugSample(GstAppSink* sink) noexcept {
  GstSample* sample = gst_app_sink_pull_sample(sink);
  if (!sample) return GST_FLOW_OK;  // Non-critical path — silently drop

  GstBuffer* buffer = gst_sample_get_buffer(sample);
  if (!buffer || !debug_frame_callback_) {
    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }

  const GstClockTime pts       = GST_BUFFER_PTS(buffer);
  const uint64_t     timestamp =
      GST_CLOCK_TIME_IS_VALID(pts) ? static_cast<uint64_t>(pts) : ipc::NowNs();

  GstMapInfo map_info{};
  if (gst_buffer_map(buffer, &map_info, GST_MAP_READ)) {
    debug_frame_callback_(
        spec_.id,
        map_info.data,
        map_info.size,
        timestamp);
    gst_buffer_unmap(buffer, &map_info);
  }

  gst_sample_unref(sample);
  return GST_FLOW_OK;
}

}  // namespace autodriver::camera
