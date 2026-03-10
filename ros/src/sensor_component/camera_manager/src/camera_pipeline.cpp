#include "camera_manager/camera_pipeline.hpp"

#include <nvbufsurface.h>
#include <gst/app/gstappsink.h>

#include <chrono>
#include <stdexcept>
#include <thread>

namespace autodriver::camera {

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

CameraPipeline::CameraPipeline(const CameraConfig& config)
    : config_(config)
{}

CameraPipeline::~CameraPipeline()
{
    stop();
}

void CameraPipeline::set_frame_callback(FrameCallback cb)
{
    frame_callback_ = std::move(cb);
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

bool CameraPipeline::start()
{
    if (running_.load()) return true;

    if (!build_pipeline()) {
        healthy_.store(false);
        return false;
    }

    running_.store(true);
    pipeline_thread_ = std::thread([this]{ run_loop(); });
    return true;
}

void CameraPipeline::stop()
{
    running_.store(false);

    if (pipeline_) {
        gst_element_send_event(pipeline_, gst_event_new_eos());
    }

    if (pipeline_thread_.joinable()) {
        pipeline_thread_.join();
    }

    destroy_pipeline();
}

// ---------------------------------------------------------------------------
// GStreamer pipeline construction
// ---------------------------------------------------------------------------

bool CameraPipeline::build_pipeline()
{
    // Pipeline string (parsed at runtime for flexibility):
    //   nvv4l2camerasrc device=<dev> !
    //   video/x-raw(memory:NVMM),format=<fmt>,width=<w>,height=<h>,framerate=<fps>/1 !
    //   nvvidconv !
    //   video/x-raw(memory:NVMM),format=NV12 !
    //   tee name=t
    //   t. ! queue max-size-buffers=2 leaky=downstream ! appsink name=inference_sink
    //   t. ! queue max-size-buffers=1 leaky=downstream !
    //      nvvidconv ! nvjpegenc ! appsink name=debug_sink

    GError* error = nullptr;
    auto pipe_str = build_pipeline_string();

    pipeline_ = gst_parse_launch(pipe_str.c_str(), &error);
    if (!pipeline_ || error) {
        if (error) g_error_free(error);
        return false;
    }

    // Retrieve appsinks
    inference_appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "inference_sink");
    debug_appsink_     = gst_bin_get_by_name(GST_BIN(pipeline_), "debug_sink");

    // Configure inference appsink
    g_object_set(inference_appsink_,
                 "emit-signals", FALSE,
                 "sync",         FALSE,
                 "max-buffers",  2,
                 "drop",         TRUE,
                 nullptr);

    GstAppSinkCallbacks inference_cbs{};
    inference_cbs.new_sample = &CameraPipeline::on_inference_sample;
    gst_app_sink_set_callbacks(GST_APP_SINK(inference_appsink_),
                                &inference_cbs, this, nullptr);

    // Configure debug appsink (non-blocking)
    if (debug_appsink_) {
        g_object_set(debug_appsink_,
                     "emit-signals", FALSE,
                     "sync",         FALSE,
                     "max-buffers",  1,
                     "drop",         TRUE,
                     nullptr);
        GstAppSinkCallbacks debug_cbs{};
        debug_cbs.new_sample = &CameraPipeline::on_debug_sample;
        gst_app_sink_set_callbacks(GST_APP_SINK(debug_appsink_),
                                    &debug_cbs, this, nullptr);
    }

    return true;
}

std::string CameraPipeline::build_pipeline_string() const
{
    // TODO(Stage 4): Full pipeline string construction
    return "";
}

void CameraPipeline::destroy_pipeline()
{
    if (inference_appsink_) { gst_object_unref(inference_appsink_); inference_appsink_ = nullptr; }
    if (debug_appsink_)     { gst_object_unref(debug_appsink_);     debug_appsink_     = nullptr; }
    if (pipeline_)          { gst_object_unref(pipeline_);          pipeline_           = nullptr; }
}

// ---------------------------------------------------------------------------
// Run loop (pipeline thread)
// ---------------------------------------------------------------------------

void CameraPipeline::run_loop()
{
    gst_element_set_state(pipeline_, GST_STATE_PLAYING);

    GstBus* bus = gst_element_get_bus(pipeline_);
    while (running_.load()) {
        GstMessage* msg = gst_bus_timed_pop_filtered(
            bus, 100 * GST_MSECOND,
            static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

        if (!msg) continue;

        switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_EOS:
            gst_message_unref(msg);
            running_.store(false);
            break;
        case GST_MESSAGE_ERROR: {
            GError* err = nullptr;
            gst_message_parse_error(msg, &err, nullptr);
            if (err) g_error_free(err);
            gst_message_unref(msg);
            if (restart_count_ < kMaxRestartAttempts) {
                restart();
            } else {
                healthy_.store(false);
                running_.store(false);
            }
            break;
        }
        default:
            gst_message_unref(msg);
            break;
        }
    }

    gst_object_unref(bus);
    gst_element_set_state(pipeline_, GST_STATE_NULL);
}

// ---------------------------------------------------------------------------
// appsink callbacks
// ---------------------------------------------------------------------------

GstFlowReturn CameraPipeline::on_inference_sample(GstAppSink* sink, gpointer user_data)
{
    return static_cast<CameraPipeline*>(user_data)->handle_inference_sample(sink);
}

GstFlowReturn CameraPipeline::handle_inference_sample(GstAppSink* sink)
{
    // TODO(Stage 4): extract NvBufSurface, export DMABUF fd, invoke frame_callback_
    GstSample* sample = gst_app_sink_pull_sample(sink);
    if (!sample) return GST_FLOW_ERROR;

    frames_captured_.fetch_add(1, std::memory_order_relaxed);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

GstFlowReturn CameraPipeline::on_debug_sample(GstAppSink* sink, gpointer user_data)
{
    return static_cast<CameraPipeline*>(user_data)->handle_debug_sample(sink);
}

GstFlowReturn CameraPipeline::handle_debug_sample(GstAppSink* sink)
{
    // TODO(Stage 4): forward JPEG data for ROS2 CompressedImage publish
    GstSample* sample = gst_app_sink_pull_sample(sink);
    if (!sample) return GST_FLOW_OK;  // Non-blocking: silently drop

    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

// ---------------------------------------------------------------------------
// Restart
// ---------------------------------------------------------------------------

bool CameraPipeline::restart()
{
    ++restart_count_;
    destroy_pipeline();
    std::this_thread::sleep_for(std::chrono::milliseconds(kRestartDelayMs));
    return build_pipeline();
}

} // namespace autodriver::camera
