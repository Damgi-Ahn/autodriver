#pragma once

#include "nvbuf_pool.hpp"
#include "scheduler.hpp"

#include <NvInfer.h>
#include <cuda_runtime.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace autodriver::inference {

// ---------------------------------------------------------------------------
// TensorRT logger — routes TRT log messages to rclcpp
// ---------------------------------------------------------------------------
class TRTLogger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override;
};

// ---------------------------------------------------------------------------
// Model configuration loaded from models.yaml
// ---------------------------------------------------------------------------
struct ModelConfig {
    std::string  name;
    std::string  engine_path;
    uint32_t     batch_size{4};
    uint32_t     timeout_ms{5};
    std::string  input_tensor_name{"input"};
    std::string  output_tensor_name{"output"};
    std::vector<uint32_t> subscribed_cameras;  ///< empty = all cameras
};

// ---------------------------------------------------------------------------
// Inference result for one batch
// ---------------------------------------------------------------------------
struct InferenceResult {
    std::string              model_name;
    std::vector<QueuedFrame> source_frames;    ///< Frames that produced this batch
    std::vector<float>       output_data;      ///< Flattened TRT output (host copy)
    uint64_t                 inference_start_ns{0};
    uint64_t                 inference_end_ns{0};
};

using ResultCallback = std::function<void(InferenceResult result)>;

// ---------------------------------------------------------------------------
// ModelRunner
//
// Owns one TensorRT engine + execution context.
// Receives batches from HybridScheduler, runs enqueueV3, copies output
// to host, and fires ResultCallback.
//
// One ModelRunner per model. Each runs on its own CUDA stream.
// ---------------------------------------------------------------------------
class ModelRunner {
public:
    ModelRunner(const ModelConfig& config,
                NvBufSurfacePool& pool,
                cudaStream_t      cuda_stream);
    ~ModelRunner();

    // Non-copyable
    ModelRunner(const ModelRunner&)            = delete;
    ModelRunner& operator=(const ModelRunner&) = delete;

    /// Load and deserialise the TRT engine. Must be called once before run_batch().
    bool init();

    /// Set callback invoked after each batch completes.
    void set_result_callback(ResultCallback cb);

    /// Execute inference on `frames`. Called from HybridScheduler drain thread.
    /// Builds input tensor from NvBufSurface batch, calls enqueueV3,
    /// synchronises, copies output, fires result callback.
    void run_batch(std::vector<QueuedFrame> frames);

    const std::string& name()       const { return config_.name;       }
    uint32_t           batch_size() const { return config_.batch_size; }

    // Stats
    uint64_t batches_run()      const { return batches_run_.load();  }
    double   avg_latency_ms()   const;

private:
    /// Preprocess: copy NvBufSurface batch into TRT input buffer (CUDA memcpy).
    bool build_input_tensor(const std::vector<QueuedFrame>& frames);

    ModelConfig              config_;
    NvBufSurfacePool&        pool_;
    cudaStream_t             stream_;
    ResultCallback           result_cb_;

    // TensorRT objects (RAII via unique_ptr with custom deleters)
    std::unique_ptr<nvinfer1::IRuntime>          runtime_;
    std::unique_ptr<nvinfer1::ICudaEngine>       engine_;
    std::unique_ptr<nvinfer1::IExecutionContext> context_;

    TRTLogger logger_;

    // Pre-allocated CUDA device buffers [input, output]
    void*    d_input_{nullptr};
    void*    d_output_{nullptr};
    size_t   input_size_bytes_{0};
    size_t   output_size_bytes_{0};

    // Host-side output buffer for async copy
    std::vector<float> h_output_;

    // Stats
    std::atomic<uint64_t> batches_run_{0};
    std::atomic<uint64_t> total_latency_us_{0};
};

} // namespace autodriver::inference
