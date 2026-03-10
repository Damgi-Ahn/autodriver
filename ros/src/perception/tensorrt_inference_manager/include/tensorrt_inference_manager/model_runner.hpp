#pragma once

#include "nvbuf_pool.hpp"
#include "scheduler.hpp"

#include <NvInfer.h>
#include <cuda_runtime.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace autodriver::inference {

// ---------------------------------------------------------------------------
// TRTLogger — routes TensorRT diagnostics to stderr with severity filter.
// ---------------------------------------------------------------------------
class TRTLogger : public nvinfer1::ILogger {
 public:
  void log(Severity severity, const char* msg) noexcept override;
};

// ---------------------------------------------------------------------------
// ModelConfig — one entry from models.yaml.
// ---------------------------------------------------------------------------
struct ModelConfig {
  std::string           name;
  std::string           engine_path;       ///< Absolute path to .engine file
  uint32_t              batch_size{4};
  uint32_t              timeout_ms{5};
  std::string           input_tensor_name{"images"};
  std::string           output_tensor_name{"output0"};
  std::vector<uint32_t> subscribed_cameras;  ///< empty = all cameras
};

// ---------------------------------------------------------------------------
// InferenceResult — output of one batch.
// ---------------------------------------------------------------------------
struct InferenceResult {
  std::string              model_name;
  std::vector<QueuedFrame> source_frames;
  std::vector<float>       output_data;     ///< Flat TRT output, host-side
  uint64_t                 inference_start_ns{0};
  uint64_t                 inference_end_ns{0};

  [[nodiscard]] double LatencyMs() const noexcept {
    return static_cast<double>(inference_end_ns - inference_start_ns) / 1e6;
  }
};

using ResultCallback = std::function<void(InferenceResult result)>;

// ---------------------------------------------------------------------------
// ModelRunner
//
// Owns one TensorRT ICudaEngine + IExecutionContext.
// Receives QueuedFrame batches from HybridScheduler drain thread,
// preprocesses (NvBufSurface → device float buffer), runs enqueueV3,
// copies output to host, fires ResultCallback.
//
// Lifecycle:
//   ModelRunner runner{config, pool, stream};
//   runner.SetResultCallback(cb);
//   runner.Init();          // deserialises engine, allocates CUDA buffers
//   runner.RunBatch(frames); // called from drain thread
//   ~ModelRunner()          // cudaFree, engine released
// ---------------------------------------------------------------------------
class ModelRunner {
 public:
  ModelRunner(const ModelConfig& config,
              NvBufSurfacePool&  pool,
              cudaStream_t       cuda_stream);
  ~ModelRunner();

  ModelRunner(const ModelRunner&)            = delete;
  ModelRunner& operator=(const ModelRunner&) = delete;

  void SetResultCallback(ResultCallback cb);

  /// Deserialise engine and allocate CUDA I/O buffers.
  /// Returns false on any failure (file not found, deserialization, OOM).
  [[nodiscard]] bool Init() noexcept;

  /// Execute one batch.  Called from HybridScheduler drain thread.
  /// Releases pool slots after copying input to d_input_.
  void RunBatch(std::vector<QueuedFrame> frames);

  [[nodiscard]] const std::string& name()       const noexcept { return config_.name; }
  [[nodiscard]] uint32_t           batch_size() const noexcept { return config_.batch_size; }

  [[nodiscard]] uint64_t BatchesRun()    const noexcept;
  [[nodiscard]] double   AvgLatencyMs()  const noexcept;

 private:
  // Preprocess: copy NvBufSurface batch to d_input_.
  // Returns false if any surface mapping fails.
  [[nodiscard]] bool BuildInputTensor(
      const std::vector<QueuedFrame>& frames) noexcept;

  // Derive I/O buffer sizes from the loaded engine's tensor shapes.
  void DeriveBufferSizes() noexcept;

  ModelConfig       config_;
  NvBufSurfacePool& pool_;
  cudaStream_t      stream_;
  ResultCallback    result_cb_;
  TRTLogger         logger_;

  // TensorRT RAII wrappers
  std::unique_ptr<nvinfer1::IRuntime>          runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine>       engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;

  // CUDA I/O device buffers
  void*  d_input_{nullptr};
  void*  d_output_{nullptr};
  size_t input_size_bytes_{0};
  size_t output_size_bytes_{0};

  // Host output buffer (reused across batches)
  std::vector<float> h_output_;

  std::atomic<uint64_t> batches_run_{0};
  std::atomic<uint64_t> total_latency_us_{0};
};

}  // namespace autodriver::inference
