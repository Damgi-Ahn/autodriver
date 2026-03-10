#include "tensorrt_inference_manager/model_runner.hpp"

#include <NvOnnxParser.h>
#include <fstream>
#include <stdexcept>
#include <chrono>

namespace autodriver::inference {

// ---------------------------------------------------------------------------
// TRT Logger
// ---------------------------------------------------------------------------

void TRTLogger::log(Severity severity, const char* msg) noexcept
{
    // TODO(Stage 5): route to rclcpp logger
    (void)severity;
    (void)msg;
}

// ---------------------------------------------------------------------------
// ModelRunner
// ---------------------------------------------------------------------------

ModelRunner::ModelRunner(const ModelConfig& config,
                          NvBufSurfacePool& pool,
                          cudaStream_t cuda_stream)
    : config_(config), pool_(pool), stream_(cuda_stream)
{}

ModelRunner::~ModelRunner()
{
    if (d_input_)  cudaFree(d_input_);
    if (d_output_) cudaFree(d_output_);
}

void ModelRunner::set_result_callback(ResultCallback cb)
{
    result_cb_ = std::move(cb);
}

bool ModelRunner::init()
{
    // ── Load serialised engine from disk ─────────────────────────────────
    std::ifstream engine_file(config_.engine_path, std::ios::binary);
    if (!engine_file) return false;

    engine_file.seekg(0, std::ios::end);
    const size_t engine_size = static_cast<size_t>(engine_file.tellg());
    engine_file.seekg(0, std::ios::beg);

    std::vector<char> engine_data(engine_size);
    engine_file.read(engine_data.data(), static_cast<std::streamsize>(engine_size));

    // ── Deserialise ───────────────────────────────────────────────────────
    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    if (!runtime_) return false;

    engine_.reset(runtime_->deserializeCudaEngine(engine_data.data(), engine_size));
    if (!engine_) return false;

    context_.reset(engine_->createExecutionContext());
    if (!context_) return false;

    // ── Allocate CUDA buffers ─────────────────────────────────────────────
    // TODO(Stage 5): derive sizes from engine tensor dims
    // Placeholder sizes:
    input_size_bytes_  = static_cast<size_t>(config_.batch_size) * 3 * 1920 * 1080 * sizeof(float);
    output_size_bytes_ = static_cast<size_t>(config_.batch_size) * 1000 * sizeof(float);

    if (cudaMalloc(&d_input_,  input_size_bytes_)  != cudaSuccess) return false;
    if (cudaMalloc(&d_output_, output_size_bytes_) != cudaSuccess) return false;

    h_output_.resize(output_size_bytes_ / sizeof(float));

    return true;
}

void ModelRunner::run_batch(std::vector<QueuedFrame> frames)
{
    if (frames.empty()) return;

    const uint64_t t_start = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());

    // ── Build input tensor from NvBufSurface batch ────────────────────────
    if (!build_input_tensor(frames)) {
        // Release pool slots on failure
        for (auto& f : frames) pool_.release(f.pool_slot);
        return;
    }

    // ── TensorRT enqueueV3 ────────────────────────────────────────────────
    void* bindings[] = {d_input_, d_output_};

    // TRT10 API: setTensorAddress + enqueueV3
    context_->setTensorAddress(config_.input_tensor_name.c_str(),  d_input_);
    context_->setTensorAddress(config_.output_tensor_name.c_str(), d_output_);
    context_->enqueueV3(stream_);

    // ── Copy output to host (async) ───────────────────────────────────────
    cudaMemcpyAsync(h_output_.data(), d_output_, output_size_bytes_,
                    cudaMemcpyDeviceToHost, stream_);
    cudaStreamSynchronize(stream_);

    // Release pool slots
    for (auto& f : frames) pool_.release(f.pool_slot);

    const uint64_t t_end = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());

    batches_run_.fetch_add(1, std::memory_order_relaxed);
    total_latency_us_.fetch_add((t_end - t_start) / 1000, std::memory_order_relaxed);

    // ── Fire result callback ──────────────────────────────────────────────
    if (result_cb_) {
        InferenceResult result;
        result.model_name        = config_.name;
        result.source_frames     = std::move(frames);
        result.output_data       = h_output_;
        result.inference_start_ns = t_start;
        result.inference_end_ns   = t_end;
        result_cb_(std::move(result));
    }
}

bool ModelRunner::build_input_tensor(const std::vector<QueuedFrame>& frames)
{
    // TODO(Stage 5): iterate frames, use NvBufSurface → cudaMemcpy2D
    // into d_input_ with correct stride/offset per batch slot.
    (void)frames;
    return true;
}

double ModelRunner::avg_latency_ms() const
{
    const uint64_t n = batches_run_.load();
    if (n == 0) return 0.0;
    return static_cast<double>(total_latency_us_.load()) / static_cast<double>(n) / 1000.0;
}

} // namespace autodriver::inference
