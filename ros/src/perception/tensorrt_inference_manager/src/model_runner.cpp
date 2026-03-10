#include "tensorrt_inference_manager/model_runner.hpp"

#include <fstream>
#include <chrono>
#include <cstring>
#include <stdexcept>

namespace autodriver::inference {

// ---------------------------------------------------------------------------
// TRTLogger
// ---------------------------------------------------------------------------

void TRTLogger::log(Severity severity, const char* msg) noexcept
{
  // Map TRT severity to stderr with a prefix.
  // In production wire to rclcpp::Logger via a globally stored reference.
  switch (severity) {
    case Severity::kINTERNAL_ERROR:
    case Severity::kERROR:
      fprintf(stderr, "[TRT][ERROR] %s\n", msg);
      break;
    case Severity::kWARNING:
      fprintf(stderr, "[TRT][WARN ] %s\n", msg);
      break;
    case Severity::kINFO:
      fprintf(stderr, "[TRT][INFO ] %s\n", msg);
      break;
    default:
      break;
  }
}

// ---------------------------------------------------------------------------
// ModelRunner
// ---------------------------------------------------------------------------

ModelRunner::ModelRunner(const ModelConfig& config,
                          NvBufSurfacePool&  pool,
                          cudaStream_t       cuda_stream)
    : config_(config), pool_(pool), stream_(cuda_stream)
{}

ModelRunner::~ModelRunner()
{
  if (d_input_)  cudaFree(d_input_);
  if (d_output_) cudaFree(d_output_);
}

void ModelRunner::SetResultCallback(ResultCallback cb)
{
  result_cb_ = std::move(cb);
}

bool ModelRunner::Init() noexcept
{
  // ── Load serialised engine ────────────────────────────────────────────────
  std::ifstream f(config_.engine_path, std::ios::binary);
  if (!f) return false;

  f.seekg(0, std::ios::end);
  const size_t size = static_cast<size_t>(f.tellg());
  f.seekg(0, std::ios::beg);
  std::vector<char> data(size);
  f.read(data.data(), static_cast<std::streamsize>(size));

  // ── Deserialise ───────────────────────────────────────────────────────────
  runtime_.reset(nvinfer1::createInferRuntime(logger_));
  if (!runtime_) return false;

  engine_.reset(runtime_->deserializeCudaEngine(data.data(), size));
  if (!engine_) return false;

  context_.reset(engine_->createExecutionContext());
  if (!context_) return false;

  // ── Derive buffer sizes from engine tensor shapes ─────────────────────────
  DeriveBufferSizes();

  if (cudaMalloc(&d_input_,  input_size_bytes_)  != cudaSuccess) return false;
  if (cudaMalloc(&d_output_, output_size_bytes_) != cudaSuccess) return false;
  h_output_.resize(output_size_bytes_ / sizeof(float));

  return true;
}

void ModelRunner::DeriveBufferSizes() noexcept
{
  // TRT 10 API: getTensorShape returns dims for named I/O tensors.
  auto volume = [](const nvinfer1::Dims& d) {
    size_t v = 1;
    for (int i = 0; i < d.nbDims; ++i)
      v *= static_cast<size_t>(d.d[i] > 0 ? d.d[i] : 1);
    return v;
  };

  const nvinfer1::Dims in_dims =
      engine_->getTensorShape(config_.input_tensor_name.c_str());
  const nvinfer1::Dims out_dims =
      engine_->getTensorShape(config_.output_tensor_name.c_str());

  // Treat dynamic dim (-1) as batch_size for the batch dimension (dim 0).
  // The remaining dims give the per-sample volume.
  size_t in_vol  = volume(in_dims);
  size_t out_vol = volume(out_dims);

  // If the engine reports -1 for the batch dim, scale by config batch size.
  if (in_dims.nbDims > 0 && in_dims.d[0] <= 0)
    in_vol  *= config_.batch_size;
  if (out_dims.nbDims > 0 && out_dims.d[0] <= 0)
    out_vol *= config_.batch_size;

  input_size_bytes_  = in_vol  * sizeof(float);
  output_size_bytes_ = out_vol * sizeof(float);

  // Guard against zero sizes (e.g., engine not fully parsed).
  if (input_size_bytes_ == 0)
    input_size_bytes_  = static_cast<size_t>(config_.batch_size) * 3 * 1920 * 1080 * sizeof(float);
  if (output_size_bytes_ == 0)
    output_size_bytes_ = static_cast<size_t>(config_.batch_size) * 1000 * sizeof(float);
}

// ---------------------------------------------------------------------------
// RunBatch
// ---------------------------------------------------------------------------

void ModelRunner::RunBatch(std::vector<QueuedFrame> frames)
{
  if (frames.empty()) return;

  const uint64_t t_start = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());

  // Pad to batch_size with a copy of the last frame if under-full batch.
  // (TRT engines built with fixed batch require a full batch input.)
  const size_t pad_to = config_.batch_size;
  while (frames.size() < pad_to) frames.push_back(frames.back());

  if (!BuildInputTensor(frames)) {
    for (auto& f : frames) {
      if (f.pool_slot != NvBufSurfacePool::kInvalidSlot)
        pool_.ReleaseSlot(f.pool_slot);
    }
    return;
  }

  // ── TRT10 enqueueV3 ───────────────────────────────────────────────────────
  context_->setTensorAddress(config_.input_tensor_name.c_str(),  d_input_);
  context_->setTensorAddress(config_.output_tensor_name.c_str(), d_output_);
  if (!context_->enqueueV3(stream_)) {
    for (auto& f : frames) {
      if (f.pool_slot != NvBufSurfacePool::kInvalidSlot)
        pool_.ReleaseSlot(f.pool_slot);
    }
    return;
  }

  // ── Copy output to host ───────────────────────────────────────────────────
  cudaMemcpyAsync(h_output_.data(), d_output_, output_size_bytes_,
                  cudaMemcpyDeviceToHost, stream_);
  cudaStreamSynchronize(stream_);

  // Release pool slots (input data already on device).
  for (auto& f : frames) {
    if (f.pool_slot != NvBufSurfacePool::kInvalidSlot)
      pool_.ReleaseSlot(f.pool_slot);
  }

  const uint64_t t_end = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());

  batches_run_.fetch_add(1, std::memory_order_relaxed);
  total_latency_us_.fetch_add((t_end - t_start) / 1000, std::memory_order_relaxed);

  if (result_cb_) {
    InferenceResult res;
    res.model_name         = config_.name;
    res.source_frames      = std::move(frames);
    res.output_data        = h_output_;
    res.inference_start_ns = t_start;
    res.inference_end_ns   = t_end;
    result_cb_(std::move(res));
  }
}

// ---------------------------------------------------------------------------
// BuildInputTensor
//
// For each frame: map pool NvBufSurface → get CUDA pointer → cudaMemcpy2D
// into the corresponding batch slot in d_input_.
//
// NV12 format: Y plane followed by interleaved UV plane.
// We copy only the Y (luma) plane as a grayscale proxy. Production code should
// use a CUDA NV12→RGB+normalise kernel here (e.g., via NPP or custom kernel).
// ---------------------------------------------------------------------------

bool ModelRunner::BuildInputTensor(const std::vector<QueuedFrame>& frames) noexcept
{
  // Per-frame size in bytes in the flat d_input_ buffer.
  const size_t frame_bytes = input_size_bytes_ / config_.batch_size;

  for (size_t i = 0; i < frames.size(); ++i) {
    const QueuedFrame& qf   = frames[i];
    if (qf.pool_slot == NvBufSurfacePool::kInvalidSlot) continue;

    NvBufSurface* surf = pool_.Surface(qf.pool_slot);
    if (!surf) return false;

    // Map surface to CPU/GPU address space for read access.
    if (NvBufSurfaceMap(surf, /*index=*/-1, /*plane=*/-1,
                         NVBUF_MAP_READ_WRITE) != 0)
      return false;

    // Synchronise mapping (required before GPU access via mapped addr).
    NvBufSurfaceSyncForCpu(surf, -1, -1);

    // Copy Y plane (or first component) to device input buffer.
    // surfaceList[0].mappedAddr.addr[0] is the Y plane VA.
    const void* src_y    = surf->surfaceList[0].mappedAddr.addr[0];
    const size_t y_bytes = surf->surfaceList[0].dataSize;
    const size_t copy_bytes = std::min(frame_bytes, y_bytes);

    uint8_t* dst = static_cast<uint8_t*>(d_input_) + i * frame_bytes;
    cudaMemcpyAsync(dst, src_y, copy_bytes,
                    cudaMemcpyHostToDevice, stream_);

    NvBufSurfaceUnMap(surf, -1, -1);
  }

  // Ensure all async copies are queued before enqueueV3.
  cudaStreamSynchronize(stream_);
  return true;
}

// ---------------------------------------------------------------------------
// Stats
// ---------------------------------------------------------------------------

uint64_t ModelRunner::BatchesRun() const noexcept
{
  return batches_run_.load(std::memory_order_relaxed);
}

double ModelRunner::AvgLatencyMs() const noexcept
{
  const uint64_t n = batches_run_.load(std::memory_order_relaxed);
  if (n == 0) return 0.0;
  return static_cast<double>(total_latency_us_.load(std::memory_order_relaxed))
         / static_cast<double>(n) / 1000.0;
}

}  // namespace autodriver::inference
