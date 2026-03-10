#include "tensorrt_inference_manager/inference_manager.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <stdexcept>

namespace autodriver::inference {

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------

InferenceManager::InferenceManager(const rclcpp::NodeOptions& options)
    : rclcpp::Node("tensorrt_inference_manager", options)
{
  declare_parameter("models_yaml",
                    "share/tensorrt_inference_manager/config/models.yaml");
  declare_parameter("system_yaml",
                    "share/tensorrt_inference_manager/config/system.yaml");
  declare_parameter("ipc_socket_path", "/tmp/autodriver/frames.sock");

  ipc_socket_path_ = get_parameter("ipc_socket_path").as_string();

  LoadConfigs();
  InitCudaStreams();
  InitNvBufPool();
  InitModelRunners();
  InitScheduler();
  BindIpcServer();

  // AcceptAndReceive blocks until camera_manager connects — run in background.
  init_thread_ = std::thread([this] { AcceptAndReceive(); });

  RCLCPP_INFO(get_logger(),
              "InferenceManager ready — %zu model(s), listening on %s",
              model_runners_.size(), ipc_socket_path_.c_str());
}

InferenceManager::~InferenceManager()
{
  Shutdown();
}

// ---------------------------------------------------------------------------
// Initialisation
// ---------------------------------------------------------------------------

void InferenceManager::LoadConfigs()
{
  namespace fs = std::filesystem;

  // ── system.yaml ──────────────────────────────────────────────────────────
  const std::string sys_yaml = get_parameter("system_yaml").as_string();
  try {
    auto root          = YAML::LoadFile(sys_yaml);
    num_cuda_streams_  = root["system"]["cuda_streams"].as<uint32_t>(8);
    nvbuf_pool_size_   = root["system"]["nvbufsurface_pool"].as<uint32_t>(128);
  } catch (const std::exception& e) {
    RCLCPP_WARN(get_logger(), "system.yaml not loaded (%s), using defaults", e.what());
  }

  // ── models.yaml ──────────────────────────────────────────────────────────
  const std::string models_yaml = get_parameter("models_yaml").as_string();
  const fs::path    yaml_dir    = fs::path(models_yaml).parent_path();

  auto root = YAML::LoadFile(models_yaml);

  for (auto it = root["models"].begin(); it != root["models"].end(); ++it) {
    ModelConfig cfg;
    cfg.name = it->first.as<std::string>();
    auto& node = it->second;

    // Resolve engine path relative to models.yaml directory if not absolute.
    fs::path ep = node["engine"].as<std::string>();
    cfg.engine_path = ep.is_absolute() ? ep : (yaml_dir / ep);

    cfg.batch_size  = node["batch_size"].as<uint32_t>(4);
    cfg.timeout_ms  = node["timeout_ms"].as<uint32_t>(5);

    if (node["input_tensor"])
      cfg.input_tensor_name  = node["input_tensor"].as<std::string>();
    if (node["output_tensor"])
      cfg.output_tensor_name = node["output_tensor"].as<std::string>();
    if (node["cameras"])
      cfg.subscribed_cameras = node["cameras"].as<std::vector<uint32_t>>();

    model_configs_.push_back(cfg);
    RCLCPP_INFO(get_logger(), "  Model '%s' → %s (batch=%u, timeout=%ums)",
                cfg.name.c_str(), cfg.engine_path.c_str(),
                cfg.batch_size, cfg.timeout_ms);
  }

  if (model_configs_.empty())
    throw std::runtime_error("No models found in " + models_yaml);
}

void InferenceManager::InitCudaStreams()
{
  cuda_streams_.resize(num_cuda_streams_);
  for (auto& s : cuda_streams_) {
    if (cudaStreamCreate(&s) != cudaSuccess)
      throw std::runtime_error("cudaStreamCreate failed");
  }
  RCLCPP_INFO(get_logger(), "Created %u CUDA streams", num_cuda_streams_);
}

void InferenceManager::InitNvBufPool()
{
  nvbuf_pool_ = std::make_unique<NvBufSurfacePool>(nvbuf_pool_size_);
  if (!nvbuf_pool_->Init())
    throw std::runtime_error("NvBufSurfacePool::Init() failed");
  RCLCPP_INFO(get_logger(), "NvBufSurfacePool: %u slots", nvbuf_pool_size_);
}

void InferenceManager::InitModelRunners()
{
  for (size_t i = 0; i < model_configs_.size(); ++i) {
    const auto& cfg    = model_configs_[i];
    cudaStream_t stream = cuda_streams_[i % num_cuda_streams_];

    auto runner = std::make_unique<ModelRunner>(cfg, *nvbuf_pool_, stream);
    runner->SetResultCallback(
        [this](InferenceResult r) { OnInferenceResult(std::move(r)); });

    if (!runner->Init())
      RCLCPP_WARN(get_logger(),
                  "ModelRunner '%s' init failed (engine not found?)",
                  cfg.name.c_str());

    model_runners_[cfg.name] = std::move(runner);

    result_pubs_[cfg.name] = create_publisher<std_msgs::msg::String>(
        "/perception/" + cfg.name + "/results", rclcpp::QoS{10});
  }
}

void InferenceManager::InitScheduler()
{
  scheduler_ = std::make_unique<HybridScheduler>(*nvbuf_pool_);

  for (const auto& cfg : model_configs_) {
    ModelQueueConfig qcfg;
    qcfg.model_name         = cfg.name;
    qcfg.batch_size         = cfg.batch_size;
    qcfg.timeout_ms         = cfg.timeout_ms;
    qcfg.subscribed_cameras = cfg.subscribed_cameras;
    scheduler_->AddModelQueue(qcfg);
  }

  scheduler_->SetBatchReadyCallback(
      [this](const std::string& model, std::vector<QueuedFrame> frames) {
        auto it = model_runners_.find(model);
        if (it != model_runners_.end())
          it->second->RunBatch(std::move(frames));
      });

  scheduler_->Start();
}

void InferenceManager::BindIpcServer()
{
  // CreateServerSocket binds + listens but does NOT block on accept.
  server_fd_ = ipc::CreateServerSocket(ipc_socket_path_);
  if (server_fd_ < 0)
    throw std::runtime_error("CreateServerSocket failed: " + ipc_socket_path_);
  RCLCPP_INFO(get_logger(), "IPC socket bound at %s, awaiting camera_manager",
              ipc_socket_path_.c_str());
}

// ---------------------------------------------------------------------------
// IPC — accept + receive loop (runs in init_thread_)
// ---------------------------------------------------------------------------

void InferenceManager::AcceptAndReceive()
{
  peer_fd_ = ipc::AcceptConnection(server_fd_);
  if (peer_fd_ < 0) {
    if (ipc_running_.load())   // unexpected
      RCLCPP_ERROR(get_logger(), "AcceptConnection failed");
    return;
  }
  ipc::SetSocketBufferSize(peer_fd_, 24 * 1024 * 1024);
  RCLCPP_INFO(get_logger(), "camera_manager connected (fd=%d)", peer_fd_);

  ipc_running_.store(true);
  IpcReceiveLoop();
}

void InferenceManager::IpcReceiveLoop()
{
  while (ipc_running_.load()) {
    int            dmabuf_fd = -1;
    ipc::FrameMeta meta{};

    const ipc::IpcStatus status = ipc::RecvFd(peer_fd_, &dmabuf_fd, &meta);

    if (status == ipc::IpcStatus::kPeerClosed) {
      RCLCPP_INFO(get_logger(), "camera_manager disconnected");
      break;
    }
    if (status != ipc::IpcStatus::kOk) {
      if (ipc_running_.load())
        RCLCPP_WARN(get_logger(), "RecvFd error — retrying");
      continue;
    }

    scheduler_->SubmitFrame(dmabuf_fd, meta);
  }
  ipc_running_.store(false);
}

// ---------------------------------------------------------------------------
// Result handling
// ---------------------------------------------------------------------------

void InferenceManager::OnInferenceResult(InferenceResult result)
{
  PublishResult(result);

  RCLCPP_DEBUG(get_logger(),
               "[%s] batch=%zu latency=%.2f ms",
               result.model_name.c_str(),
               result.source_frames.size(),
               result.LatencyMs());
}

void InferenceManager::PublishResult(const InferenceResult& result)
{
  auto it = result_pubs_.find(result.model_name);
  if (it == result_pubs_.end()) return;

  // Structured string payload (replace with typed msgs once custom .msg defined)
  std_msgs::msg::String msg;
  msg.data =
      "{\"model\":\"" + result.model_name +
      "\",\"frames\":" + std::to_string(result.source_frames.size()) +
      ",\"latency_ms\":" + std::to_string(result.LatencyMs()) +
      ",\"output_floats\":" + std::to_string(result.output_data.size()) + "}";
  it->second->publish(msg);
}

// ---------------------------------------------------------------------------
// Shutdown
// ---------------------------------------------------------------------------

void InferenceManager::Shutdown()
{
  ipc_running_.store(false);

  // Unblock IpcReceiveLoop by closing the peer fd.
  if (peer_fd_ >= 0) { ::close(peer_fd_);   peer_fd_   = -1; }
  // Unblock AcceptAndReceive (if still waiting for camera_manager).
  if (server_fd_ >= 0) { ::close(server_fd_); server_fd_ = -1; }

  if (init_thread_.joinable()) init_thread_.join();

  if (scheduler_) scheduler_->Stop();

  for (auto& s : cuda_streams_) cudaStreamDestroy(s);
  cuda_streams_.clear();
}

}  // namespace autodriver::inference
