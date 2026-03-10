#include "tensorrt_inference_manager/inference_manager.hpp"

#include <yaml-cpp/yaml.h>
#include <stdexcept>

namespace autodriver::inference {

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------

InferenceManager::InferenceManager(const rclcpp::NodeOptions& options)
    : rclcpp::Node("tensorrt_inference_manager", options)
{
    declare_parameter("models_yaml",     "config/models.yaml");
    declare_parameter("system_yaml",     "config/system.yaml");
    declare_parameter("ipc_socket_path", "/tmp/autodriver/frames.sock");

    ipc_socket_path_ = get_parameter("ipc_socket_path").as_string();

    load_configs();
    init_cuda_streams();
    init_nvbuf_pool();
    init_model_runners();
    init_scheduler();
    connect_ipc();

    RCLCPP_INFO(get_logger(), "InferenceManager ready — %zu model(s)",
                model_runners_.size());
}

InferenceManager::~InferenceManager()
{
    shutdown();
}

// ---------------------------------------------------------------------------
// Initialisation
// ---------------------------------------------------------------------------

void InferenceManager::load_configs()
{
    // ── system.yaml ───────────────────────────────────────────────────────
    const auto sys_yaml = get_parameter("system_yaml").as_string();
    try {
        auto root           = YAML::LoadFile(sys_yaml);
        num_cuda_streams_   = root["system"]["cuda_streams"].as<uint32_t>(8);
        nvbuf_pool_size_    = root["system"]["nvbufsurface_pool"].as<uint32_t>(128);
    } catch (...) {}

    // ── models.yaml ───────────────────────────────────────────────────────
    const auto models_yaml = get_parameter("models_yaml").as_string();
    auto root = YAML::LoadFile(models_yaml);

    for (auto it = root["models"].begin(); it != root["models"].end(); ++it) {
        ModelConfig cfg;
        cfg.name         = it->first.as<std::string>();
        auto& node       = it->second;
        cfg.engine_path  = node["engine"].as<std::string>();
        cfg.batch_size   = node["batch_size"].as<uint32_t>(4);
        cfg.timeout_ms   = node["timeout_ms"].as<uint32_t>(5);

        if (node["input_tensor"])  cfg.input_tensor_name  = node["input_tensor"].as<std::string>();
        if (node["output_tensor"]) cfg.output_tensor_name = node["output_tensor"].as<std::string>();
        if (node["cameras"])
            cfg.subscribed_cameras = node["cameras"].as<std::vector<uint32_t>>();

        model_configs_.push_back(cfg);
    }
}

void InferenceManager::init_cuda_streams()
{
    cuda_streams_.resize(num_cuda_streams_);
    for (auto& s : cuda_streams_)
        if (cudaStreamCreate(&s) != cudaSuccess)
            throw std::runtime_error("cudaStreamCreate failed");
}

void InferenceManager::init_nvbuf_pool()
{
    nvbuf_pool_ = std::make_unique<NvBufSurfacePool>(nvbuf_pool_size_);
    if (!nvbuf_pool_->init())
        throw std::runtime_error("NvBufSurfacePool init failed");
}

void InferenceManager::init_model_runners()
{
    for (size_t i = 0; i < model_configs_.size(); ++i) {
        const auto& cfg = model_configs_[i];
        cudaStream_t stream = cuda_streams_[i % num_cuda_streams_];

        auto runner = std::make_unique<ModelRunner>(cfg, *nvbuf_pool_, stream);
        if (!runner->init())
            RCLCPP_WARN(get_logger(), "ModelRunner '%s' init failed", cfg.name.c_str());

        runner->set_result_callback(
            [this](InferenceResult r){ on_inference_result(std::move(r)); });

        model_runners_[cfg.name] = std::move(runner);

        // ROS2 result publisher per model
        result_pubs_[cfg.name] = create_publisher<std_msgs::msg::String>(
            "/perception/" + cfg.name + "/results", 10);
    }
}

void InferenceManager::init_scheduler()
{
    scheduler_ = std::make_unique<HybridScheduler>(*nvbuf_pool_);

    for (const auto& cfg : model_configs_) {
        ModelQueueConfig qcfg;
        qcfg.model_name          = cfg.name;
        qcfg.batch_size          = cfg.batch_size;
        qcfg.timeout_ms          = cfg.timeout_ms;
        qcfg.subscribed_cameras  = cfg.subscribed_cameras;
        scheduler_->add_model_queue(qcfg);
    }

    scheduler_->set_batch_ready_callback(
        [this](const std::string& model, std::vector<QueuedFrame> frames) {
            auto it = model_runners_.find(model);
            if (it != model_runners_.end())
                it->second->run_batch(std::move(frames));
        });

    scheduler_->start();
}

void InferenceManager::connect_ipc()
{
    ipc_fd_ = ipc::create_client_socket(ipc_socket_path_);
    if (ipc_fd_ < 0)
        throw std::runtime_error("IPC connect failed: " + ipc_socket_path_);

    ipc::set_socket_buffer_size(ipc_fd_, 4 * 1024 * 1024);

    ipc_running_.store(true);
    ipc_thread_ = std::thread([this]{ ipc_receive_loop(); });
    RCLCPP_INFO(get_logger(), "IPC connected to %s", ipc_socket_path_.c_str());
}

// ---------------------------------------------------------------------------
// IPC receive loop
// ---------------------------------------------------------------------------

void InferenceManager::ipc_receive_loop()
{
    while (ipc_running_.load()) {
        int dmabuf_fd = -1;
        ipc::FrameMeta meta{};

        if (!ipc::recv_fd(ipc_fd_, &dmabuf_fd, &meta)) {
            if (ipc_running_.load())
                RCLCPP_WARN(get_logger(), "IPC recv failed — retrying");
            continue;
        }

        scheduler_->submit_frame(dmabuf_fd, meta);
    }
}

// ---------------------------------------------------------------------------
// Result handling
// ---------------------------------------------------------------------------

void InferenceManager::on_inference_result(InferenceResult result)
{
    // TODO(Stage 5/7): decode output_data and publish typed messages
    auto it = result_pubs_.find(result.model_name);
    if (it == result_pubs_.end()) return;

    const double latency_ms =
        (result.inference_end_ns - result.inference_start_ns) / 1e6;

    std_msgs::msg::String msg;
    msg.data = result.model_name + " latency=" + std::to_string(latency_ms) + "ms";
    it->second->publish(msg);
}

// ---------------------------------------------------------------------------
// Shutdown
// ---------------------------------------------------------------------------

void InferenceManager::shutdown()
{
    ipc_running_.store(false);
    if (ipc_fd_ >= 0) { ::close(ipc_fd_); ipc_fd_ = -1; }
    if (ipc_thread_.joinable()) ipc_thread_.join();

    if (scheduler_) scheduler_->stop();

    for (auto& s : cuda_streams_) cudaStreamDestroy(s);
    cuda_streams_.clear();
}

} // namespace autodriver::inference
