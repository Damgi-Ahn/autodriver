#include "hybrid_localization/hybrid_localization_node.hpp"

#include <chrono>
#include <cmath>
#include <cstring>

#include <boost/make_shared.hpp>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>

namespace hybrid_localization
{

// NOTE: 노드의 수명주기(초기화/구독/퍼블리셔/타이머/TF 설정)를 한 곳에 모아
// 시스템 전체 배선 구조를 빠르게 파악할 수 있도록 구성한다.

HybridLocalizationNode::HybridLocalizationNode(const rclcpp::NodeOptions & t_options)
: Node("hybrid_localization", t_options), imu_count_(0), gnss_count_(0),
  gnss_vel_count_(0), velocity_count_(0), steering_count_(0),
  map_projector_received_(false), last_imu_stamp_(0, 0, RCL_ROS_TIME),
  last_gnss_stamp_(0, 0, RCL_ROS_TIME),
  last_gnss_vel_stamp_(0, 0, RCL_ROS_TIME),
  last_velocity_stamp_(0, 0, RCL_ROS_TIME),
  last_steering_stamp_(0, 0, RCL_ROS_TIME),
  last_heading_stamp_(0, 0, RCL_ROS_TIME)
{
  RCLCPP_INFO(this->get_logger(), "Initializing Hybrid Localization Node");

  // CallbackGroup 생성 — 각 주파수 도메인을 독립 스레드에서 실행
  imu_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  sensor_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  load_parameters(); // 파라미터 로딩은 별도 로더에서 일괄 처리

  // FGO Stage 2: ImuPreintegration 파라미터 적용 및 초기화
  imu_preint_ = ImuPreintegration(node_params_.imu_preint);
  keyframe_buffer_.set_max_size(
    static_cast<size_t>(node_params_.fgo_backend.window_size));

  // FGO Stage 3: ISAM2 백엔드 + GTSAM IMU 사전적분 초기화
  fgo_backend_.initialize(node_params_.fgo_backend, node_params_.imu_preint);
  gtsam_preint_ =
    boost::make_shared<gtsam::PreintegratedCombinedMeasurements>(
    fgo_backend_.gtsam_preint_params(),
    gtsam::imuBias::ConstantBias{});

  // FGO Stage 4+5: EskfCorrector 파라미터 적용
  eskf_corrector_.set_params(node_params_.fgo_corrector);

  const auto & io = node_params_.io;
  OdomBuilderConfig odom_config;
  odom_config.map_frame = io.map_frame;
  odom_config.base_frame = io.base_frame;
  odom_config.gyro_noise_std = node_params_.eskf.gyro_noise_std;
  odom_builder_.set_config(odom_config);

  // TF 버퍼/리스너는 초기화 초기에 생성해 다른 모듈에서 사용 가능하도록 한다.
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
    io.output_odom_topic, 10);
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/localization/pose_twist_fusion_filter/pose", 10);
  gnss_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
    "/localization/kinematic_state_gnss", 10);
  if (node_params_.gnss.publish_pose_with_covariance) {
    gnss_pose_cov_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      node_params_.gnss.pose_with_covariance_topic, 10);
  }
  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", 10);
  preprocessed_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
    "/sensing/imu/imu_data", 10);
  gnss_velocity_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>(
    "/sensing/gnss/velocity_status", 10);
  keyframe_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "/localization/fgo/keyframe_path", 10);
  keyframe_path_.header.frame_id = io.map_frame;

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  tf_cache_.set_tf_buffer(tf_buffer_);
  tf_cache_.set_base_frame(io.base_frame);
  tf_cache_.cache_common_extrinsics(this->get_logger());

  // 센서 구독/퍼블리셔 설정은 노드 배선의 핵심
  // IMU: 전용 고주파 그룹 (200Hz)
  {
    rclcpp::SubscriptionOptions imu_opts;
    imu_opts.callback_group = imu_cb_group_;
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      io.imu_topic, 1,
      std::bind(&HybridLocalizationNode::imu_callback, this, std::placeholders::_1),
      imu_opts);
  }

  // 저주파 센서 구독: GNSS / Vehicle / Heading — 공유 센서 그룹
  {
    rclcpp::SubscriptionOptions sensor_opts;
    sensor_opts.callback_group = sensor_cb_group_;

    gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      io.gnss_topic, 1,
      std::bind(&HybridLocalizationNode::gnss_callback, this, std::placeholders::_1),
      sensor_opts);

    gnss_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      io.gnss_vel_topic, 1,
      std::bind(&HybridLocalizationNode::gnss_vel_callback, this, std::placeholders::_1),
      sensor_opts);

    map_projector_sub_ =
      this->create_subscription<tier4_map_msgs::msg::MapProjectorInfo>(
      io.map_projector_info_topic, rclcpp::QoS(1).transient_local(),
      std::bind(
        &HybridLocalizationNode::map_projector_info_callback, this,
        std::placeholders::_1),
      sensor_opts);

    velocity_sub_ =
      this->create_subscription<autoware_vehicle_msgs::msg::VelocityReport>(
      io.velocity_topic, 1,
      std::bind(&HybridLocalizationNode::velocity_callback, this, std::placeholders::_1),
      sensor_opts);

    steering_sub_ =
      this->create_subscription<autoware_vehicle_msgs::msg::SteeringReport>(
      io.steering_topic, 1,
      std::bind(&HybridLocalizationNode::steering_callback, this, std::placeholders::_1),
      sensor_opts);

    heading_sub_ = this->create_subscription<skyautonet_msgs::msg::Gphdt>(
      io.heading_topic, 1,
      std::bind(&HybridLocalizationNode::heading_callback, this, std::placeholders::_1),
      sensor_opts);

    // 외부 초기자세 입력 (/initialpose3d)
    initialpose_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      node_params_.init.external_initialpose_topic, 1,
      std::bind(
        &HybridLocalizationNode::initialpose_callback, this,
        std::placeholders::_1),
      sensor_opts);
  }

  // Autoware-compatible trigger service (activate/deactivate)
  service_trigger_node_ = this->create_service<std_srvs::srv::SetBool>(
    "trigger_node_srv",
    std::bind(
      &HybridLocalizationNode::service_trigger_node, this,
      std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(),
    sensor_cb_group_);


  // IMU 캘리브레이션 경로/시작 여부는 파라미터에 의해 결정
  if (node_params_.init_imu_calibration) {
    RCLCPP_INFO(
      this->get_logger(), "Starting IMU calibration (duration: %.1f seconds)",
      IMU_CALIBRATION_DURATION_SEC);
    RCLCPP_WARN(this->get_logger(), "Please ensure the vehicle is stationary during calibration!");
    imu_calibration_manager_.start(this->now(), IMU_CALIBRATION_DURATION_SEC);
  } else {
    if (imu_preprocessor_.load_calibration(
        node_params_.calibration_file_path))
    {
      RCLCPP_INFO(
        this->get_logger(), "Loaded IMU calibration from file: %s",
        node_params_.calibration_file_path.c_str());
      RCLCPP_INFO(
        this->get_logger(), " Gyro bias: [%.6f, %.6f, %.6f] rad/s",
        imu_preprocessor_.calibration().gyro_bias.x,
        imu_preprocessor_.calibration().gyro_bias.y,
        imu_preprocessor_.calibration().gyro_bias.z);
    } else {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to load IMU calibration from file: %s. Please run with " "init_imu_calibration:=true to calibrate.",
        node_params_.calibration_file_path.c_str());
    }
  }

  // 발행 타이머 생성 — 타이머 전용 그룹에 할당
  const double rate_hz = node_params_.io.publish_rate;
  const auto period = std::chrono::duration<double>(1.0 / rate_hz);
  publish_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&HybridLocalizationNode::publish_timer_callback, this),
    timer_cb_group_);

  RCLCPP_INFO(
    this->get_logger(), "Hybrid Localization Node ready (publish_rate=%.0f Hz, threads=3)",
    rate_hz);
}

void HybridLocalizationNode::maybe_push_keyframe(
  const rclcpp::Time & stamp,
  const NominalState & state,
  const EskfCore::P15 & P)
{
  // state_mutex_ 가 이미 잠긴 상태에서 호출된다.
  if (!keyframe_buffer_.should_create_keyframe(
      stamp, state, node_params_.keyframe))
  {
    return;
  }

  Keyframe kf;
  kf.stamp = stamp;
  kf.state = state;
  kf.P = P;
  kf.preint = imu_preint_;  // ESKF 재선형화용 사전적분

  // FGO Stage 3: GTSAM 사전적분 스냅샷 저장 (null = 첫 키프레임)
  if (gtsam_preint_ && gtsam_preint_->deltaTij() > 0.0) {
    kf.gtsam_preint = boost::make_shared<gtsam::PreintegratedCombinedMeasurements>(
      *gtsam_preint_);
  }

  keyframe_buffer_.push(kf);
  ++fgo_keyframe_count_;

  // FGO Stage 5: 키프레임 경로에 새 포즈 추가
  {
    geometry_msgs::msg::PoseStamped kf_pose;
    kf_pose.header.stamp = stamp;
    kf_pose.header.frame_id = node_params_.io.map_frame;
    kf_pose.pose.position.x = state.p_map.x();
    kf_pose.pose.position.y = state.p_map.y();
    kf_pose.pose.position.z = state.p_map.z();
    kf_pose.pose.orientation.w = state.q_map_from_base.w();
    kf_pose.pose.orientation.x = state.q_map_from_base.x();
    kf_pose.pose.orientation.y = state.q_map_from_base.y();
    kf_pose.pose.orientation.z = state.q_map_from_base.z();
    keyframe_path_.header.stamp = stamp;
    keyframe_path_.poses.push_back(kf_pose);
    // 슬라이딩 윈도우 크기에 맞게 경로도 트리밍
    const size_t max_path = static_cast<size_t>(node_params_.fgo_backend.window_size) * 2;
    while (keyframe_path_.poses.size() > max_path) {
      keyframe_path_.poses.erase(keyframe_path_.poses.begin());
    }
    keyframe_path_pub_->publish(keyframe_path_);
  }

  // FGO Stage 3+4+5: 그래프 업데이트 + ESKF 앵커 보정 + 진단 카운터
  if (fgo_backend_.is_initialized()) {
    const auto result = fgo_backend_.update(kf, latest_fgo_gnss_);
    latest_fgo_gnss_.reset();  // 사용 후 소비

    if (result.valid) {
      // Stage 4+5: FGO 결과를 ESKF 명목 상태+공분산에 반영 + keyframe 이후 IMU 재적분
      const int n_reint = eskf_corrector_.apply(
        eskf_, result, kf.stamp.seconds(), imu_buffer_);
      ++fgo_correction_count_;

      RCLCPP_DEBUG(
        this->get_logger(),
        "FGO correction #%zu: kf=%lu  p_fgo=(%.2f, %.2f, %.2f)  reint=%d  "
        "P_pos_max=%.4f",
        fgo_correction_count_,
        result.keyframe_index,
        result.state.p_map.x(),
        result.state.p_map.y(),
        result.state.p_map.z(),
        n_reint,
        result.P.block<3, 3>(0, 0).diagonal().maxCoeff());
    }
  }

  // GTSAM 사전적분 리셋 (새 바이어스 추정값 기준)
  if (gtsam_preint_) {
    gtsam_preint_->resetIntegrationAndSetBias(
      gtsam::imuBias::ConstantBias(
        gtsam::Vector3(state.b_a.x(), state.b_a.y(), state.b_a.z()),
        gtsam::Vector3(state.b_g.x(), state.b_g.y(), state.b_g.z())));
  }

  // ESKF 사전적분 리셋
  imu_preint_.reset(state.b_g, state.b_a);
}

bool HybridLocalizationNode::validate_stamp_and_order(
  const rclcpp::Time & current_stamp, const rclcpp::Time & now,
  const rclcpp::Time & last_stamp, const char * label)
{
  auto clock = this->get_clock();
  const auto stamp_result = time_processor_.validate_stamp(current_stamp, now);
  if (stamp_result == StampValidationResult::kInvalidStamp) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *clock, 5000, "%s: Invalid stamp (zero or negative), skipping", label);
    return false;
  }

  const double delay = (now - current_stamp).seconds();
  if (stamp_result == StampValidationResult::kTooOld) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *clock, 5000, "%s: Delay %.3fs exceeds max %.3fs, skipping", label, delay,
      node_params_.time.max_delay);
    return false;
  }

  if (stamp_result == StampValidationResult::kFutureStamp) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *clock, 5000, "%s: Future stamp (%.3fs), skipping", label, delay);
    return false;
  }

  if (time_processor_.validate_order(last_stamp, current_stamp) ==
    OrderValidationResult::kOutOfOrderOrDuplicate)
  {
    const double dt = (current_stamp - last_stamp).seconds();
    if (label != nullptr && std::strcmp(label, "Velocity") == 0 &&
      std::isfinite(dt) && std::abs(dt) <= 1e-12)
    {
      return false;
    }
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *clock, 5000, "%s: Out-of-order or duplicate stamp (dt=%.6fs), skipping", label,
      dt);
    return false;
  }

  return true;
}

void HybridLocalizationNode::reset_imu_dt_stats() {imu_dt_stats_.reset();}

} // namespace hybrid_localization

