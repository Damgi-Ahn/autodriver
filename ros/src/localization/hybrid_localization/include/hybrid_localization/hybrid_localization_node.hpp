#ifndef HYBRID_LOCALIZATION__HYBRID_LOCALIZATION_NODE_HPP_
#define HYBRID_LOCALIZATION__HYBRID_LOCALIZATION_NODE_HPP_

#include <atomic>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <boost/shared_ptr.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <skyautonet_msgs/msg/gphdt.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tier4_map_msgs/msg/map_projector_info.hpp>

#include <Eigen/Core>

#include "hybrid_localization/eskf/eskf_core.hpp"
#include "hybrid_localization/eskf/gnss_update_handler.hpp"
#include "hybrid_localization/eskf/odom_builder.hpp"
#include "hybrid_localization/fgo/eskf_corrector.hpp"
#include "hybrid_localization/fgo/fgo_backend.hpp"
#include "hybrid_localization/fgo/imu_buffer.hpp"
#include "hybrid_localization/fgo/imu_preintegration.hpp"
#include "hybrid_localization/fgo/keyframe_buffer.hpp"
#include "hybrid_localization/parameters.hpp"
#include "hybrid_localization/preprocess/gnss_heading_arbitrator.hpp"
#include "hybrid_localization/preprocess/gnss_preprocessor.hpp"
#include "hybrid_localization/preprocess/imu_calibration_manager.hpp"
#include "hybrid_localization/preprocess/imu_preprocessor.hpp"
#include "hybrid_localization/util/diagnostics_publisher.hpp"
#include "hybrid_localization/util/map_projector.hpp"
#include "hybrid_localization/util/tf_cache.hpp"
#include "hybrid_localization/util/time_processing.hpp"

namespace hybrid_localization
{

// ---------------------------------------------------------------------------
// LocalizationState: 모듈 동작 상태 (상위 시스템 fault 보고용)
//
//  UNINITIALIZED → INITIALIZING → OPERATING
//                                     ↕  (GNSS 불량 5초 이상)
//                                 DEGRADED
//                                     ↕  (ESKF non-finite 발생 시)
//                                  FAILED → INITIALIZING
// ---------------------------------------------------------------------------
enum class LocalizationState : uint8_t
{
  UNINITIALIZED = 0,  // 미활성화 또는 IMU 캘리브 대기
  INITIALIZING  = 1,  // 첫 GNSS/Heading 대기 중
  OPERATING     = 2,  // 정상 측위 출력 중
  DEGRADED      = 3,  // GNSS 불량/timeout — IMU 단독 추측항법
  FAILED        = 4,  // ESKF non-finite 강제 리셋
};

class HybridLocalizationNode : public rclcpp::Node
{
public:
  explicit HybridLocalizationNode(const rclcpp::NodeOptions & t_options);

private:
  // ---- Callbacks ----------------------------------------------------------
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr t_msg);
  void gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr t_msg);
  void gnss_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr t_msg);
  void map_projector_info_callback(
    const tier4_map_msgs::msg::MapProjectorInfo::SharedPtr t_msg);
  void velocity_callback(
    const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr t_msg);
  void steering_callback(
    const autoware_vehicle_msgs::msg::SteeringReport::SharedPtr t_msg);
  void heading_callback(const skyautonet_msgs::msg::Gphdt::SharedPtr t_msg);
  void initialpose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr t_msg);

  void publish_timer_callback();
  void load_parameters();

  void service_trigger_node(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res);

  // ---- Callback groups (one per frequency domain) -------------------------
  // IMU(200Hz): 전용 그룹 — 다른 콜백에 블록되지 않도록 격리
  rclcpp::CallbackGroup::SharedPtr imu_cb_group_;
  // GNSS/Vehicle/Heading(1-10Hz): 상호 배타적 — 상태 갱신 충돌 방지
  rclcpp::CallbackGroup::SharedPtr sensor_cb_group_;
  // 발행 타이머(200Hz): 독립 그룹
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;

  // ---- Publishers ---------------------------------------------------------
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gnss_odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    gnss_pose_cov_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr preprocessed_imu_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr
    gnss_velocity_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr keyframe_path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

  // ---- Subscribers --------------------------------------------------------
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr gnss_vel_sub_;
  rclcpp::Subscription<tier4_map_msgs::msg::MapProjectorInfo>::SharedPtr
    map_projector_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr
    velocity_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr
    steering_sub_;
  rclcpp::Subscription<skyautonet_msgs::msg::Gphdt>::SharedPtr heading_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initialpose_sub_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_trigger_node_;

  // ---- TF -----------------------------------------------------------------
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr publish_timer_;

  // ---- Parameters ---------------------------------------------------------
  HybridLocalizationNodeParams node_params_;
  TimeProcessor time_processor_{node_params_.time};

  // ---- Modules ------------------------------------------------------------
  TfCache tf_cache_;
  MapProjector map_projector_;
  GnssPreprocessor gnss_preprocessor_;
  mutable std::mutex heading_arbitrator_mutex_;
  GnssHeadingArbitrator heading_arbitrator_;
  ImuPreprocessor imu_preprocessor_;
  ImuCalibrationManager imu_calibration_manager_;

  // ---- Counters & timestamps ----------------------------------------------
  std::atomic<size_t> imu_count_{0};
  std::atomic<size_t> gnss_count_{0};
  std::atomic<size_t> gnss_vel_count_{0};
  std::atomic<size_t> velocity_count_{0};
  std::atomic<size_t> steering_count_{0};
  std::atomic<bool> map_projector_received_{false};

  rclcpp::Time last_imu_stamp_;
  rclcpp::Time last_gnss_stamp_;
  rclcpp::Time last_gnss_vel_stamp_;
  rclcpp::Time last_velocity_stamp_;
  rclcpp::Time last_steering_stamp_;
  rclcpp::Time last_heading_stamp_;

  ImuDtStats imu_dt_stats_;

  // ---- Latest sensor buffers ----------------------------------------------
  sensor_msgs::msg::NavSatFix::SharedPtr latest_gnss_;
  geometry_msgs::msg::TwistStamped::SharedPtr latest_gnss_vel_;
  autoware_vehicle_msgs::msg::VelocityReport::SharedPtr latest_velocity_;
  autoware_vehicle_msgs::msg::SteeringReport::SharedPtr latest_steering_;
  skyautonet_msgs::msg::Gphdt::SharedPtr latest_heading_;
  double latest_heading_yaw_rad_{0.0};
  std::atomic<bool> heading_received_{false};

  rclcpp::Time eskf_init_stamp_{0, 0, RCL_ROS_TIME};
  int heading_rate_gate_skip_count_{0};
  rclcpp::Time heading_rate_gate_skip_window_start_{0, 0, RCL_ROS_TIME};

  // ---- Diagnostics debug --------------------------------------------------
  double last_yaw_meas_var_rad2_{0.01};
  double last_heading_status_inflate_dbg_{1.0};
  double last_heading_recover_inflate_dbg_{1.0};
  double last_heading_yaw_var_pre_nis_dbg_{std::numeric_limits<double>::quiet_NaN()};
  double last_heading_yaw_var_applied_dbg_{std::numeric_limits<double>::quiet_NaN()};
  std::string last_heading_yaw_var_source_dbg_{"init"};
  EskfYawUpdateDebug last_heading_yaw_update_dbg_{};

  size_t diag_counter_{0};
  EskfDiagnosticsPublisher diag_builder_;

  // ---- Activation control -------------------------------------------------
  std::atomic<bool> is_activated_{true};
  bool use_external_initialpose_{false};

  // ---- Localization state machine -----------------------------------------
  // state_mutex_ 와 무관하게 publish timer에서 read-only 접근 가능 (atomic)
  std::atomic<LocalizationState> localization_state_{LocalizationState::UNINITIALIZED};
  // GNSS good 마지막 수신 시각 (gnss_recover_mutex_ 보호)
  rclcpp::Time last_gnss_good_stamp_{0, 0, RCL_ROS_TIME};

  // 상태 전이 + 상위 시스템 fault 보고 (state_pub_ 발행)
  // 반드시 publish_timer_callback() 에서만 호출 (timer_cb_group_)
  void transition_state(LocalizationState new_state);

  // ---- Helper methods -----------------------------------------------------
  void reset_imu_dt_stats();
  bool validate_stamp_and_order(
    const rclcpp::Time & current_stamp,
    const rclcpp::Time & now,
    const rclcpp::Time & last_stamp,
    const char * label);

  enum class GnssRecoverChannel { kPosition, kVelocity };

  void update_gnss_recover_state(int curr_status, const rclcpp::Time & stamp);
  bool compute_gnss_recover_inflate(
    GnssRecoverChannel channel,
    const rclcpp::Time & stamp,
    bool & skip_update,
    double & out_inflate);
  double apply_gnss_inflate_envelope(
    GnssRecoverChannel channel,
    const rclcpp::Time & stamp,
    double target_inflate);
  void update_heading_recover_state(int curr_status, const rclcpp::Time & stamp);
  void raise_heading_recover_inflate(
    const std::string & reason,
    const rclcpp::Time & stamp);
  double compute_heading_measurement_inflate(
    const rclcpp::Time & stamp,
    double * status_inflate = nullptr,
    double * recover_inflate = nullptr);

  // ---- ESKF state ---------------------------------------------------------
  mutable std::mutex state_mutex_;
  EskfCore eskf_{node_params_.eskf};
  rclcpp::Time state_stamp_{0, 0, RCL_ROS_TIME};

  bool pending_init_position_{false};
  Eigen::Vector3d pending_init_p_map_{Eigen::Vector3d::Zero()};
  rclcpp::Time pending_init_stamp_{0, 0, RCL_ROS_TIME};

  OdomBuilder odom_builder_;
  GnssUpdateHandler gnss_update_handler_;
  geometry_msgs::msg::Vector3 last_omega_base_{};
  bool have_last_omega_base_{false};
  EskfGnssPosUpdateDebug last_gnss_pos_update_dbg_{};
  EskfGnssVelUpdateDebug last_gnss_vel_update_dbg_{};
  double last_gnss_pos_recover_inflate_dbg_{1.0};
  double last_gnss_vel_recover_inflate_dbg_{1.0};

  // ---- GNSS recover state -------------------------------------------------
  mutable std::mutex gnss_recover_mutex_;
  int prev_gnss_status_{std::numeric_limits<int>::min()};
  bool gnss_recover_active_{false};
  rclcpp::Time gnss_recover_holdoff_until_{0, 0, RCL_ROS_TIME};
  rclcpp::Time gnss_recover_pos_last_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time gnss_recover_vel_last_stamp_{0, 0, RCL_ROS_TIME};
  double gnss_recover_pos_inflate_{1.0};
  double gnss_recover_vel_inflate_{1.0};
  rclcpp::Time gnss_env_pos_last_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time gnss_env_vel_last_stamp_{0, 0, RCL_ROS_TIME};
  double gnss_env_pos_inflate_{1.0};
  double gnss_env_vel_inflate_{1.0};

  mutable std::mutex heading_recover_mutex_;
  int prev_heading_status_{std::numeric_limits<int>::min()};
  rclcpp::Time heading_recover_holdoff_until_{0, 0, RCL_ROS_TIME};
  rclcpp::Time heading_recover_last_stamp_{0, 0, RCL_ROS_TIME};
  double heading_recover_inflate_{1.0};
  double heading_status_inflate_{1.0};

  // ---- FGO Stage 2: KeyframeBuffer + ImuPreintegration --------------------
  // state_mutex_ 로 보호 (ESKF 상태와 동일 뮤텍스)
  KeyframeBuffer keyframe_buffer_{20};
  ImuPreintegration imu_preint_{};    // 진행 중인 사전적분 (현재 ~ 다음 키프레임)

  // ---- FGO Stage 3: GTSAM ISAM2 FGO 백엔드 -------------------------------
  FgoBackend fgo_backend_;
  // GTSAM IMU 사전적분 (키프레임 사이 raw IMU 누적, FgoBackend 와 공유 params 사용)
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements> gtsam_preint_;

  // 최신 FGO 용 GNSS 측정값 (state_mutex_ 보호)
  std::optional<GnssMeasurement> latest_fgo_gnss_;

  // ---- FGO Stage 4: EskfCorrector + ImuBuffer ----------------------------
  // ImuBuffer: cb_imu 에서 push, maybe_push_keyframe 에서 read (state_mutex_ 보호)
  ImuBuffer imu_buffer_;
  EskfCorrector eskf_corrector_;

  // ---- FGO Stage 5: 진단 카운터 + 경로 시각화 ----------------------------
  std::atomic<size_t> fgo_correction_count_{0};   // 총 FGO 보정 적용 횟수
  std::atomic<size_t> fgo_keyframe_count_{0};     // 총 생성된 키프레임 수
  // 키프레임 경로 (state_mutex_ 보호, maybe_push_keyframe 에서 갱신)
  nav_msgs::msg::Path keyframe_path_;

  // ---- Adaptive FGO window ------------------------------------------------
  // state_mutex_ 로 보호
  int current_fgo_window_size_{0};  // 0 = 초기화 전 (params 로드 후 설정)

  // 키프레임 생성 후 내부 헬퍼
  void maybe_push_keyframe(
    const rclcpp::Time & stamp,
    const NominalState & state,
    const EskfCore::P15 & P);
};

}  // namespace hybrid_localization

#endif  // HYBRID_LOCALIZATION__HYBRID_LOCALIZATION_NODE_HPP_
