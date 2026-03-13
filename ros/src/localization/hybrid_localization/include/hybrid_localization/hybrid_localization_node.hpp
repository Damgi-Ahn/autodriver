#ifndef HYBRID_LOCALIZATION__HYBRID_LOCALIZATION_NODE_HPP_
#define HYBRID_LOCALIZATION__HYBRID_LOCALIZATION_NODE_HPP_

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
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <skyautonet_msgs/msg/gphdt.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tier4_map_msgs/msg/map_projector_info.hpp>

#include <Eigen/Core>

#include "hybrid_localization/eskf/eskf_core.hpp"
#include "hybrid_localization/eskf/gnss_update_handler.hpp"
#include "hybrid_localization/eskf/odom_builder.hpp"
#include "hybrid_localization/fgo/fgo_backend.hpp"
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
  rclcpp::CallbackGroup::SharedPtr m_imu_cb_group_;
  // GNSS/Vehicle/Heading(1-10Hz): 상호 배타적 — 상태 갱신 충돌 방지
  rclcpp::CallbackGroup::SharedPtr m_sensor_cb_group_;
  // 발행 타이머(200Hz): 독립 그룹
  rclcpp::CallbackGroup::SharedPtr m_timer_cb_group_;

  // ---- Publishers ---------------------------------------------------------
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_pose_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_gnss_odom_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    m_gnss_pose_cov_pub;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr m_diag_pub;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_preprocessed_imu_pub;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr
    m_gnss_velocity_pub;

  // ---- Subscribers --------------------------------------------------------
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr m_gnss_sub;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr m_gnss_vel_sub;
  rclcpp::Subscription<tier4_map_msgs::msg::MapProjectorInfo>::SharedPtr
    m_map_projector_sub;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr
    m_velocity_sub;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr
    m_steering_sub;
  rclcpp::Subscription<skyautonet_msgs::msg::Gphdt>::SharedPtr m_heading_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    m_initialpose_sub;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_trigger_node_;

  // ---- TF -----------------------------------------------------------------
  std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

  rclcpp::TimerBase::SharedPtr m_publish_timer;

  // ---- Parameters ---------------------------------------------------------
  HybridLocalizationNodeParams m_node_params;
  TimeProcessor m_time_processor{m_node_params.time};

  // ---- Modules ------------------------------------------------------------
  TfCache m_tf_cache;
  MapProjector m_map_projector;
  GnssPreprocessor m_gnss_preprocessor;
  mutable std::mutex m_heading_arbitrator_mutex;
  GnssHeadingArbitrator m_heading_arbitrator;
  ImuPreprocessor m_imu_preprocessor;
  ImuCalibrationManager m_imu_calibration_manager;
  static constexpr double IMU_CALIBRATION_DURATION_SEC = 30.0;

  // ---- Counters & timestamps ----------------------------------------------
  size_t m_imu_count{0};
  size_t m_gnss_count{0};
  size_t m_gnss_vel_count{0};
  size_t m_velocity_count{0};
  size_t m_steering_count{0};
  bool m_map_projector_received{false};

  rclcpp::Time m_last_imu_stamp;
  rclcpp::Time m_last_gnss_stamp;
  rclcpp::Time m_last_gnss_vel_stamp;
  rclcpp::Time m_last_velocity_stamp;
  rclcpp::Time m_last_steering_stamp;
  rclcpp::Time m_last_heading_stamp;

  ImuDtStats m_imu_dt_stats;

  // ---- Latest sensor buffers ----------------------------------------------
  sensor_msgs::msg::NavSatFix::SharedPtr m_latest_gnss;
  geometry_msgs::msg::TwistStamped::SharedPtr m_latest_gnss_vel;
  autoware_vehicle_msgs::msg::VelocityReport::SharedPtr m_latest_velocity;
  autoware_vehicle_msgs::msg::SteeringReport::SharedPtr m_latest_steering;
  skyautonet_msgs::msg::Gphdt::SharedPtr m_latest_heading;
  double m_latest_heading_yaw_rad{0.0};
  bool m_heading_received{false};

  rclcpp::Time m_eskf_init_stamp_{0, 0, RCL_ROS_TIME};
  static constexpr double k_heading_rate_gate_init_grace_sec_{5.0};
  int m_heading_rate_gate_skip_count_{0};
  rclcpp::Time m_heading_rate_gate_skip_window_start_{0, 0, RCL_ROS_TIME};

  // ---- Diagnostics debug --------------------------------------------------
  double m_last_yaw_meas_var_rad2{0.01};
  double m_last_heading_status_inflate_dbg{1.0};
  double m_last_heading_recover_inflate_dbg{1.0};
  double m_last_heading_yaw_var_pre_nis_dbg{std::numeric_limits<double>::quiet_NaN()};
  double m_last_heading_yaw_var_applied_dbg{std::numeric_limits<double>::quiet_NaN()};
  std::string m_last_heading_yaw_var_source_dbg{"init"};
  EskfYawUpdateDebug m_last_heading_yaw_update_dbg{};

  size_t m_diag_counter{0};
  EskfDiagnosticsPublisher m_diag_builder;

  // ---- Activation control -------------------------------------------------
  bool m_is_activated_{true};
  bool m_use_external_initialpose_{false};

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
  mutable std::mutex m_state_mutex;
  EskfCore m_eskf{m_node_params.eskf};
  rclcpp::Time m_state_stamp{0, 0, RCL_ROS_TIME};

  bool m_pending_init_position{false};
  Eigen::Vector3d m_pending_init_p_map{Eigen::Vector3d::Zero()};
  rclcpp::Time m_pending_init_stamp{0, 0, RCL_ROS_TIME};

  OdomBuilder m_odom_builder;
  GnssUpdateHandler m_gnss_update_handler;
  geometry_msgs::msg::Vector3 m_last_omega_base{};
  bool m_have_last_omega_base{false};
  EskfGnssPosUpdateDebug m_last_gnss_pos_update_dbg{};
  EskfGnssVelUpdateDebug m_last_gnss_vel_update_dbg{};
  double m_last_gnss_pos_recover_inflate_dbg{1.0};
  double m_last_gnss_vel_recover_inflate_dbg{1.0};

  // ---- GNSS recover state -------------------------------------------------
  mutable std::mutex m_gnss_recover_mutex_;
  int m_prev_gnss_status_{std::numeric_limits<int>::min()};
  bool m_gnss_recover_active_{false};
  rclcpp::Time m_gnss_recover_holdoff_until_{0, 0, RCL_ROS_TIME};
  rclcpp::Time m_gnss_recover_pos_last_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time m_gnss_recover_vel_last_stamp_{0, 0, RCL_ROS_TIME};
  double m_gnss_recover_pos_inflate_{1.0};
  double m_gnss_recover_vel_inflate_{1.0};
  rclcpp::Time m_gnss_env_pos_last_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time m_gnss_env_vel_last_stamp_{0, 0, RCL_ROS_TIME};
  double m_gnss_env_pos_inflate_{1.0};
  double m_gnss_env_vel_inflate_{1.0};

  mutable std::mutex m_heading_recover_mutex_;
  int m_prev_heading_status_{std::numeric_limits<int>::min()};
  rclcpp::Time m_heading_recover_holdoff_until_{0, 0, RCL_ROS_TIME};
  rclcpp::Time m_heading_recover_last_stamp_{0, 0, RCL_ROS_TIME};
  double m_heading_recover_inflate_{1.0};
  double m_heading_status_inflate_{1.0};

  // ---- FGO Stage 2: KeyframeBuffer + ImuPreintegration --------------------
  // m_state_mutex 로 보호 (ESKF 상태와 동일 뮤텍스)
  KeyframeBuffer m_keyframe_buffer{20};
  ImuPreintegration m_imu_preint{};    // 진행 중인 사전적분 (현재 ~ 다음 키프레임)

  // ---- FGO Stage 3: GTSAM ISAM2 FGO 백엔드 -------------------------------
  FgoBackend m_fgo_backend;
  // GTSAM IMU 사전적분 (키프레임 사이 raw IMU 누적, FgoBackend 와 공유 params 사용)
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements> m_gtsam_preint_;

  // 최신 FGO 용 GNSS 측정값 (m_state_mutex 보호)
  std::optional<GnssMeasurement> m_latest_fgo_gnss_;

  // 키프레임 생성 후 내부 헬퍼
  void maybe_push_keyframe(
    const rclcpp::Time & stamp,
    const NominalState & state,
    const EskfCore::P15 & P);
};

}  // namespace hybrid_localization

#endif  // HYBRID_LOCALIZATION__HYBRID_LOCALIZATION_NODE_HPP_
