#ifndef HYBRID_LOCALIZATION__TYPES_HPP_
#define HYBRID_LOCALIZATION__TYPES_HPP_

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <string>

namespace hybrid_localization
{

// I/O 토픽 및 프레임 파라미터
struct ESKFParameters
{
  // Topics
  std::string imu_topic{"/sensing/imu/imu_raw"};
  std::string gnss_topic{"/sensing/gnss/navsatfix"};
  std::string gnss_vel_topic{"/sensing/gnss/vel"};
  std::string heading_topic{"/sensing/gnss/heading"};
  std::string map_projector_info_topic{"/map/map_projector_info"};
  std::string velocity_topic{"/vehicle/status/velocity_status"};
  std::string steering_topic{"/vehicle/status/steering_status"};
  std::string output_odom_topic{"/localization/kinematic_state"};

  // Frames
  std::string map_frame{"map"};
  std::string base_frame{"base_link"};

  // Output options
  bool publish_tf{true};
  double publish_rate{200.0};  // [Hz]
};

// 차량 기구학 파라미터
struct VehicleParams
{
  double wheelbase{2.85};  // [m]
};

// 타임스탬프 처리 파라미터
struct TimeProcessingParams
{
  double max_delay{1.0};         // [s] 이 이상 지연된 메시지 스킵
  double future_tolerance{0.1};  // [s] 미래 타임스탬프 허용 한계
  double min_dt{1.0e-4};         // [s] dt 클램프 하한
  double max_dt{0.5};            // [s] dt 클램프 상한
};

// TF 캐시: base_link ↔ sensor 변환
struct ExtrinsicCache
{
  geometry_msgs::msg::TransformStamped base_to_imu{};
  bool imu_valid{false};

  geometry_msgs::msg::TransformStamped base_to_gnss{};
  bool gnss_valid{false};
};

}  // namespace hybrid_localization

#endif  // HYBRID_LOCALIZATION__TYPES_HPP_
