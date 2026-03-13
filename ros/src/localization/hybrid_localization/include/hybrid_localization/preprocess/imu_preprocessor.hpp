#ifndef HYBRID_LOCALIZATION__IMU_PREPROCESSOR_HPP_
#define HYBRID_LOCALIZATION__IMU_PREPROCESSOR_HPP_

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <string>
#include <vector>

#include "hybrid_localization/preprocess/ema_filter.hpp"

namespace hybrid_localization
{

// IMU preprocessing parameters
struct ImuPreprocessParams
{
  // LPF cutoff frequencies (Hz)
  // `imu.gyro_lpf_cutoff_hz` [Hz]
  // - ↑: 더 고주파까지 통과(반응성↑), 진동/노이즈 영향↑
  // - ↓: 더 부드럽게(노이즈↓), 반응 지연↑
  double gyro_lpf_cutoff_hz{10.0};  // Angular velocity LPF
  double accel_lpf_cutoff_hz{20.0}; // Linear acceleration LPF

  // Gravity magnitude (m/s^2)
  double gravity_magnitude{9.80665};

  // Enable gravity removal using orientation
  bool enable_gravity_removal{true};

  // Enable orientation calibration (align to eskf_base_link)
  bool enable_orientation_calibration{true};
};

struct ImuCalibrationData
{
  // Gyro bias in imu_link frame (rad/s)
  geometry_msgs::msg::Vector3 gyro_bias{};

  // Accel bias in imu_link frame (m/s^2)
  // NOTE: Does not include gravity (calibrated in level position)
  geometry_msgs::msg::Vector3 accel_bias{};

  // Orientation calibration: transforms from IMU orientation to eskf_base_link
  // When applied: q_calibrated = q_imu * q_orientation_bias_inv
  // So at rest, q_calibrated ≈ identity (0, 0, 0, 1)
  geometry_msgs::msg::Quaternion orientation_bias{};
  bool orientation_bias_valid{false};

  // Gravity vector measured during calibration (in imu_link frame)
  // Used to estimate initial orientation bias
  geometry_msgs::msg::Vector3 measured_gravity{};

  // Temperature at calibration time (Celsius)
  double calibration_temperature{25.0};

  // Number of samples used for calibration
  size_t sample_count{0};

  bool valid{false};
};

enum class ImuPreprocessStatus
{
  kOk = 0,
  kNoCalibration,
  kNonFiniteInput,
  kInvalidOrientation,
};

struct ImuPreprocessResult
{
  // Angular velocity in base_link frame (bias removed, LPF applied)
  geometry_msgs::msg::Vector3 angular_velocity_base{};

  // Linear acceleration in base_link frame (bias removed, gravity removed, LPF applied)
  geometry_msgs::msg::Vector3 linear_acceleration_base{};

  // Calibrated orientation (aligned to eskf_base_link frame)
  // At rest: should be approximately identity (0, 0, 0, 1)
  geometry_msgs::msg::Quaternion orientation_calibrated{};

  // Original angular velocity in imu_link frame (before bias removal)
  geometry_msgs::msg::Vector3 angular_velocity_raw{};

  // Original linear acceleration in imu_link frame (before bias removal)
  geometry_msgs::msg::Vector3 linear_acceleration_raw{};

  // LPF-filtered values before gravity removal (for debugging)
  geometry_msgs::msg::Vector3 angular_velocity_filtered{};
  geometry_msgs::msg::Vector3 linear_acceleration_filtered{};

  // Gravity vector used for removal (in base_link frame)
  geometry_msgs::msg::Vector3 gravity_removed{};
};

class ImuPreprocessor
{
public:
  // Constructor with parameters
  explicit ImuPreprocessor(const ImuPreprocessParams & t_params = ImuPreprocessParams{});

  // Set parameters (resets filters)
  void set_params(const ImuPreprocessParams & t_params);

  // Get current parameters
  const ImuPreprocessParams & params() const {return params_;}

  // Calibration: collect samples for averaging
  void add_calibration_sample(const sensor_msgs::msg::Imu & msg);

  // Finalize calibration by computing mean and saving to file
  bool finalize_calibration(const std::string & calibration_file_path);

  // Load calibration from file
  bool load_calibration(const std::string & calibration_file_path);

  // Preprocess IMU data: remove bias, apply LPF, remove gravity, transform to base_link
  ImuPreprocessStatus preprocess(
    const sensor_msgs::msg::Imu & msg,
    const geometry_msgs::msg::TransformStamped & tf_imu_to_base,
    double dt,
    ImuPreprocessResult & out);

  // Get calibration data (for inspection)
  const ImuCalibrationData & calibration() const {return calibration_;}

  // Get number of collected samples (during calibration)
  size_t sample_count() const {return gyro_samples_.size();}

  // Reset LPF filters
  void reset_filters();

private:
  ImuPreprocessParams params_;
  ImuCalibrationData calibration_;

  // EMA Low-pass filters
  EmaFilterVector3 gyro_lpf_;
  EmaFilterVector3 accel_lpf_;

  // Temporary buffers for calibration
  std::vector<geometry_msgs::msg::Vector3> gyro_samples_;
  std::vector<geometry_msgs::msg::Vector3> accel_samples_;
  std::vector<geometry_msgs::msg::Quaternion> orientation_samples_;


  // ---- Pipeline steps -------------------------------------------------------
  // preprocess() 가 순서대로 호출. 각 단계는 독립적으로 테스트 가능.

  // Step 1: 입력 유한성 검증
  static ImuPreprocessStatus validate_input(const sensor_msgs::msg::Imu & msg);

  // Step 2: gyro/accel 바이어스 제거 (imu_link 프레임)
  static void remove_bias(
    const geometry_msgs::msg::Vector3 & raw_gyro,
    const geometry_msgs::msg::Vector3 & raw_accel,
    const ImuCalibrationData & cal,
    geometry_msgs::msg::Vector3 & gyro_out,
    geometry_msgs::msg::Vector3 & accel_out);

  // Step 3: EMA LPF 적용
  void apply_lpf(
    const geometry_msgs::msg::Vector3 & gyro_debiased,
    const geometry_msgs::msg::Vector3 & accel_debiased,
    double dt,
    geometry_msgs::msg::Vector3 & gyro_out,
    geometry_msgs::msg::Vector3 & accel_out);

  // Step 4: 중력 제거 및 자세 교정
  void remove_gravity_and_calibrate_orientation(
    const sensor_msgs::msg::Imu & msg,
    const geometry_msgs::msg::Vector3 & accel_filtered,
    geometry_msgs::msg::Vector3 & accel_out,
    geometry_msgs::msg::Quaternion & q_calibrated_out) const;

  // Step 5: base_link 프레임으로 좌표 변환
  static void transform_to_base(
    const geometry_msgs::msg::Vector3 & gyro_filtered,
    const geometry_msgs::msg::Vector3 & accel_gravity_removed,
    const geometry_msgs::msg::TransformStamped & tf_imu_to_base,
    geometry_msgs::msg::Vector3 & gyro_base_out,
    geometry_msgs::msg::Vector3 & accel_base_out);

  // ---- Helpers --------------------------------------------------------------

  // Vector3 회전 변환 (TF 이용, 회전만 적용)
  static geometry_msgs::msg::Vector3 transform_vector3(
    const geometry_msgs::msg::Vector3 & vec,
    const geometry_msgs::msg::TransformStamped & transform);

  // 중력 벡터로부터 IMU 자세 바이어스 추정
  geometry_msgs::msg::Quaternion compute_orientation_bias_from_gravity(
    const geometry_msgs::msg::Vector3 & measured_gravity) const;

};

} // namespace hybrid_localization

#endif // HYBRID_LOCALIZATION__IMU_PREPROCESSOR_HPP_
