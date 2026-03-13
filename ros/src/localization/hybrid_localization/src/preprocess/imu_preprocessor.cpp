#include "hybrid_localization/preprocess/imu_preprocessor.hpp"

#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <fstream>
#include <numeric>
#include <sstream>

namespace hybrid_localization
{

ImuPreprocessor::ImuPreprocessor(const ImuPreprocessParams & t_params)
: m_params(t_params),
  m_gyro_lpf(t_params.gyro_lpf_cutoff_hz),
  m_accel_lpf(t_params.accel_lpf_cutoff_hz) {}

void ImuPreprocessor::set_params(const ImuPreprocessParams & t_params)
{
  // 파라미터 갱신 시 필터 상태도 초기화
  m_params = t_params;
  m_gyro_lpf.set_cutoff_freq(t_params.gyro_lpf_cutoff_hz);
  m_accel_lpf.set_cutoff_freq(t_params.accel_lpf_cutoff_hz);
  reset_filters();
}

void ImuPreprocessor::reset_filters()
{
  m_gyro_lpf.reset();
  m_accel_lpf.reset();
}

void ImuPreprocessor::add_calibration_sample(const sensor_msgs::msg::Imu & msg)
{
  // Check for finite values
  if (!std::isfinite(msg.angular_velocity.x) ||
    !std::isfinite(msg.angular_velocity.y) ||
    !std::isfinite(msg.angular_velocity.z) ||
    !std::isfinite(msg.linear_acceleration.x) ||
    !std::isfinite(msg.linear_acceleration.y) ||
    !std::isfinite(msg.linear_acceleration.z))
  {
    return; // Skip invalid samples
  }

  m_gyro_samples.push_back(msg.angular_velocity);
  m_accel_samples.push_back(msg.linear_acceleration);

  // Collect orientation samples if valid
  if (quat_utils::is_valid(msg.orientation)) {
    m_orientation_samples.push_back(msg.orientation);
  }
}

bool ImuPreprocessor::finalize_calibration(
  const std::string & calibration_file_path)
{
  // 수집된 샘플을 평균화하여 캘리브레이션 값 계산
  if (m_gyro_samples.empty() || m_accel_samples.empty()) {
    return false;
  }

  // Compute mean of gyro samples (bias)
  geometry_msgs::msg::Vector3 gyro_mean{};
  for (const auto & sample : m_gyro_samples) {
    gyro_mean.x += sample.x;
    gyro_mean.y += sample.y;
    gyro_mean.z += sample.z;
  }
  const double n_gyro = static_cast<double>(m_gyro_samples.size());
  gyro_mean.x /= n_gyro;
  gyro_mean.y /= n_gyro;
  gyro_mean.z /= n_gyro;

  // Compute mean of accel samples (includes gravity)
  geometry_msgs::msg::Vector3 accel_mean{};
  for (const auto & sample : m_accel_samples) {
    accel_mean.x += sample.x;
    accel_mean.y += sample.y;
    accel_mean.z += sample.z;
  }
  const double n_accel = static_cast<double>(m_accel_samples.size());
  accel_mean.x /= n_accel;
  accel_mean.y /= n_accel;
  accel_mean.z /= n_accel;

  // Store measured gravity (raw accel at rest = gravity in sensor frame)
  m_calibration.measured_gravity = accel_mean;

  // Compute orientation bias from measured gravity
  // This transforms IMU orientation to eskf_base_link frame
  if (m_params.enable_orientation_calibration) {
    m_calibration.orientation_bias =
      compute_orientation_bias_from_gravity(accel_mean);
    m_calibration.orientation_bias_valid = true;
  }

  // For accel bias: remove gravity component
  // In a level, stationary position, accel should measure only gravity
  // accel_bias = measured_accel - expected_gravity
  // If the sensor is perfectly level, expected gravity = (0, 0, g)
  // But we use orientation to determine actual gravity direction
  geometry_msgs::msg::Vector3 expected_gravity{};
  expected_gravity.x = 0.0;
  expected_gravity.y = 0.0;
  expected_gravity.z = m_params.gravity_magnitude;

  // The accel bias is the deviation from expected gravity
  // (assuming vehicle is stationary during calibration)
  m_calibration.accel_bias.x = accel_mean.x - expected_gravity.x;
  m_calibration.accel_bias.y = accel_mean.y - expected_gravity.y;
  m_calibration.accel_bias.z = accel_mean.z - expected_gravity.z;

  // Update calibration
  m_calibration.gyro_bias = gyro_mean;
  m_calibration.sample_count = m_gyro_samples.size();
  m_calibration.calibration_temperature = 25.0;
  m_calibration.valid = true;

  // Save to file
  std::ofstream ofs(calibration_file_path);
  if (!ofs.is_open()) {
    return false;
  }

  ofs << "# IMU Calibration Data (auto-generated)\n";
  ofs << "# Calibration assumes vehicle is STATIONARY and reasonably LEVEL\n";
  ofs << "gyro_bias:\n";
  ofs << "  x: " << gyro_mean.x << "\n";
  ofs << "  y: " << gyro_mean.y << "\n";
  ofs << "  z: " << gyro_mean.z << "\n";
  ofs << "accel_bias:\n";
  ofs << "  x: " << m_calibration.accel_bias.x << "\n";
  ofs << "  y: " << m_calibration.accel_bias.y << "\n";
  ofs << "  z: " << m_calibration.accel_bias.z << "\n";
  ofs << "measured_gravity:\n";
  ofs << "  x: " << accel_mean.x << "\n";
  ofs << "  y: " << accel_mean.y << "\n";
  ofs << "  z: " << accel_mean.z << "\n";
  ofs << "orientation_bias:\n";
  ofs << "  x: " << m_calibration.orientation_bias.x << "\n";
  ofs << "  y: " << m_calibration.orientation_bias.y << "\n";
  ofs << "  z: " << m_calibration.orientation_bias.z << "\n";
  ofs << "  w: " << m_calibration.orientation_bias.w << "\n";
  ofs << "orientation_bias_valid: "
      << (m_calibration.orientation_bias_valid ? "true" : "false") << "\n";
  ofs << "calibration_temperature: " << m_calibration.calibration_temperature
      << "\n";
  ofs << "sample_count: " << m_calibration.sample_count << "\n";
  ofs.close();

  // Clear temporary buffers
  m_gyro_samples.clear();
  m_accel_samples.clear();
  m_orientation_samples.clear();

  // Reset filters for fresh start
  reset_filters();

  return true;
}

bool ImuPreprocessor::load_calibration(
  const std::string & calibration_file_path)
{
  // 파일 기반 캘리브레이션 로딩
  std::ifstream ifs(calibration_file_path);
  if (!ifs.is_open()) {
    return false;
  }

  std::string line;
  std::string current_section;
  geometry_msgs::msg::Vector3 gyro_bias{};
  geometry_msgs::msg::Vector3 accel_bias{};
  geometry_msgs::msg::Vector3 measured_gravity{};
  geometry_msgs::msg::Quaternion orientation_bias{};
  orientation_bias.w = 1.0; // Default identity
  bool orientation_bias_valid = false;
  double calibration_temperature = 25.0;
  size_t sample_count = 0;

  while (std::getline(ifs, line)) {
    // Skip comments and empty lines
    if (line.empty() || line[0] == '#') {
      continue;
    }

    // Trim leading whitespace
    const auto first = line.find_first_not_of(" \t");
    if (first == std::string::npos) {
      continue;
    }
    line = line.substr(first);

    // Check for section headers
    if (line.find("gyro_bias:") == 0) {
      current_section = "gyro_bias";
      continue;
    }
    if (line.find("accel_bias:") == 0) {
      current_section = "accel_bias";
      continue;
    }
    if (line.find("measured_gravity:") == 0) {
      current_section = "measured_gravity";
      continue;
    }
    if (line.find("orientation_bias:") == 0) {
      current_section = "orientation_bias";
      continue;
    }

    // Parse key-value pairs
    const auto colon_pos = line.find(':');
    if (colon_pos == std::string::npos) {
      continue;
    }

    std::string key = line.substr(0, colon_pos);
    std::string value_str = line.substr(colon_pos + 1);

    // Trim key and value
    key.erase(0, key.find_first_not_of(" \t"));
    key.erase(key.find_last_not_of(" \t") + 1);
    value_str.erase(0, value_str.find_first_not_of(" \t"));
    value_str.erase(value_str.find_last_not_of(" \t") + 1);

    if (current_section == "gyro_bias") {
      try {
        double value = std::stod(value_str);
        if (key == "x") {
          gyro_bias.x = value;
        }
        if (key == "y") {
          gyro_bias.y = value;
        }
        if (key == "z") {
          gyro_bias.z = value;
        }
      } catch (const std::exception &) {
        continue;
      }
    } else if (current_section == "accel_bias") {
      try {
        double value = std::stod(value_str);
        if (key == "x") {
          accel_bias.x = value;
        }
        if (key == "y") {
          accel_bias.y = value;
        }
        if (key == "z") {
          accel_bias.z = value;
        }
      } catch (const std::exception &) {
        continue;
      }
    } else if (current_section == "measured_gravity") {
      try {
        double value = std::stod(value_str);
        if (key == "x") {
          measured_gravity.x = value;
        }
        if (key == "y") {
          measured_gravity.y = value;
        }
        if (key == "z") {
          measured_gravity.z = value;
        }
      } catch (const std::exception &) {
        continue;
      }
    } else if (current_section == "orientation_bias") {
      try {
        double value = std::stod(value_str);
        if (key == "x") {
          orientation_bias.x = value;
        }
        if (key == "y") {
          orientation_bias.y = value;
        }
        if (key == "z") {
          orientation_bias.z = value;
        }
        if (key == "w") {
          orientation_bias.w = value;
        }
      } catch (const std::exception &) {
        continue;
      }
    } else {
      // Top-level fields
      if (key == "orientation_bias_valid") {
        orientation_bias_valid = (value_str == "true");
      } else if (key == "calibration_temperature") {
        try {
          calibration_temperature = std::stod(value_str);
        } catch (const std::exception &) {
        }
      } else if (key == "sample_count") {
        try {
          sample_count = static_cast<size_t>(std::stod(value_str));
        } catch (const std::exception &) {
        }
      }
    }
  }

  ifs.close();

  // Update calibration
  m_calibration.gyro_bias = gyro_bias;
  m_calibration.accel_bias = accel_bias;
  m_calibration.measured_gravity = measured_gravity;
  m_calibration.orientation_bias = quat_utils::normalize(orientation_bias);
  m_calibration.orientation_bias_valid = orientation_bias_valid;
  m_calibration.calibration_temperature = calibration_temperature;
  m_calibration.sample_count = sample_count;
  m_calibration.valid = true;

  // Reset filters for fresh start
  reset_filters();

  return true;
}

// ---------------------------------------------------------------------------
// Pipeline step implementations
// ---------------------------------------------------------------------------

ImuPreprocessStatus ImuPreprocessor::validate_input(
  const sensor_msgs::msg::Imu & msg)
{
  if (!std::isfinite(msg.angular_velocity.x) ||
    !std::isfinite(msg.angular_velocity.y) ||
    !std::isfinite(msg.angular_velocity.z) ||
    !std::isfinite(msg.linear_acceleration.x) ||
    !std::isfinite(msg.linear_acceleration.y) ||
    !std::isfinite(msg.linear_acceleration.z))
  {
    return ImuPreprocessStatus::kNonFiniteInput;
  }
  return ImuPreprocessStatus::kOk;
}

void ImuPreprocessor::remove_bias(
  const geometry_msgs::msg::Vector3 & raw_gyro,
  const geometry_msgs::msg::Vector3 & raw_accel,
  const ImuCalibrationData & cal,
  geometry_msgs::msg::Vector3 & gyro_out,
  geometry_msgs::msg::Vector3 & accel_out)
{
  gyro_out.x = raw_gyro.x - cal.gyro_bias.x;
  gyro_out.y = raw_gyro.y - cal.gyro_bias.y;
  gyro_out.z = raw_gyro.z - cal.gyro_bias.z;

  accel_out.x = raw_accel.x - cal.accel_bias.x;
  accel_out.y = raw_accel.y - cal.accel_bias.y;
  accel_out.z = raw_accel.z - cal.accel_bias.z;
}

void ImuPreprocessor::apply_lpf(
  const geometry_msgs::msg::Vector3 & gyro_debiased,
  const geometry_msgs::msg::Vector3 & accel_debiased,
  double dt,
  geometry_msgs::msg::Vector3 & gyro_out,
  geometry_msgs::msg::Vector3 & accel_out)
{
  gyro_out = m_gyro_lpf.update(gyro_debiased, dt);
  accel_out = m_accel_lpf.update(accel_debiased, dt);
}

void ImuPreprocessor::remove_gravity_and_calibrate_orientation(
  const sensor_msgs::msg::Imu & msg,
  const geometry_msgs::msg::Vector3 & accel_filtered,
  geometry_msgs::msg::Vector3 & accel_out,
  geometry_msgs::msg::Quaternion & q_calibrated_out) const
{
  accel_out = accel_filtered;
  q_calibrated_out = quat_utils::identity();

  if (!m_params.enable_gravity_removal) {
    return;
  }

  if (!quat_utils::is_valid(msg.orientation)) {
    // 유효한 자세가 없으면 수평 가정: Z 방향으로만 중력 제거
    accel_out.z -= m_params.gravity_magnitude;
    return;
  }

  // 자세 교정: q_calibrated = q_imu * q_bias_inv
  auto q_imu = quat_utils::normalize(msg.orientation);
  if (m_calibration.orientation_bias_valid && m_params.enable_orientation_calibration) {
    auto q_bias_inv = quat_utils::conjugate(m_calibration.orientation_bias);
    q_calibrated_out = quat_utils::normalize(quat_utils::multiply(q_imu, q_bias_inv));
  } else {
    q_calibrated_out = q_imu;
  }

  // 중력 제거: g_sensor = R^(-1) * [0, 0, g]_world
  geometry_msgs::msg::Vector3 gravity_world{};
  gravity_world.z = m_params.gravity_magnitude;  // ENU: +Z up
  auto q_inv = quat_utils::conjugate(q_calibrated_out);
  const auto gravity_sensor = quat_utils::rotate_vector(q_inv, gravity_world);

  accel_out.x = accel_filtered.x - gravity_sensor.x;
  accel_out.y = accel_filtered.y - gravity_sensor.y;
  accel_out.z = accel_filtered.z - gravity_sensor.z;
}

void ImuPreprocessor::transform_to_base(
  const geometry_msgs::msg::Vector3 & gyro_filtered,
  const geometry_msgs::msg::Vector3 & accel_gravity_removed,
  const geometry_msgs::msg::TransformStamped & tf_imu_to_base,
  geometry_msgs::msg::Vector3 & gyro_base_out,
  geometry_msgs::msg::Vector3 & accel_base_out)
{
  gyro_base_out = transform_vector3(gyro_filtered, tf_imu_to_base);
  accel_base_out = transform_vector3(accel_gravity_removed, tf_imu_to_base);
}

// ---------------------------------------------------------------------------
// Public preprocess() — pipeline steps를 순서대로 호출
// ---------------------------------------------------------------------------
ImuPreprocessStatus ImuPreprocessor::preprocess(
  const sensor_msgs::msg::Imu & msg,
  const geometry_msgs::msg::TransformStamped & tf_imu_to_base,
  double dt,
  ImuPreprocessResult & out)
{
  out = ImuPreprocessResult{};

  if (!m_calibration.valid) {
    return ImuPreprocessStatus::kNoCalibration;
  }

  // Step 1: 입력 검증
  const auto step1_status = validate_input(msg);
  if (step1_status != ImuPreprocessStatus::kOk) {
    return step1_status;
  }
  out.angular_velocity_raw = msg.angular_velocity;
  out.linear_acceleration_raw = msg.linear_acceleration;

  // Step 2: 바이어스 제거
  geometry_msgs::msg::Vector3 gyro_debiased, accel_debiased;
  remove_bias(
    msg.angular_velocity, msg.linear_acceleration, m_calibration,
    gyro_debiased, accel_debiased);

  // Step 3: LPF 적용
  geometry_msgs::msg::Vector3 gyro_filtered, accel_filtered;
  apply_lpf(gyro_debiased, accel_debiased, dt, gyro_filtered, accel_filtered);
  out.angular_velocity_filtered = gyro_filtered;
  out.linear_acceleration_filtered = accel_filtered;

  // Step 4: 중력 제거 및 자세 교정
  geometry_msgs::msg::Vector3 accel_no_gravity;
  remove_gravity_and_calibrate_orientation(
    msg, accel_filtered, accel_no_gravity, out.orientation_calibrated);

  // Step 5: base_link 변환
  transform_to_base(
    gyro_filtered, accel_no_gravity, tf_imu_to_base,
    out.angular_velocity_base, out.linear_acceleration_base);

  return ImuPreprocessStatus::kOk;
}

geometry_msgs::msg::Vector3 ImuPreprocessor::transform_vector3(
  const geometry_msgs::msg::Vector3 & vec,
  const geometry_msgs::msg::TransformStamped & transform)
{
  geometry_msgs::msg::Vector3Stamped vec_stamped;
  vec_stamped.vector = vec;

  geometry_msgs::msg::Vector3Stamped vec_stamped_transformed;
  tf2::doTransform(vec_stamped, vec_stamped_transformed, transform);
  return vec_stamped_transformed.vector;
}

geometry_msgs::msg::Quaternion ImuPreprocessor::compute_orientation_bias_from_gravity(
  const geometry_msgs::msg::Vector3 & measured_gravity) const
{
  // 중력 벡터로부터 IMU 자세 바이어스를 추정
  // Compute orientation bias from measured gravity vector
  //
  // When the vehicle is stationary on level ground:
  // - Expected gravity in sensor frame (if aligned with base_link): [0, 0, g]
  // - Measured gravity: [gx, gy, gz]
  //
  // The orientation bias is the rotation that transforms measured_gravity to [0, 0, g]
  //
  // We compute this as a rotation from the gravity vector to the Z-axis
  // using the axis-angle representation

  const double gx = measured_gravity.x;
  const double gy = measured_gravity.y;
  const double gz = measured_gravity.z;

  const double g_norm = std::sqrt(gx * gx + gy * gy + gz * gz);

  if (g_norm < 1e-6) {
    // Invalid gravity measurement, return identity
    return quat_utils::identity();
  }

  // Normalize gravity vector
  const double nx = gx / g_norm;
  const double ny = gy / g_norm;
  const double nz = gz / g_norm;

  // Target vector: [0, 0, 1] (gravity should point in +Z direction)
  const double tx = 0.0;
  const double ty = 0.0;
  const double tz = 1.0;

  // Compute rotation axis: n × t
  const double ax = ny * tz - nz * ty;
  const double ay = nz * tx - nx * tz;
  const double az = nx * ty - ny * tx;

  const double sin_angle = std::sqrt(ax * ax + ay * ay + az * az);
  const double cos_angle = nx * tx + ny * ty + nz * tz; // n · t

  geometry_msgs::msg::Quaternion q;

  if (sin_angle < 1e-6) {
    // Vectors are parallel
    if (cos_angle > 0) {
      // Same direction, no rotation needed
      return quat_utils::identity();
    } else {
      // Opposite direction, rotate 180° around any perpendicular axis
      // Choose X or Y axis based on which is more perpendicular to Z
      q.x = 1.0;
      q.y = 0.0;
      q.z = 0.0;
      q.w = 0.0;
      return q;
    }
  }

  // Normalize axis
  const double axis_x = ax / sin_angle;
  const double axis_y = ay / sin_angle;
  const double axis_z = az / sin_angle;

  // Compute quaternion from axis-angle
  // angle = atan2(sin_angle, cos_angle)
  const double angle = std::atan2(sin_angle, cos_angle);
  const double half_angle = angle / 2.0;
  const double sin_half = std::sin(half_angle);
  const double cos_half = std::cos(half_angle);

  q.x = axis_x * sin_half;
  q.y = axis_y * sin_half;
  q.z = axis_z * sin_half;
  q.w = cos_half;

  return quat_utils::normalize(q);
}

} // namespace hybrid_localization
