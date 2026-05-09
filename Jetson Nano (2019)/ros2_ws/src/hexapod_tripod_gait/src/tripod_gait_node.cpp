#include "hexapod_tripod_gait/tripod_gait_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace hexapod_tripod_gait
{

TripodGaitNode::TripodGaitNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("hexapod_tripod_gait", options)
{
  declare_and_load_parameters();

  auto motor_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  motor_qos.best_effort();
  motor_qos.durability_volatile();

  if (command_output_mode_is_phase()) {
    phase_command_publisher_ = create_publisher<LegPhaseCommand>(phase_command_topic_, rclcpp::QoS(rclcpp::KeepLast(1)));
  } else {
    rpm_publisher_ = create_publisher<MotorRpmArray>(rpm_command_topic_, motor_qos);
  }

  if (use_compact_odometry_feedback_ && !odometry_topic_.empty()) {
    odometry_subscription_ = create_subscription<MotorOutputOdometryArray>(
      odometry_topic_,
      motor_qos,
      std::bind(&TripodGaitNode::odometry_callback, this, std::placeholders::_1));
  }

  if (use_motor_state_feedback_ && !motor_state_topic_.empty()) {
    motor_state_subscription_ = create_subscription<HexapodMotorState>(
      motor_state_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&TripodGaitNode::motor_state_callback, this, std::placeholders::_1));
  }

  if (!cmd_vel_topic_.empty()) {
    twist_subscription_ = create_subscription<Twist>(
      cmd_vel_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&TripodGaitNode::twist_callback, this, std::placeholders::_1));
  }

  if (!cmd_vel_stamped_topic_.empty()) {
    twist_stamped_subscription_ = create_subscription<TwistStamped>(
      cmd_vel_stamped_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&TripodGaitNode::twist_stamped_callback, this, std::placeholders::_1));
  }

  if (imu_feedback_enabled_ && !imu_topic_.empty()) {
    imu_subscription_ = create_subscription<Imu>(
      imu_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      std::bind(&TripodGaitNode::imu_callback, this, std::placeholders::_1));
  }

  if (euler_feedback_enabled_ && !euler_topic_.empty()) {
    euler_subscription_ = create_subscription<Vector3Stamped>(
      euler_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      std::bind(&TripodGaitNode::euler_callback, this, std::placeholders::_1));
  }

  if (euler_feedback_enabled_ && !euler_vector_topic_.empty()) {
    euler_vector_subscription_ = create_subscription<Vector3>(
      euler_vector_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      std::bind(&TripodGaitNode::euler_vector_callback, this, std::placeholders::_1));
  }

  const auto publish_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / publish_rate_hz_));
  publish_timer_ = create_wall_timer(
    publish_period,
    std::bind(&TripodGaitNode::publish_timer_callback, this));

  RCLCPP_INFO(
    get_logger(),
    "tripod gait ready cmd_vel='%s' cmd_vel_stamped='%s' compact_odom='%s' motor_state='%s' output_mode='%s' rpm='%s' phase_cmd='%s' imu='%s' euler='%s' euler_vector='%s' rate=%.2f Hz latch_last_teleop_command=%s publish_unchanged_commands=%s use_motor_state_feedback=%s use_compact_odometry_feedback=%s imu_feedback=%s euler_feedback=%s linear_accel_feedback=%s order=[RB,RM,RF,LF,LM,LB] motor_state_order=[RF,RM,RB,LF,LM,LB] tripod_a=[RM,LF,LB] tripod_b=[RB,RF,LM]",
    cmd_vel_topic_.c_str(),
    cmd_vel_stamped_topic_.c_str(),
    odometry_topic_.c_str(),
    motor_state_topic_.c_str(),
    command_output_mode_.c_str(),
    rpm_command_topic_.c_str(),
    phase_command_topic_.c_str(),
    imu_topic_.c_str(),
    euler_topic_.c_str(),
    euler_vector_topic_.c_str(),
    publish_rate_hz_,
    latch_last_teleop_command_ ? "true" : "false",
    publish_unchanged_commands_ ? "true" : "false",
    use_motor_state_feedback_ ? "true" : "false",
    use_compact_odometry_feedback_ ? "true" : "false",
    imu_feedback_enabled_ ? "true" : "false",
    euler_feedback_enabled_ ? "true" : "false",
    linear_accel_feedback_enabled_ ? "true" : "false");
}

void TripodGaitNode::declare_and_load_parameters()
{
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
  cmd_vel_stamped_topic_ = declare_parameter<std::string>("cmd_vel_stamped_topic", "cmd_vel_stamped");
  odometry_topic_ = declare_parameter<std::string>("odometry_topic", "motor_output_odom");
  motor_state_topic_ = declare_parameter<std::string>("motor_state_topic", "/hexapod/motor_state");
  rpm_command_topic_ = declare_parameter<std::string>("rpm_command_topic", "motor_rpm_cmd");
  phase_command_topic_ = declare_parameter<std::string>("phase_command_topic", "hexapod/phase_cmd");
  command_output_mode_ = declare_parameter<std::string>("command_output_mode", "rpm");
  if (command_output_mode_ != "rpm" && command_output_mode_ != "phase") {
    RCLCPP_WARN(
      get_logger(),
      "command_output_mode must be 'rpm' or 'phase'; requested '%s', using 'rpm'",
      command_output_mode_.c_str());
    command_output_mode_ = "rpm";
  }
  imu_topic_ = declare_parameter<std::string>("imu_topic", "imu/data_raw");
  euler_topic_ = declare_parameter<std::string>("euler_topic", "imu/euler");
  euler_vector_topic_ = declare_parameter<std::string>("euler_vector_topic", "");

  const double requested_publish_rate_hz = declare_parameter<double>("publish_rate_hz", 10.0);
  publish_rate_hz_ = clamp_double(requested_publish_rate_hz, 0.1, 10.0);
  if (std::abs(requested_publish_rate_hz - publish_rate_hz_) > 1e-9) {
    RCLCPP_WARN(
      get_logger(),
      "publish_rate_hz %.3f requested; clamped to %.3f to keep RPM commands <= 10 Hz",
      requested_publish_rate_hz,
      publish_rate_hz_);
  }

  linear_rpm_per_mps_ = declare_parameter<double>("linear_rpm_per_mps", 1200.0);
  yaw_rpm_per_radps_ = declare_parameter<double>("yaw_rpm_per_radps", 600.0);
  max_motor_rpm_ = std::max(0.0, declare_parameter<double>("max_motor_rpm", 1500.0));
  rpm_rate_limit_per_sec_ = std::max(0.0, declare_parameter<double>("rpm_rate_limit_per_sec", 0.0));
  gear_ratio_ = std::max(1.0, declare_parameter<double>("gear_ratio", 50.0));
  linear_deadband_mps_ = std::max(0.0, declare_parameter<double>("linear_deadband_mps", 0.01));
  angular_deadband_radps_ = std::max(0.0, declare_parameter<double>("angular_deadband_radps", 0.01));
  command_timeout_sec_ = std::max(0.0, declare_parameter<double>("command_timeout_sec", 0.30));
  latch_last_teleop_command_ = declare_parameter<bool>("latch_last_teleop_command", true);
  odometry_timeout_sec_ = std::max(0.0, declare_parameter<double>("odometry_timeout_sec", 0.50));
  imu_timeout_sec_ = std::max(0.0, declare_parameter<double>("imu_timeout_sec", 0.35));
  euler_timeout_sec_ = std::max(0.0, declare_parameter<double>("euler_timeout_sec", 0.35));
  phase_kp_rpm_per_rad_ = std::max(0.0, declare_parameter<double>("phase_kp_rpm_per_rad", 25.0));
  max_phase_correction_rpm_ = std::max(0.0, declare_parameter<double>("max_phase_correction_rpm", 100.0));
  phase_lock_min_linear_rpm_ = std::max(0.0, declare_parameter<double>("phase_lock_min_linear_rpm", 10.0));
  yaw_rate_feedback_kp_ = std::max(0.0, declare_parameter<double>("yaw_rate_feedback_kp", 0.0));
  max_yaw_rate_correction_radps_ = std::max(
    0.0,
    declare_parameter<double>("max_yaw_rate_correction_radps", 0.35));
  linear_velocity_feedback_kp_ = std::max(0.0, declare_parameter<double>("linear_velocity_feedback_kp", 0.0));
  max_linear_velocity_correction_mps_ = std::max(
    0.0,
    declare_parameter<double>("max_linear_velocity_correction_mps", 0.20));
  accel_deadband_mps2_ = std::max(0.0, declare_parameter<double>("accel_deadband_mps2", 0.10));
  accel_velocity_leak_tau_sec_ = std::max(0.0, declare_parameter<double>("accel_velocity_leak_tau_sec", 1.50));
  max_estimated_linear_velocity_mps_ = std::max(
    0.0,
    declare_parameter<double>("max_estimated_linear_velocity_mps", 1.00));
  roll_soft_limit_rad_ = std::max(0.0, declare_parameter<double>("roll_soft_limit_rad", 0.35));
  roll_hard_limit_rad_ = std::max(roll_soft_limit_rad_, declare_parameter<double>("roll_hard_limit_rad", 0.70));
  pitch_soft_limit_rad_ = std::max(0.0, declare_parameter<double>("pitch_soft_limit_rad", 0.35));
  pitch_hard_limit_rad_ = std::max(pitch_soft_limit_rad_, declare_parameter<double>("pitch_hard_limit_rad", 0.70));
  phase_lock_enabled_ = declare_parameter<bool>("phase_lock_enabled", true);
  phase_lock_during_pure_turn_ = declare_parameter<bool>("phase_lock_during_pure_turn", false);
  require_odometry_for_motion_ = declare_parameter<bool>("require_odometry_for_motion", false);
  use_motor_state_feedback_ = declare_parameter<bool>("use_motor_state_feedback", true);
  use_compact_odometry_feedback_ = declare_parameter<bool>("use_compact_odometry_feedback", false);
  motor_state_require_comm_ok_ = declare_parameter<bool>("motor_state_require_comm_ok", false);
  motor_state_require_fresh_ = declare_parameter<bool>("motor_state_require_fresh", false);
  publish_unchanged_commands_ = declare_parameter<bool>("publish_unchanged_commands", false);
  imu_feedback_enabled_ = declare_parameter<bool>("imu_feedback_enabled", true);
  euler_feedback_enabled_ = declare_parameter<bool>("euler_feedback_enabled", true);
  yaw_rate_feedback_enabled_ = declare_parameter<bool>("yaw_rate_feedback_enabled", true);
  linear_accel_feedback_enabled_ = declare_parameter<bool>("linear_accel_feedback_enabled", false);
  tilt_safety_enabled_ = declare_parameter<bool>("tilt_safety_enabled", true);
  use_raw_imu_orientation_ = declare_parameter<bool>("use_raw_imu_orientation", true);
  euler_angles_are_degrees_ = declare_parameter<bool>("euler_angles_are_degrees", false);
  phase_command_use_phase_velocity_ = declare_parameter<bool>("phase_command_use_phase_velocity", true);
  phase_command_use_motor_rpm_ff_ = declare_parameter<bool>("phase_command_use_motor_rpm_ff", false);
  const int requested_linear_accel_axis = declare_parameter<int>("linear_accel_axis", 0);
  if (requested_linear_accel_axis < 0 || requested_linear_accel_axis > 2) {
    RCLCPP_WARN(
      get_logger(),
      "linear_accel_axis must be 0=x, 1=y, or 2=z; requested %d, using x",
      requested_linear_accel_axis);
    linear_accel_axis_ = 0;
  } else {
    linear_accel_axis_ = requested_linear_accel_axis;
  }
  linear_accel_sign_ = declare_parameter<double>("linear_accel_sign", 1.0) < 0.0 ? -1.0 : 1.0;

  declare_parameter<std::vector<double>>(
    "forward_motor_signs",
    std::vector<double>{1.0, 1.0, 1.0, -1.0, -1.0, -1.0});
  declare_parameter<std::vector<double>>(
    "phase_direction_signs",
    std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
  declare_parameter<std::vector<double>>(
    "phase_correction_motor_signs",
    std::vector<double>{-1.0, -1.0, -1.0, 1.0, 1.0, 1.0});
  declare_parameter<std::vector<double>>(
    "yaw_side_signs",
    std::vector<double>{1.0, 1.0, 1.0, -1.0, -1.0, -1.0});
  declare_parameter<std::vector<double>>(
    "phase_offsets_rad",
    std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  declare_parameter<std::vector<double>>(
    "euler_axis_signs",
    std::vector<double>{1.0, 1.0, 1.0});
  declare_parameter<std::vector<double>>(
    "model_to_physical_motor_signs",
    std::vector<double>{-1.0, -1.0, -1.0, 1.0, 1.0, 1.0});
  declare_parameter<std::vector<int64_t>>(
    "internal_to_motor_state_index",
    std::vector<int64_t>{2, 1, 0, 3, 4, 5});
  declare_parameter<std::vector<int64_t>>(
    "internal_to_phase_command_index",
    std::vector<int64_t>{2, 1, 0, 3, 4, 5});

  forward_motor_signs_ = get_sign_array_parameter("forward_motor_signs", forward_motor_signs_);
  phase_direction_signs_ = get_sign_array_parameter("phase_direction_signs", phase_direction_signs_);
  phase_correction_motor_signs_ =
    get_sign_array_parameter("phase_correction_motor_signs", phase_correction_motor_signs_);
  yaw_side_signs_ = get_sign_array_parameter("yaw_side_signs", yaw_side_signs_);
  phase_offsets_rad_ = get_double_array_parameter("phase_offsets_rad", phase_offsets_rad_);
  model_to_physical_motor_signs_ =
    get_sign_array_parameter("model_to_physical_motor_signs", model_to_physical_motor_signs_);
  internal_to_motor_state_index_ =
    get_index_array_parameter("internal_to_motor_state_index", internal_to_motor_state_index_);
  internal_to_phase_command_index_ =
    get_index_array_parameter("internal_to_phase_command_index", internal_to_phase_command_index_);

  const auto euler_axis_values = get_parameter("euler_axis_signs").as_double_array();
  if (euler_axis_values.size() == euler_axis_signs_.size()) {
    for (std::size_t index = 0U; index < euler_axis_signs_.size(); index++) {
      euler_axis_signs_[index] = (euler_axis_values[index] < 0.0) ? -1.0 : 1.0;
    }
  } else {
    RCLCPP_WARN(get_logger(), "parameter 'euler_axis_signs' must have exactly 3 values; using defaults");
  }
}

std::array<double, TripodGaitNode::motor_count> TripodGaitNode::get_double_array_parameter(
  const std::string & name,
  const std::array<double, motor_count> & fallback)
{
  const auto values = get_parameter(name).as_double_array();
  if (values.size() != motor_count) {
    RCLCPP_WARN(
      get_logger(),
      "parameter '%s' must have exactly %zu values; using fallback",
      name.c_str(),
      motor_count);
    return fallback;
  }

  std::array<double, motor_count> result{};
  for (std::size_t index = 0U; index < motor_count; index++) {
    result[index] = values[index];
  }
  return result;
}

std::array<double, TripodGaitNode::motor_count> TripodGaitNode::get_sign_array_parameter(
  const std::string & name,
  const std::array<double, motor_count> & fallback)
{
  const auto values = get_double_array_parameter(name, fallback);
  std::array<double, motor_count> result{};
  for (std::size_t index = 0U; index < motor_count; index++) {
    if (values[index] > 0.0) {
      result[index] = 1.0;
    } else if (values[index] < 0.0) {
      result[index] = -1.0;
    } else {
      RCLCPP_WARN(
        get_logger(),
        "parameter '%s' index %zu is zero; using fallback sign %.0f",
        name.c_str(),
        index,
        fallback[index]);
      result[index] = fallback[index];
    }
  }
  return result;
}

std::array<std::size_t, TripodGaitNode::motor_count> TripodGaitNode::get_index_array_parameter(
  const std::string & name,
  const std::array<std::size_t, motor_count> & fallback)
{
  const auto values = get_parameter(name).as_integer_array();
  if (values.size() != motor_count) {
    RCLCPP_WARN(
      get_logger(),
      "parameter '%s' must have exactly %zu values; using fallback",
      name.c_str(),
      motor_count);
    return fallback;
  }

  std::array<std::size_t, motor_count> result{};
  std::array<bool, motor_count> seen{};
  for (std::size_t index = 0U; index < motor_count; index++) {
    if (values[index] < 0 || values[index] >= static_cast<int64_t>(motor_count)) {
      RCLCPP_WARN(
        get_logger(),
        "parameter '%s' index %zu=%ld is outside [0,%zu); using fallback",
        name.c_str(),
        index,
        static_cast<long>(values[index]),
        motor_count);
      return fallback;
    }
    const auto mapped_index = static_cast<std::size_t>(values[index]);
    if (seen[mapped_index]) {
      RCLCPP_WARN(
        get_logger(),
        "parameter '%s' repeats mapped index %zu; using fallback",
        name.c_str(),
        mapped_index);
      return fallback;
    }
    result[index] = mapped_index;
    seen[mapped_index] = true;
  }

  return result;
}

void TripodGaitNode::twist_callback(const Twist::SharedPtr message)
{
  latest_linear_x_mps_ = std::abs(message->linear.x) < linear_deadband_mps_ ? 0.0 : message->linear.x;
  latest_angular_z_radps_ = std::abs(message->angular.z) < angular_deadband_radps_ ? 0.0 : message->angular.z;
  have_command_ = true;
  last_command_time_ = std::chrono::steady_clock::now();
}

void TripodGaitNode::twist_stamped_callback(const TwistStamped::SharedPtr message)
{
  latest_linear_x_mps_ = std::abs(message->twist.linear.x) < linear_deadband_mps_ ? 0.0 : message->twist.linear.x;
  latest_angular_z_radps_ = std::abs(message->twist.angular.z) < angular_deadband_radps_ ? 0.0 : message->twist.angular.z;
  have_command_ = true;
  last_command_time_ = std::chrono::steady_clock::now();
}

void TripodGaitNode::odometry_callback(const MotorOutputOdometryArray::SharedPtr message)
{
  for (std::size_t index = 0U; index < motor_count; index++) {
    latest_output_phase_rad_[index] = static_cast<double>(message->output_phase_rad[index]);
  }
  have_odometry_ = true;
  last_odometry_time_ = std::chrono::steady_clock::now();
}

void TripodGaitNode::motor_state_callback(const HexapodMotorState::SharedPtr message)
{
  if (message->phase_rad.size() < motor_count ||
      message->comm_ok.size() < motor_count ||
      message->fresh.size() < motor_count)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "hexapod motor_state arrays are incomplete; holding previous phase feedback");
    return;
  }

  bool accepted = true;
  bool any_comm_bad = false;
  bool any_not_fresh = false;
  for (std::size_t internal_index = 0U; internal_index < motor_count; internal_index++) {
    const auto state_index = internal_to_motor_state_index_[internal_index];
    if (!message->comm_ok[state_index]) {
      any_comm_bad = true;
      if (motor_state_require_comm_ok_) {
        accepted = false;
        break;
      }
    }
    if (!message->fresh[state_index]) {
      any_not_fresh = true;
      if (motor_state_require_fresh_) {
        accepted = false;
        break;
      }
    }
  }

  if (!accepted) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "hexapod motor_state comm/fresh check failed; holding previous phase feedback");
    return;
  }

  // When strict checks are disabled, accept the canonical phase estimate silently.
  // Some firmware/aggregator combinations keep comm/fresh flags low even while
  // phase arrays are valid and updating; warning here creates persistent log spam.
  (void)any_comm_bad;
  (void)any_not_fresh;

  for (std::size_t internal_index = 0U; internal_index < motor_count; internal_index++) {
    const auto state_index = internal_to_motor_state_index_[internal_index];
    latest_output_phase_rad_[internal_index] = static_cast<double>(message->phase_rad[state_index]);
  }

  have_odometry_ = true;
  last_odometry_time_ = std::chrono::steady_clock::now();
}

void TripodGaitNode::imu_callback(const Imu::SharedPtr message)
{
  const auto now = std::chrono::steady_clock::now();
  latest_yaw_rate_radps_ = static_cast<double>(message->angular_velocity.z);
  latest_linear_accel_x_mps2_ = static_cast<double>(message->linear_acceleration.x);
  latest_linear_accel_y_mps2_ = static_cast<double>(message->linear_acceleration.y);
  latest_linear_accel_z_mps2_ = static_cast<double>(message->linear_acceleration.z);
  update_linear_velocity_estimate(*message, now);
  have_imu_ = true;
  last_imu_time_ = now;

  const double qx = static_cast<double>(message->orientation.x);
  const double qy = static_cast<double>(message->orientation.y);
  const double qz = static_cast<double>(message->orientation.z);
  const double qw = static_cast<double>(message->orientation.w);
  const double quaternion_norm = std::sqrt((qx * qx) + (qy * qy) + (qz * qz) + (qw * qw));

  if (use_raw_imu_orientation_ && message->orientation_covariance[0] >= 0.0 && quaternion_norm > 1e-6) {
    double roll_rad = 0.0;
    double pitch_rad = 0.0;
    double yaw_rad = 0.0;
    quaternion_to_roll_pitch_yaw(
      qx / quaternion_norm,
      qy / quaternion_norm,
      qz / quaternion_norm,
      qw / quaternion_norm,
      roll_rad,
      pitch_rad,
      yaw_rad);
    update_euler_measurement(roll_rad, pitch_rad, yaw_rad);
  }
}

void TripodGaitNode::euler_callback(const Vector3Stamped::SharedPtr message)
{
  update_euler_measurement_from_vector(message->vector);
}

void TripodGaitNode::euler_vector_callback(const Vector3::SharedPtr message)
{
  update_euler_measurement_from_vector(*message);
}

void TripodGaitNode::publish_timer_callback()
{
  const auto now = std::chrono::steady_clock::now();
  const bool command_recent = is_command_recent(now);
  const bool odometry_recent = is_odometry_recent(now);
  const bool command_active = command_recent && (!require_odometry_for_motion_ || odometry_recent);

  if (require_odometry_for_motion_ && command_recent && !odometry_recent) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "fresh cmd_vel received, but odometry is stale/missing; publishing zero RPM because require_odometry_for_motion=true");
  }

  const auto adjusted_command = get_feedback_adjusted_command(now);
  const auto target_motor_rpm = compute_unlimited_motor_rpm(command_active, adjusted_command);
  const bool immediate_stop = !command_active || is_effectively_zero_command();
  const auto motor_rpm = limit_and_quantize_motor_rpm(target_motor_rpm, immediate_stop);

  if (command_output_mode_is_phase()) {
    /*
     * Phase-command mode intentionally publishes ROS keepalives at the timer
     * rate even when the RPM vector is unchanged. The phase_command_router_node
     * uses command freshness internally, then suppresses unchanged /motor_rpm_cmd
     * writes before the ESP/RS485 bus.
     */
    publish_phase_command(motor_rpm);
    last_published_motor_rpm_ = motor_rpm;
    have_published_output_ = true;
    last_publish_time_ = now;
    return;
  }

  if (!publish_unchanged_commands_ && have_published_output_ && motor_rpm == last_published_motor_rpm_) {
    return;
  }

  publish_rpm_command(motor_rpm);

  last_published_motor_rpm_ = motor_rpm;
  have_published_output_ = true;
  last_publish_time_ = now;
}

void TripodGaitNode::publish_rpm_command(const std::array<int16_t, motor_count> & motor_rpm)
{
  if (!rpm_publisher_) {
    return;
  }

  MotorRpmArray message;
  message.command_seq = ++command_seq_;
  message.motor_rpm = motor_rpm;
  rpm_publisher_->publish(message);
}

void TripodGaitNode::publish_phase_command(const std::array<int16_t, motor_count> & motor_rpm)
{
  if (!phase_command_publisher_) {
    return;
  }

  LegPhaseCommand message;
  message.header.stamp = get_clock()->now();
  message.use_phase_targets = false;
  message.use_phase_velocity = phase_command_use_phase_velocity_;
  message.use_motor_rpm_ff = phase_command_use_motor_rpm_ff_;
  message.gait_mode = 1U;

  for (std::size_t internal_index = 0U; internal_index < motor_count; internal_index++) {
    const auto command_index = internal_to_phase_command_index_[internal_index];
    const double physical_motor_rpm = static_cast<double>(motor_rpm[internal_index]);
    const double model_sign =
      model_to_physical_motor_signs_[command_index] >= 0.0 ? 1.0 : -1.0;
    const double model_motor_rpm = model_sign * physical_motor_rpm;

    message.motor_id[command_index] = static_cast<uint8_t>(command_index + 1U);
    message.phase_target_rad[command_index] = 0.0f;
    message.phase_velocity_rad_s[command_index] =
      static_cast<float>(motor_rpm_to_output_phase_velocity(model_motor_rpm));
    message.motor_rpm_ff[command_index] = static_cast<float>(model_motor_rpm);
  }

  phase_command_publisher_->publish(message);
}

double TripodGaitNode::motor_rpm_to_output_phase_velocity(double motor_rpm) const
{
  return motor_rpm * (2.0 * pi) / (60.0 * std::max(1.0, gear_ratio_));
}

bool TripodGaitNode::command_output_mode_is_phase() const
{
  return command_output_mode_ == "phase";
}

TripodGaitNode::PlanarCommand TripodGaitNode::get_feedback_adjusted_command(SteadyTime now) const
{
  PlanarCommand command;
  command.linear_x_mps = latest_linear_x_mps_;
  command.angular_z_radps = latest_angular_z_radps_;

  if (imu_feedback_enabled_ && yaw_rate_feedback_enabled_ && is_imu_recent(now) &&
    yaw_rate_feedback_kp_ > 0.0)
  {
    const double yaw_rate_error_radps = latest_angular_z_radps_ - latest_yaw_rate_radps_;
    const double yaw_correction_radps = clamp_double(
      yaw_rate_feedback_kp_ * yaw_rate_error_radps,
      -max_yaw_rate_correction_radps_,
      max_yaw_rate_correction_radps_);
    command.angular_z_radps += yaw_correction_radps;
  }

  if (imu_feedback_enabled_ && linear_accel_feedback_enabled_ && is_imu_recent(now) &&
    have_accel_velocity_estimate_ && linear_velocity_feedback_kp_ > 0.0)
  {
    const double linear_velocity_error_mps = latest_linear_x_mps_ - estimated_linear_velocity_mps_;
    const double linear_velocity_correction_mps = clamp_double(
      linear_velocity_feedback_kp_ * linear_velocity_error_mps,
      -max_linear_velocity_correction_mps_,
      max_linear_velocity_correction_mps_);
    command.linear_x_mps += linear_velocity_correction_mps;
  }

  double roll_rad = 0.0;
  double pitch_rad = 0.0;
  if (tilt_safety_enabled_ && get_latest_roll_pitch(now, roll_rad, pitch_rad)) {
    const double tilt_scale = compute_tilt_scale(roll_rad, pitch_rad);
    command.linear_x_mps *= tilt_scale;
    command.angular_z_radps *= tilt_scale;
  }

  return command;
}

std::array<double, TripodGaitNode::motor_count> TripodGaitNode::compute_unlimited_motor_rpm(
  bool command_active,
  const PlanarCommand & command) const
{
  std::array<double, motor_count> target_motor_rpm{};
  if (!command_active) {
    return target_motor_rpm;
  }

  const double linear_rpm = command.linear_x_mps * linear_rpm_per_mps_;
  const double yaw_rpm = command.angular_z_radps * yaw_rpm_per_radps_;
  const bool phase_lock_allowed =
    phase_lock_enabled_ &&
    have_odometry_ &&
    (std::abs(linear_rpm) >= phase_lock_min_linear_rpm_) &&
    (phase_lock_during_pure_turn_ || (std::abs(command.linear_x_mps) >= linear_deadband_mps_));
  const auto phase_correction_rpm = compute_phase_correction_rpm(phase_lock_allowed);

  for (std::size_t index = 0U; index < motor_count; index++) {
    const double logical_forward_rpm = linear_rpm + (yaw_side_signs_[index] * yaw_rpm);
    target_motor_rpm[index] =
      (forward_motor_signs_[index] * logical_forward_rpm) +
      (phase_correction_motor_signs_[index] * phase_correction_rpm[index]);
  }

  return target_motor_rpm;
}

std::array<double, TripodGaitNode::motor_count> TripodGaitNode::compute_phase_correction_rpm(
  bool phase_lock_allowed) const
{
  std::array<double, motor_count> correction_rpm{};
  if (!phase_lock_allowed) {
    return correction_rpm;
  }

  std::array<double, motor_count> gait_phase{};
  for (std::size_t index = 0U; index < motor_count; index++) {
    gait_phase[index] = wrap_to_pi(
      (phase_direction_signs_[index] * latest_output_phase_rad_[index]) + phase_offsets_rad_[index]);
  }

  double sum_cos = 0.0;
  double sum_sin = 0.0;
  const std::array<std::size_t, 3U> tripod_a = {right_middle, left_front, left_back};
  const std::array<std::size_t, 3U> tripod_b = {right_back, right_front, left_middle};

  for (const auto index : tripod_a) {
    sum_cos += std::cos(gait_phase[index]);
    sum_sin += std::sin(gait_phase[index]);
  }
  for (const auto index : tripod_b) {
    const double equivalent_a_phase = wrap_to_pi(gait_phase[index] - pi);
    sum_cos += std::cos(equivalent_a_phase);
    sum_sin += std::sin(equivalent_a_phase);
  }

  if (std::abs(sum_cos) < std::numeric_limits<double>::epsilon() &&
    std::abs(sum_sin) < std::numeric_limits<double>::epsilon())
  {
    return correction_rpm;
  }

  const double tripod_a_reference = std::atan2(sum_sin, sum_cos);
  const double tripod_b_reference = wrap_to_pi(tripod_a_reference + pi);

  for (const auto index : tripod_a) {
    const double phase_error = wrap_to_pi(tripod_a_reference - gait_phase[index]);
    correction_rpm[index] = clamp_double(
      phase_kp_rpm_per_rad_ * phase_error,
      -max_phase_correction_rpm_,
      max_phase_correction_rpm_);
  }
  for (const auto index : tripod_b) {
    const double phase_error = wrap_to_pi(tripod_b_reference - gait_phase[index]);
    correction_rpm[index] = clamp_double(
      phase_kp_rpm_per_rad_ * phase_error,
      -max_phase_correction_rpm_,
      max_phase_correction_rpm_);
  }

  return correction_rpm;
}

std::array<int16_t, TripodGaitNode::motor_count> TripodGaitNode::limit_and_quantize_motor_rpm(
  const std::array<double, motor_count> & target_motor_rpm,
  bool immediate_stop)
{
  std::array<int16_t, motor_count> result{};
  const auto now = std::chrono::steady_clock::now();

  double dt_sec = 1.0 / publish_rate_hz_;
  if (have_limited_output_) {
    dt_sec = std::chrono::duration<double>(now - last_publish_time_).count();
    if (dt_sec <= 0.0) {
      dt_sec = 1.0 / publish_rate_hz_;
    }
  }

  for (std::size_t index = 0U; index < motor_count; index++) {
    double limited_rpm = clamp_double(target_motor_rpm[index], -max_motor_rpm_, max_motor_rpm_);

    if (have_limited_output_ && !immediate_stop && rpm_rate_limit_per_sec_ > 0.0) {
      const double max_delta = rpm_rate_limit_per_sec_ * dt_sec;
      limited_rpm = clamp_double(
        limited_rpm,
        last_limited_motor_rpm_[index] - max_delta,
        last_limited_motor_rpm_[index] + max_delta);
    }

    if (std::abs(limited_rpm) < 0.5) {
      limited_rpm = 0.0;
    }

    limited_rpm = clamp_double(limited_rpm, -32768.0, 32767.0);
    result[index] = static_cast<int16_t>(std::lrint(limited_rpm));
    last_limited_motor_rpm_[index] = static_cast<double>(result[index]);
  }

  have_limited_output_ = true;
  return result;
}

void TripodGaitNode::update_linear_velocity_estimate(const Imu & message, SteadyTime now)
{
  if (!linear_accel_feedback_enabled_) {
    last_accel_time_ = now;
    return;
  }

  if (is_effectively_zero_command()) {
    estimated_linear_velocity_mps_ = 0.0;
    have_accel_velocity_estimate_ = true;
    last_accel_time_ = now;
    return;
  }

  const double selected_acceleration_mps2 = get_selected_forward_acceleration(message);
  const double acceleration_mps2 =
    std::abs(selected_acceleration_mps2) < accel_deadband_mps2_ ? 0.0 : selected_acceleration_mps2;

  if (!have_accel_velocity_estimate_) {
    estimated_linear_velocity_mps_ = 0.0;
    have_accel_velocity_estimate_ = true;
    last_accel_time_ = now;
    return;
  }

  double dt_sec = std::chrono::duration<double>(now - last_accel_time_).count();
  last_accel_time_ = now;
  if (dt_sec <= 0.0) {
    return;
  }

  if (dt_sec > 0.50) {
    dt_sec = 0.50;
  }

  estimated_linear_velocity_mps_ += acceleration_mps2 * dt_sec;
  if (accel_velocity_leak_tau_sec_ > 0.0) {
    estimated_linear_velocity_mps_ *= std::exp(-dt_sec / accel_velocity_leak_tau_sec_);
  }
  estimated_linear_velocity_mps_ = clamp_double(
    estimated_linear_velocity_mps_,
    -max_estimated_linear_velocity_mps_,
    max_estimated_linear_velocity_mps_);
}

double TripodGaitNode::get_selected_forward_acceleration(const Imu & message) const
{
  double acceleration_mps2 = static_cast<double>(message.linear_acceleration.x);
  if (linear_accel_axis_ == 1) {
    acceleration_mps2 = static_cast<double>(message.linear_acceleration.y);
  } else if (linear_accel_axis_ == 2) {
    acceleration_mps2 = static_cast<double>(message.linear_acceleration.z);
  }
  return linear_accel_sign_ * acceleration_mps2;
}

void TripodGaitNode::update_euler_measurement(double roll_rad, double pitch_rad, double yaw_rad)
{
  latest_roll_rad_ = euler_axis_signs_[0] * roll_rad;
  latest_pitch_rad_ = euler_axis_signs_[1] * pitch_rad;
  latest_yaw_rad_ = euler_axis_signs_[2] * yaw_rad;
  have_euler_ = true;
  last_euler_time_ = std::chrono::steady_clock::now();
}

void TripodGaitNode::update_euler_measurement_from_vector(const Vector3 & vector_message)
{
  const double unit_scale = euler_angles_are_degrees_ ? (pi / 180.0) : 1.0;
  update_euler_measurement(
    unit_scale * static_cast<double>(vector_message.x),
    unit_scale * static_cast<double>(vector_message.y),
    unit_scale * static_cast<double>(vector_message.z));
}

bool TripodGaitNode::get_latest_roll_pitch(SteadyTime now, double & roll_rad, double & pitch_rad) const
{
  if (!is_euler_recent(now)) {
    return false;
  }

  roll_rad = latest_roll_rad_;
  pitch_rad = latest_pitch_rad_;
  return true;
}

bool TripodGaitNode::is_command_recent(SteadyTime now) const
{
  if (!have_command_) {
    return false;
  }
  if (latch_last_teleop_command_ || command_timeout_sec_ <= 0.0) {
    return true;
  }
  return std::chrono::duration<double>(now - last_command_time_).count() <= command_timeout_sec_;
}

bool TripodGaitNode::is_odometry_recent(SteadyTime now) const
{
  if (!have_odometry_) {
    return false;
  }
  if (odometry_timeout_sec_ <= 0.0) {
    return true;
  }
  return std::chrono::duration<double>(now - last_odometry_time_).count() <= odometry_timeout_sec_;
}

bool TripodGaitNode::is_imu_recent(SteadyTime now) const
{
  if (!have_imu_) {
    return false;
  }
  if (imu_timeout_sec_ <= 0.0) {
    return true;
  }
  return std::chrono::duration<double>(now - last_imu_time_).count() <= imu_timeout_sec_;
}

bool TripodGaitNode::is_euler_recent(SteadyTime now) const
{
  if (!have_euler_) {
    return false;
  }
  if (euler_timeout_sec_ <= 0.0) {
    return true;
  }
  return std::chrono::duration<double>(now - last_euler_time_).count() <= euler_timeout_sec_;
}

bool TripodGaitNode::is_effectively_zero_command() const
{
  return std::abs(latest_linear_x_mps_) < linear_deadband_mps_ &&
         std::abs(latest_angular_z_radps_) < angular_deadband_radps_;
}

double TripodGaitNode::compute_tilt_scale(double roll_rad, double pitch_rad) const
{
  double scale = 1.0;

  if (roll_hard_limit_rad_ > roll_soft_limit_rad_ && std::abs(roll_rad) > roll_soft_limit_rad_) {
    const double roll_scale = 1.0 -
      ((std::abs(roll_rad) - roll_soft_limit_rad_) / (roll_hard_limit_rad_ - roll_soft_limit_rad_));
    scale = std::min(scale, clamp_double(roll_scale, 0.0, 1.0));
  } else if (std::abs(roll_rad) >= roll_hard_limit_rad_) {
    scale = 0.0;
  }

  if (pitch_hard_limit_rad_ > pitch_soft_limit_rad_ && std::abs(pitch_rad) > pitch_soft_limit_rad_) {
    const double pitch_scale = 1.0 -
      ((std::abs(pitch_rad) - pitch_soft_limit_rad_) / (pitch_hard_limit_rad_ - pitch_soft_limit_rad_));
    scale = std::min(scale, clamp_double(pitch_scale, 0.0, 1.0));
  } else if (std::abs(pitch_rad) >= pitch_hard_limit_rad_) {
    scale = 0.0;
  }

  return scale;
}

double TripodGaitNode::wrap_to_pi(double angle_rad) const
{
  double wrapped = std::fmod(angle_rad + pi, 2.0 * pi);
  if (wrapped < 0.0) {
    wrapped += 2.0 * pi;
  }
  return wrapped - pi;
}

double TripodGaitNode::clamp_double(double value, double minimum, double maximum) const
{
  return std::max(minimum, std::min(value, maximum));
}

void TripodGaitNode::quaternion_to_roll_pitch_yaw(
  double x,
  double y,
  double z,
  double w,
  double & roll_rad,
  double & pitch_rad,
  double & yaw_rad) const
{
  const double sinr_cosp = 2.0 * ((w * x) + (y * z));
  const double cosr_cosp = 1.0 - (2.0 * ((x * x) + (y * y)));
  roll_rad = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * ((w * y) - (z * x));
  if (std::abs(sinp) >= 1.0) {
    pitch_rad = std::copysign(pi / 2.0, sinp);
  } else {
    pitch_rad = std::asin(sinp);
  }

  const double siny_cosp = 2.0 * ((w * z) + (x * y));
  const double cosy_cosp = 1.0 - (2.0 * ((y * y) + (z * z)));
  yaw_rad = std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace hexapod_tripod_gait
