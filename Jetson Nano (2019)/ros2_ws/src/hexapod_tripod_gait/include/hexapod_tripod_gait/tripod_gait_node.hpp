#ifndef HEXAPOD_TRIPOD_GAIT__TRIPOD_GAIT_NODE_HPP_
#define HEXAPOD_TRIPOD_GAIT__TRIPOD_GAIT_NODE_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "dsy_motor_msgs/msg/motor_output_odometry_array.hpp"
#include "dsy_motor_msgs/msg/motor_rpm_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "hexapod_control_interfaces/msg/hexapod_motor_state.hpp"
#include "hexapod_control_interfaces/msg/leg_phase_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace hexapod_tripod_gait
{

class TripodGaitNode : public rclcpp::Node
{
public:
  explicit TripodGaitNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using MotorOutputOdometryArray = dsy_motor_msgs::msg::MotorOutputOdometryArray;
  using MotorRpmArray = dsy_motor_msgs::msg::MotorRpmArray;
  using HexapodMotorState = hexapod_control_interfaces::msg::HexapodMotorState;
  using LegPhaseCommand = hexapod_control_interfaces::msg::LegPhaseCommand;
  using Imu = sensor_msgs::msg::Imu;
  using Twist = geometry_msgs::msg::Twist;
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  using Vector3 = geometry_msgs::msg::Vector3;
  using Vector3Stamped = geometry_msgs::msg::Vector3Stamped;
  using SteadyTime = std::chrono::steady_clock::time_point;

  static constexpr std::size_t motor_count = 6U;
  static constexpr double pi = 3.141592653589793238462643383279502884;

  enum LegIndex : std::size_t
  {
    right_back = 0U,
    right_middle = 1U,
    right_front = 2U,
    left_front = 3U,
    left_middle = 4U,
    left_back = 5U,
  };

  struct PlanarCommand
  {
    double linear_x_mps = 0.0;
    double angular_z_radps = 0.0;
  };

  void declare_and_load_parameters();
  std::array<double, motor_count> get_double_array_parameter(
    const std::string & name,
    const std::array<double, motor_count> & fallback);
  std::array<double, motor_count> get_sign_array_parameter(
    const std::string & name,
    const std::array<double, motor_count> & fallback);
  std::array<std::size_t, motor_count> get_index_array_parameter(
    const std::string & name,
    const std::array<std::size_t, motor_count> & fallback);

  void twist_callback(const Twist::SharedPtr message);
  void twist_stamped_callback(const TwistStamped::SharedPtr message);
  void odometry_callback(const MotorOutputOdometryArray::SharedPtr message);
  void motor_state_callback(const HexapodMotorState::SharedPtr message);
  void imu_callback(const Imu::SharedPtr message);
  void euler_callback(const Vector3Stamped::SharedPtr message);
  void euler_vector_callback(const Vector3::SharedPtr message);
  void publish_timer_callback();

  PlanarCommand get_feedback_adjusted_command(SteadyTime now) const;
  std::array<double, motor_count> compute_phase_correction_rpm(bool phase_lock_allowed) const;
  std::array<double, motor_count> compute_unlimited_motor_rpm(
    bool command_active,
    const PlanarCommand & command) const;
  std::array<int16_t, motor_count> limit_and_quantize_motor_rpm(
    const std::array<double, motor_count> & target_motor_rpm,
    bool immediate_stop);
  void publish_rpm_command(const std::array<int16_t, motor_count> & motor_rpm);
  void publish_phase_command(const std::array<int16_t, motor_count> & motor_rpm);
  double motor_rpm_to_output_phase_velocity(double motor_rpm) const;
  bool command_output_mode_is_phase() const;

  void update_euler_measurement(double roll_rad, double pitch_rad, double yaw_rad);
  void update_euler_measurement_from_vector(const Vector3 & vector_message);
  void update_linear_velocity_estimate(const Imu & message, SteadyTime now);
  double get_selected_forward_acceleration(const Imu & message) const;
  bool get_latest_roll_pitch(SteadyTime now, double & roll_rad, double & pitch_rad) const;
  bool is_command_recent(SteadyTime now) const;
  bool is_odometry_recent(SteadyTime now) const;
  bool is_imu_recent(SteadyTime now) const;
  bool is_euler_recent(SteadyTime now) const;
  bool is_effectively_zero_command() const;
  double compute_tilt_scale(double roll_rad, double pitch_rad) const;
  double wrap_to_pi(double angle_rad) const;
  double clamp_double(double value, double minimum, double maximum) const;
  void quaternion_to_roll_pitch_yaw(
    double x,
    double y,
    double z,
    double w,
    double & roll_rad,
    double & pitch_rad,
    double & yaw_rad) const;

  rclcpp::Subscription<Twist>::SharedPtr twist_subscription_;
  rclcpp::Subscription<TwistStamped>::SharedPtr twist_stamped_subscription_;
  rclcpp::Subscription<MotorOutputOdometryArray>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<HexapodMotorState>::SharedPtr motor_state_subscription_;
  rclcpp::Subscription<Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<Vector3Stamped>::SharedPtr euler_subscription_;
  rclcpp::Subscription<Vector3>::SharedPtr euler_vector_subscription_;
  rclcpp::Publisher<MotorRpmArray>::SharedPtr rpm_publisher_;
  rclcpp::Publisher<LegPhaseCommand>::SharedPtr phase_command_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::string cmd_vel_topic_;
  std::string cmd_vel_stamped_topic_;
  std::string odometry_topic_;
  std::string motor_state_topic_;
  std::string rpm_command_topic_;
  std::string phase_command_topic_;
  std::string command_output_mode_;
  std::string imu_topic_;
  std::string euler_topic_;
  std::string euler_vector_topic_;

  double publish_rate_hz_ = 10.0;
  double linear_rpm_per_mps_ = 1200.0;
  double yaw_rpm_per_radps_ = 600.0;
  double max_motor_rpm_ = 1500.0;
  double rpm_rate_limit_per_sec_ = 0.0;
  double gear_ratio_ = 50.0;
  double linear_deadband_mps_ = 0.01;
  double angular_deadband_radps_ = 0.01;
  double command_timeout_sec_ = 0.30;
  bool latch_last_teleop_command_ = true;
  double odometry_timeout_sec_ = 0.50;
  double imu_timeout_sec_ = 0.35;
  double euler_timeout_sec_ = 0.35;
  double phase_kp_rpm_per_rad_ = 25.0;
  double max_phase_correction_rpm_ = 100.0;
  double phase_lock_min_linear_rpm_ = 10.0;
  double yaw_rate_feedback_kp_ = 0.0;
  double max_yaw_rate_correction_radps_ = 0.35;
  double linear_velocity_feedback_kp_ = 0.0;
  double max_linear_velocity_correction_mps_ = 0.20;
  double accel_deadband_mps2_ = 0.10;
  double accel_velocity_leak_tau_sec_ = 1.50;
  double max_estimated_linear_velocity_mps_ = 1.00;
  double roll_soft_limit_rad_ = 0.35;
  double roll_hard_limit_rad_ = 0.70;
  double pitch_soft_limit_rad_ = 0.35;
  double pitch_hard_limit_rad_ = 0.70;
  bool phase_lock_enabled_ = true;
  bool phase_lock_during_pure_turn_ = false;
  bool require_odometry_for_motion_ = false;
  bool use_motor_state_feedback_ = true;
  bool use_compact_odometry_feedback_ = false;
  bool motor_state_require_comm_ok_ = false;
  bool motor_state_require_fresh_ = false;
  bool publish_unchanged_commands_ = false;
  bool imu_feedback_enabled_ = true;
  bool euler_feedback_enabled_ = true;
  bool yaw_rate_feedback_enabled_ = true;
  bool linear_accel_feedback_enabled_ = false;
  bool tilt_safety_enabled_ = true;
  bool use_raw_imu_orientation_ = true;
  bool euler_angles_are_degrees_ = false;
  bool phase_command_use_phase_velocity_ = true;
  bool phase_command_use_motor_rpm_ff_ = false;
  int linear_accel_axis_ = 0;
  double linear_accel_sign_ = 1.0;

  std::array<double, motor_count> forward_motor_signs_ = {1.0, 1.0, 1.0, -1.0, -1.0, -1.0};
  std::array<double, motor_count> phase_direction_signs_ = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  std::array<double, motor_count> yaw_side_signs_ = {1.0, 1.0, 1.0, -1.0, -1.0, -1.0};
  std::array<double, motor_count> phase_correction_motor_signs_ = {-1.0, -1.0, -1.0, 1.0, 1.0, 1.0};
  std::array<double, 3U> euler_axis_signs_ = {1.0, 1.0, 1.0};
  std::array<double, motor_count> phase_offsets_rad_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, motor_count> model_to_physical_motor_signs_ = {-1.0, -1.0, -1.0, 1.0, 1.0, 1.0};
  std::array<std::size_t, motor_count> internal_to_motor_state_index_ = {2U, 1U, 0U, 3U, 4U, 5U};
  std::array<std::size_t, motor_count> internal_to_phase_command_index_ = {2U, 1U, 0U, 3U, 4U, 5U};

  std::array<double, motor_count> latest_output_phase_rad_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, motor_count> last_limited_motor_rpm_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<int16_t, motor_count> last_published_motor_rpm_ = {0, 0, 0, 0, 0, 0};

  double latest_linear_x_mps_ = 0.0;
  double latest_angular_z_radps_ = 0.0;
  double latest_yaw_rate_radps_ = 0.0;
  double latest_linear_accel_x_mps2_ = 0.0;
  double latest_linear_accel_y_mps2_ = 0.0;
  double latest_linear_accel_z_mps2_ = 0.0;
  double estimated_linear_velocity_mps_ = 0.0;
  double latest_roll_rad_ = 0.0;
  double latest_pitch_rad_ = 0.0;
  double latest_yaw_rad_ = 0.0;
  bool have_command_ = false;
  bool have_odometry_ = false;
  bool have_imu_ = false;
  bool have_euler_ = false;
  bool have_accel_velocity_estimate_ = false;
  bool have_limited_output_ = false;
  bool have_published_output_ = false;
  uint32_t command_seq_ = 0U;

  SteadyTime last_command_time_{};
  SteadyTime last_odometry_time_{};
  SteadyTime last_imu_time_{};
  SteadyTime last_euler_time_{};
  SteadyTime last_accel_time_{};
  SteadyTime last_publish_time_{};
};

}  // namespace hexapod_tripod_gait

#endif  // HEXAPOD_TRIPOD_GAIT__TRIPOD_GAIT_NODE_HPP_
