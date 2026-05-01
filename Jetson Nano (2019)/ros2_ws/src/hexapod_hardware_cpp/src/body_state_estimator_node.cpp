#include "hexapod_hardware_cpp/klann_geometry.hpp"
#include "hexapod_hardware_cpp/path_utils.hpp"

#include <Eigen/Dense>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <hexapod_control_interfaces/msg/hexapod_motor_state.hpp>
#include <hexapod_control_interfaces/msg/klann_body_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace hexapod_hardware_cpp
{

namespace
{
double wrap_to_pi(double angle_rad)
{
  while (angle_rad > M_PI) angle_rad -= 2.0 * M_PI;
  while (angle_rad < -M_PI) angle_rad += 2.0 * M_PI;
  return angle_rad;
}

void quaternion_to_rpy(const geometry_msgs::msg::Quaternion & q, double & roll, double & pitch, double & yaw)
{
  const double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
  const double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
  pitch = std::abs(sinp) >= 1.0 ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);

  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  yaw = std::atan2(siny_cosp, cosy_cosp);
}

std::array<double, 3> parse_axis_scale(const std::vector<double> & values, const std::array<double, 3> & defaults)
{
  std::array<double, 3> result = defaults;
  for (std::size_t i = 0; i < result.size() && i < values.size(); ++i) {
    if (std::isfinite(values[i])) {
      result[i] = values[i];
    }
  }
  return result;
}
}  // namespace

class BodyStateEstimatorNode : public rclcpp::Node
{
public:
  BodyStateEstimatorNode()
  : Node("body_state_estimator_node"),
    tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(*this))
  {
    linkage_yaml_ = declare_parameter<std::string>("linkage_yaml", "");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/hexapod/kinematic_odom");
    odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "hexapod_kinematic_odom");
    publish_tf_ = declare_parameter<bool>("publish_tf", false);
    dt_hint_s_ = declare_parameter<double>("dt_hint_s", 0.01);
    twist_lowpass_alpha_ = declare_parameter<double>("twist_lowpass_alpha", 0.35);
    min_stance_confidence_ = declare_parameter<double>("min_stance_confidence", 0.5);
    min_stance_legs_ = declare_parameter<int>("min_stance_legs", 3);
    zero_twist_without_stance_ = declare_parameter<bool>("zero_twist_without_stance", true);
    use_imu_roll_pitch_ = declare_parameter<bool>("use_imu_roll_pitch", true);
    use_imu_yaw_absolute_ = declare_parameter<bool>("use_imu_yaw_absolute", true);
    use_imu_yaw_rate_ = declare_parameter<bool>("use_imu_yaw_rate", true);
    body_heading_offset_rad_ = declare_parameter<double>("body_heading_offset_rad", M_PI);
    imu_yaw_alpha_ = declare_parameter<double>("imu_yaw_alpha", 0.12);
    imu_yaw_rate_alpha_ = declare_parameter<double>("imu_yaw_rate_alpha", 0.35);
    body_velocity_scale_ = declare_parameter<double>("body_velocity_scale", 1.0);
    body_twist_axis_scale_ = parse_axis_scale(
      declare_parameter<std::vector<double>>("body_twist_axis_scale", std::vector<double>{1.0, 1.0, 1.0}),
      std::array<double, 3>{1.0, 1.0, 1.0});
    mirror_kinematics_across_y_axis_ = declare_parameter<bool>("mirror_kinematics_across_y_axis", false);
    phase_velocity_scale_ = declare_parameter<double>("phase_velocity_scale", 1.0);
    use_measurement_delay_compensation_ = declare_parameter<bool>("use_measurement_delay_compensation", true);
    measurement_age_gain_ = declare_parameter<double>("measurement_age_gain", 1.0);
    publish_foot_debug_ = declare_parameter<bool>("publish_foot_debug", true);

    yaw_ = wrap_to_pi(body_heading_offset_rad_);
    linkage_yaml_ = linkage_yaml_.empty() ? auto_linkage_yaml("hexapod_hardware_cpp") : expand_user_path(linkage_yaml_);
    RCLCPP_INFO(get_logger(), "Using linkage YAML: %s", linkage_yaml_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Kinematic odom calibration: mirror_kinematics_across_y_axis=%s, body_velocity_scale=%.6g, body_twist_axis_scale=[%.6g, %.6g, %.6g]",
      mirror_kinematics_across_y_axis_ ? "true" : "false", body_velocity_scale_,
      body_twist_axis_scale_[0], body_twist_axis_scale_[1], body_twist_axis_scale_[2]);
    models_ = LinkageSolver::load_models_from_yaml(linkage_yaml_);

    motor_sub_ = create_subscription<hexapod_control_interfaces::msg::HexapodMotorState>(
      "hexapod/motor_state", 10, std::bind(&BodyStateEstimatorNode::on_motor_state, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu/data_raw", 20, std::bind(&BodyStateEstimatorNode::on_imu, this, std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    body_pub_ = create_publisher<hexapod_control_interfaces::msg::KlannBodyState>("hexapod/body_state", 10);

    timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&BodyStateEstimatorNode::update, this));
  }

private:
  struct TwistSolveResult
  {
    Eigen::Vector3d twist_body{Eigen::Vector3d::Zero()};
    int stance_leg_count{0};
    bool valid{false};
  };

  void on_motor_state(const hexapod_control_interfaces::msg::HexapodMotorState::SharedPtr msg)
  {
    latest_motor_state_ = msg;
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    latest_imu_ = msg;
  }

  TwistSolveResult solve_body_twist_from_stance(
    const std::array<Eigen::Vector2d, 6> & foot,
    const std::array<Eigen::Vector2d, 6> & foot_vel,
    const std::array<double, 6> & stance_confidence)
  {
    TwistSolveResult result;
    std::vector<int> active;
    for (int i = 0; i < 6; ++i) {
      if (stance_confidence[static_cast<std::size_t>(i)] >= min_stance_confidence_) {
        active.push_back(i);
      }
    }

    result.stance_leg_count = static_cast<int>(active.size());
    if (static_cast<int>(active.size()) < min_stance_legs_) {
      return result;
    }

    Eigen::MatrixXd A(2 * active.size(), 3);
    Eigen::VectorXd b(2 * active.size());
    Eigen::VectorXd w(2 * active.size());
    int row = 0;

    for (int idx : active) {
      const auto & p = foot[static_cast<std::size_t>(idx)];
      const auto & v = foot_vel[static_cast<std::size_t>(idx)];
      const double conf = std::max(0.05, stance_confidence[static_cast<std::size_t>(idx)]);

      A(row, 0) = 1.0;
      A(row, 1) = 0.0;
      A(row, 2) = -p.y();
      b(row) = -v.x();
      w(row) = std::sqrt(conf);
      ++row;

      A(row, 0) = 0.0;
      A(row, 1) = 1.0;
      A(row, 2) = p.x();
      b(row) = -v.y();
      w(row) = std::sqrt(conf);
      ++row;
    }

    Eigen::MatrixXd W = w.asDiagonal();
    const Eigen::Vector3d xi = (W * A).colPivHouseholderQr().solve(W * b);
    result.twist_body = xi;
    result.valid = true;
    return result;
  }

  void update()
  {
    if (!latest_motor_state_) {
      return;
    }

    const rclcpp::Time stamp = now();
    double dt = dt_hint_s_;
    if (last_update_time_.nanoseconds() > 0) {
      dt = std::max(1.0e-3, (stamp - last_update_time_).seconds());
    }
    last_update_time_ = stamp;

    std::array<Eigen::Vector2d, 6> foot {};
    std::array<Eigen::Vector2d, 6> foot_vel {};
    std::array<double, 6> foot_z {};
    std::array<double, 6> foot_vel_z {};
    std::array<double, 6> stance_conf {};
    std::array<double, 6> phase {};
    std::array<double, 6> phase_velocity {};
    std::array<LegKinematicState, 6> kinematics {};

    for (std::size_t i = 0; i < 6 && i < models_.size(); ++i) {
      phase_velocity[i] = phase_velocity_scale_ * latest_motor_state_->phase_velocity_rad_s[i];
      phase[i] = latest_motor_state_->phase_rad[i];
      if (use_measurement_delay_compensation_) {
        phase[i] += measurement_age_gain_ * static_cast<double>(latest_motor_state_->estimated_measurement_age_s) * phase_velocity[i];
      }
      kinematics[i] = solver_.evaluate(models_[i], phase[i], phase_velocity[i], std::max(dt_hint_s_, dt));
      if (!kinematics[i].valid) {
        kinematics[i] = solver_.evaluate(models_[i], phase[i], 0.0, std::max(dt_hint_s_, dt));
      }
      foot[i] = kinematics[i].foot_body_xy;
      foot_vel[i] = kinematics[i].foot_velocity_body_xy;
      if (mirror_kinematics_across_y_axis_) {
        foot[i].x() = -foot[i].x();
        foot_vel[i].x() = -foot_vel[i].x();
      }
      foot_z[i] = kinematics[i].foot_body_z;
      foot_vel_z[i] = kinematics[i].foot_velocity_body_z;
      stance_conf[i] = latest_motor_state_->fresh[i] ? kinematics[i].stance_confidence : 0.0;
    }

    const TwistSolveResult solve = solve_body_twist_from_stance(foot, foot_vel, stance_conf);

    Eigen::Vector3d measured_twist_body = solve.twist_body * body_velocity_scale_;
    measured_twist_body.x() *= body_twist_axis_scale_[0];
    measured_twist_body.y() *= body_twist_axis_scale_[1];
    measured_twist_body.z() *= body_twist_axis_scale_[2];
    if (!solve.valid && zero_twist_without_stance_) {
      measured_twist_body.setZero();
    }

    double imu_roll = 0.0;
    double imu_pitch = 0.0;
    double imu_yaw = yaw_;
    double imu_yaw_rate = 0.0;
    if (latest_imu_) {
      quaternion_to_rpy(latest_imu_->orientation, imu_roll, imu_pitch, imu_yaw);
      imu_yaw = wrap_to_pi(imu_yaw + body_heading_offset_rad_);
      imu_yaw_rate = latest_imu_->angular_velocity.z;
    }

    if (use_imu_yaw_rate_ && latest_imu_) {
      measured_twist_body.z() =
        (1.0 - imu_yaw_rate_alpha_) * measured_twist_body.z() +
        imu_yaw_rate_alpha_ * imu_yaw_rate;
    }

    twist_body_.x() += twist_lowpass_alpha_ * (measured_twist_body.x() - twist_body_.x());
    twist_body_.y() += twist_lowpass_alpha_ * (measured_twist_body.y() - twist_body_.y());
    twist_body_.z() += twist_lowpass_alpha_ * (measured_twist_body.z() - twist_body_.z());

    const double cos_yaw = std::cos(yaw_);
    const double sin_yaw = std::sin(yaw_);
    const double vx_world = cos_yaw * twist_body_.x() - sin_yaw * twist_body_.y();
    const double vy_world = sin_yaw * twist_body_.x() + cos_yaw * twist_body_.y();

    x_ += vx_world * dt;
    y_ += vy_world * dt;
    yaw_ = wrap_to_pi(yaw_ + twist_body_.z() * dt);
    if (use_imu_yaw_absolute_ && latest_imu_) {
      yaw_ = wrap_to_pi(yaw_ + imu_yaw_alpha_ * wrap_to_pi(imu_yaw - yaw_));
    }

    const double roll = use_imu_roll_pitch_ && latest_imu_ ? imu_roll : 0.0;
    const double pitch = use_imu_roll_pitch_ && latest_imu_ ? imu_pitch : 0.0;

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = base_frame_id_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = twist_body_.x();
    odom.twist.twist.linear.y = twist_body_.y();
    odom.twist.twist.angular.z = twist_body_.z();
    odom_pub_->publish(odom);

    hexapod_control_interfaces::msg::KlannBodyState body;
    body.header = odom.header;
    body.pose = odom.pose.pose;
    body.twist = odom.twist.twist;
    body.roll_rad = static_cast<float>(roll);
    body.pitch_rad = static_cast<float>(pitch);
    body.yaw_rad = static_cast<float>(yaw_);
    body.imu_yaw_rad = static_cast<float>(imu_yaw);
    body.imu_yaw_rate_rad_s = static_cast<float>(imu_yaw_rate);
    body.stance_leg_count = solve.stance_leg_count;
    body.battery_voltage_v = latest_motor_state_->battery_voltage_v;
    body.feedback_dt_s = latest_motor_state_->feedback_dt_s;
    body.estimated_measurement_age_s = latest_motor_state_->estimated_measurement_age_s;
    body.estimated_future_command_apply_delay_s = latest_motor_state_->estimated_future_command_apply_delay_s;
    body.estimated_total_preview_s = latest_motor_state_->estimated_measurement_age_s + latest_motor_state_->estimated_future_command_apply_delay_s;
    for (std::size_t i = 0; i < 6; ++i) {
      body.phase_rad[i] = static_cast<float>(phase[i]);
      body.phase_velocity_rad_s[i] = static_cast<float>(phase_velocity[i]);
      body.stance_confidence[i] = static_cast<float>(stance_conf[i]);
      if (publish_foot_debug_) {
        body.foot_position_body[i].x = foot[i].x();
        body.foot_position_body[i].y = foot[i].y();
        body.foot_position_body[i].z = foot_z[i];
        body.foot_velocity_body[i].x = foot_vel[i].x();
        body.foot_velocity_body[i].y = foot_vel[i].y();
        body.foot_velocity_body[i].z = foot_vel_z[i];
      }
    }
    body_pub_->publish(body);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = odom.header;
      tf.child_frame_id = base_frame_id_;
      tf.transform.translation.x = x_;
      tf.transform.translation.y = y_;
      tf.transform.translation.z = 0.0;
      tf.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }
  }

  std::string linkage_yaml_;
  std::string base_frame_id_;
  std::string odom_topic_;
  std::string odom_frame_id_;
  bool publish_tf_{true};
  double dt_hint_s_{0.01};
  double twist_lowpass_alpha_{0.35};
  double min_stance_confidence_{0.5};
  int min_stance_legs_{3};
  bool zero_twist_without_stance_{true};
  bool use_imu_roll_pitch_{true};
  bool use_imu_yaw_absolute_{true};
  bool use_imu_yaw_rate_{true};
  double body_heading_offset_rad_{M_PI};
  double imu_yaw_alpha_{0.12};
  double imu_yaw_rate_alpha_{0.35};
  double body_velocity_scale_{1.0};
  std::array<double, 3> body_twist_axis_scale_{1.0, 1.0, 1.0};
  bool mirror_kinematics_across_y_axis_{false};
  double phase_velocity_scale_{1.0};
  bool use_measurement_delay_compensation_{true};
  double measurement_age_gain_{1.0};
  bool publish_foot_debug_{true};

  LinkageSolver solver_;
  std::vector<LegModel> models_;
  hexapod_control_interfaces::msg::HexapodMotorState::SharedPtr latest_motor_state_;
  sensor_msgs::msg::Imu::SharedPtr latest_imu_;

  rclcpp::Subscription<hexapod_control_interfaces::msg::HexapodMotorState>::SharedPtr motor_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<hexapod_control_interfaces::msg::KlannBodyState>::SharedPtr body_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  Eigen::Vector3d twist_body_{Eigen::Vector3d::Zero()};
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
};

}  // namespace hexapod_hardware_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hexapod_hardware_cpp::BodyStateEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
