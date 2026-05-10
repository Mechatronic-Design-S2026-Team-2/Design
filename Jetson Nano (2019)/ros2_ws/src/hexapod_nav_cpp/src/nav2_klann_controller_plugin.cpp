#include "hexapod_nav_cpp/nav2_klann_controller_plugin.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace hexapod_nav_cpp
{

void Nav2KlannController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  name_ = std::move(name);
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  core_ = std::make_shared<KlannControllerCore>();

  auto node = parent_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock lifecycle node");
  }

  const auto linkage_yaml = node->declare_parameter<std::string>(name_ + ".linkage_yaml", "");
  if (!linkage_yaml.empty()) {
    core_->initialize_lookup_models(linkage_yaml);
  }

  const double horizon_dt_s = node->declare_parameter<double>(name_ + ".horizon_dt_s", 0.05);
  const int horizon_steps = node->declare_parameter<int>(name_ + ".horizon_steps", 14);
  const int sample_count = node->declare_parameter<int>(name_ + ".sample_count", 48);
  const double nominal_phase_rate = node->declare_parameter<double>(name_ + ".nominal_phase_rate_rad_s", 0.25);
  linear_speed_limit_mps_ = node->declare_parameter<double>(name_ + ".linear_speed_limit_mps", 0.03);
  const double angular_speed_limit = node->declare_parameter<double>(name_ + ".angular_speed_limit_rps", 0.08);

  core_->set_parameters(
    horizon_dt_s,
    horizon_steps,
    sample_count,
    nominal_phase_rate,
    linear_speed_limit_mps_,
    angular_speed_limit,
    node->declare_parameter<double>(name_ + ".temperature", 0.35),
    node->declare_parameter<double>(name_ + ".vx_noise_std", 0.005),
    node->declare_parameter<double>(name_ + ".wz_noise_std", 0.02),
    node->declare_parameter<double>(name_ + ".phase_rate_noise_std", 0.02),
    node->declare_parameter<double>(name_ + ".path_weight", 8.0),
    node->declare_parameter<double>(name_ + ".heading_weight", 8.0),
    node->declare_parameter<double>(name_ + ".progress_weight", 0.25),
    node->declare_parameter<double>(name_ + ".support_weight", 50.0),
    node->declare_parameter<double>(name_ + ".support_margin_weight", 12.0),
    node->declare_parameter<double>(name_ + ".slip_weight", 6.0),
    node->declare_parameter<double>(name_ + ".phase_sync_weight", 20.0),
    node->declare_parameter<double>(name_ + ".control_smooth_weight", 15.0),
    node->declare_parameter<double>(name_ + ".lateral_velocity_weight", 8.0),
    node->declare_parameter<double>(name_ + ".yaw_rate_tracking_weight", 1.5),
    node->declare_parameter<double>(name_ + ".roll_weight", 3.0),
    node->declare_parameter<double>(name_ + ".pitch_weight", 3.0),
    node->declare_parameter<double>(name_ + ".roll_tracking_weight", 0.75),
    node->declare_parameter<double>(name_ + ".pitch_tracking_weight", 0.75),
    node->declare_parameter<double>(name_ + ".phase_rate_limit_rad_s", 0.45),
    node->declare_parameter<double>(name_ + ".phase_sync_gain", 0.8),
    node->declare_parameter<double>(name_ + ".turn_phase_bias_gain", 0.35),
    node->declare_parameter<double>(name_ + ".minimum_speed_scale", 0.2),
    node->declare_parameter<double>(name_ + ".wave_turn_threshold_rps", 0.35),
    node->declare_parameter<double>(name_ + ".in_place_turn_threshold_rps", 0.10),
    node->declare_parameter<double>(name_ + ".forward_phase_rate_sign", -1.0),
    node->declare_parameter<bool>(name_ + ".mirror_kinematics_across_y_axis", true),
    node->declare_parameter<double>(name_ + ".goal_tolerance_m", 0.06),
    node->declare_parameter<bool>(name_ + ".lock_forward_phase_rate_sign", true),
    node->declare_parameter<double>(name_ + ".forward_lock_vx_threshold_mps", 0.005),
    node->declare_parameter<double>(name_ + ".forward_lock_wz_threshold_rps", 0.12),
    node->declare_parameter<double>(name_ + ".min_locked_phase_rate_rad_s", 0.03),
    node->declare_parameter<int>(name_ + ".random_seed", 42));

  auto motor_ids = node->declare_parameter<std::vector<int64_t>>(name_ + ".motor_ids", std::vector<int64_t>{3, 2, 1, 4, 5, 6});
  for (std::size_t i = 0; i < 6 && i < motor_ids.size(); ++i) {
    motor_ids_[i] = static_cast<uint8_t>(motor_ids[i]);
  }

  body_sub_ = node->create_subscription<hexapod_control_interfaces::msg::KlannBodyState>(
    "hexapod/body_state", 10, std::bind(&Nav2KlannController::on_body_state, this, std::placeholders::_1));
  phase_pub_ = node->create_publisher<hexapod_control_interfaces::msg::LegPhaseCommand>("hexapod/phase_cmd", 10);
}

void Nav2KlannController::cleanup()
{
  body_sub_.reset();
  phase_pub_.reset();
  core_.reset();
}

void Nav2KlannController::activate()
{
}

void Nav2KlannController::deactivate()
{
}

void Nav2KlannController::setPlan(const nav_msgs::msg::Path & path)
{
  plan_ = path;
}

geometry_msgs::msg::TwistStamped Nav2KlannController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped &,
  const geometry_msgs::msg::Twist &,
  nav2_core::GoalChecker *)
{
  if (!latest_body_state_) {
    geometry_msgs::msg::TwistStamped zero;
    zero.header.stamp = rclcpp::Clock().now();
    zero.header.frame_id = "base_link";
    return zero;
  }

  const auto result = core_->compute(plan_, *latest_body_state_, motor_ids_);
  phase_pub_->publish(result.phase_cmd);
  return result.cmd_vel;
}

void Nav2KlannController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    linear_speed_limit_mps_ *= speed_limit;
  } else {
    linear_speed_limit_mps_ = speed_limit;
  }
}

void Nav2KlannController::on_body_state(const hexapod_control_interfaces::msg::KlannBodyState::SharedPtr msg)
{
  latest_body_state_ = msg;
}

}  // namespace hexapod_nav_cpp

PLUGINLIB_EXPORT_CLASS(hexapod_nav_cpp::Nav2KlannController, nav2_core::Controller)
