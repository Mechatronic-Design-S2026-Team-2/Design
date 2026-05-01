#include "hexapod_nav_cpp/klann_controller_core.hpp"
#include "hexapod_hardware_cpp/path_utils.hpp"

#include <hexapod_control_interfaces/msg/klann_body_state.hpp>
#include <hexapod_control_interfaces/msg/leg_phase_command.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <array>

namespace hexapod_nav_cpp
{

class KlannMppiControllerNode : public rclcpp::Node
{
public:
  KlannMppiControllerNode()
  : Node("klann_mppi_controller_node")
  {
    motor_ids_vec_ = declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>{3, 2, 1, 4, 5, 6});
    auto linkage_yaml = declare_parameter<std::string>("linkage_yaml", "");
    linkage_yaml = linkage_yaml.empty() ? hexapod_hardware_cpp::auto_linkage_yaml("hexapod_hardware_cpp") : hexapod_hardware_cpp::expand_user_path(linkage_yaml);
    RCLCPP_INFO(get_logger(), "Using linkage YAML: %s", linkage_yaml.c_str());
    core_.initialize_lookup_models(linkage_yaml);
    core_.set_parameters(
      declare_parameter<double>("horizon_dt_s", 0.05),
      declare_parameter<int>("horizon_steps", 18),
      declare_parameter<int>("sample_count", 192),
      declare_parameter<double>("nominal_phase_rate_rad_s", 2.0),
      declare_parameter<double>("linear_speed_limit_mps", 0.25),
      declare_parameter<double>("angular_speed_limit_rps", 0.75),
      declare_parameter<double>("temperature", 0.6),
      declare_parameter<double>("vx_noise_std", 0.04),
      declare_parameter<double>("wz_noise_std", 0.16),
      declare_parameter<double>("phase_rate_noise_std", 0.8),
      declare_parameter<double>("path_weight", 8.0),
      declare_parameter<double>("heading_weight", 3.0),
      declare_parameter<double>("progress_weight", 1.0),
      declare_parameter<double>("support_weight", 30.0),
      declare_parameter<double>("support_margin_weight", 8.0),
      declare_parameter<double>("slip_weight", 4.0),
      declare_parameter<double>("phase_sync_weight", 1.5),
      declare_parameter<double>("control_smooth_weight", 0.3),
      declare_parameter<double>("lateral_velocity_weight", 2.0),
      declare_parameter<double>("yaw_rate_tracking_weight", 0.6),
      declare_parameter<double>("roll_weight", 2.0),
      declare_parameter<double>("pitch_weight", 2.0),
      declare_parameter<double>("roll_tracking_weight", 0.75),
      declare_parameter<double>("pitch_tracking_weight", 0.75),
      declare_parameter<double>("phase_rate_limit_rad_s", 8.0),
      declare_parameter<double>("phase_sync_gain", 1.8),
      declare_parameter<double>("turn_phase_bias_gain", 0.8),
      declare_parameter<double>("minimum_speed_scale", 0.2),
      declare_parameter<double>("wave_turn_threshold_rps", 0.35),
      declare_parameter<double>("in_place_turn_threshold_rps", 0.10));

    body_sub_ = create_subscription<hexapod_control_interfaces::msg::KlannBodyState>(
      "hexapod/body_state", 10, std::bind(&KlannMppiControllerNode::on_body, this, std::placeholders::_1));
    plan_sub_ = create_subscription<nav_msgs::msg::Path>(
      "plan", 10, std::bind(&KlannMppiControllerNode::on_plan, this, std::placeholders::_1));

    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
    phase_pub_ = create_publisher<hexapod_control_interfaces::msg::LegPhaseCommand>("hexapod/phase_cmd", 10);

    timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&KlannMppiControllerNode::tick, this));
  }

private:
  void on_body(const hexapod_control_interfaces::msg::KlannBodyState::SharedPtr msg)
  {
    latest_body_ = msg;
  }

  void on_plan(const nav_msgs::msg::Path::SharedPtr msg)
  {
    latest_plan_ = *msg;
    received_first_plan_ = !latest_plan_.poses.empty();
    if (latest_plan_.poses.empty()) {
      RCLCPP_INFO(get_logger(), "Received empty plan; controller will hold zero motion.");
    } else {
      RCLCPP_INFO(get_logger(), "Received plan with %zu poses; controller active.", latest_plan_.poses.size());
    }
  }

  void tick()
  {
    if (!latest_body_) {
      return;
    }

    std::array<uint8_t, 6> motor_ids{};
    for (std::size_t i = 0; i < 6; ++i) {
      motor_ids[i] = static_cast<uint8_t>(motor_ids_vec_.at(i));
    }

    // With no non-empty plan, the core returns an explicit hold/zero command.
    (void)received_first_plan_;
    const auto result = core_.compute(latest_plan_, *latest_body_, motor_ids);
    twist_pub_->publish(result.cmd_vel);
    phase_pub_->publish(result.phase_cmd);
  }

  std::vector<int64_t> motor_ids_vec_;
  KlannControllerCore core_;
  nav_msgs::msg::Path latest_plan_;
  bool received_first_plan_{false};
  hexapod_control_interfaces::msg::KlannBodyState::SharedPtr latest_body_;

  rclcpp::Subscription<hexapod_control_interfaces::msg::KlannBodyState>::SharedPtr body_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<hexapod_control_interfaces::msg::LegPhaseCommand>::SharedPtr phase_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace hexapod_nav_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hexapod_nav_cpp::KlannMppiControllerNode>());
  rclcpp::shutdown();
  return 0;
}
