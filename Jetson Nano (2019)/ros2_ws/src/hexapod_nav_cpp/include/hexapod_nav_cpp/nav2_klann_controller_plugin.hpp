#ifndef HEXAPOD_NAV_CPP__NAV2_KLANN_CONTROLLER_PLUGIN_HPP_
#define HEXAPOD_NAV_CPP__NAV2_KLANN_CONTROLLER_PLUGIN_HPP_

#include "hexapod_nav_cpp/klann_controller_core.hpp"

#include <nav2_core/controller.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

#include <hexapod_control_interfaces/msg/klann_body_state.hpp>
#include <hexapod_control_interfaces/msg/leg_phase_command.hpp>

#include <memory>
#include <string>

namespace hexapod_nav_cpp
{

class Nav2KlannController : public nav2_core::Controller
{
public:
  Nav2KlannController() = default;
  ~Nav2KlannController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  void on_body_state(const hexapod_control_interfaces::msg::KlannBodyState::SharedPtr msg);

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path plan_;
  std::shared_ptr<KlannControllerCore> core_;
  rclcpp::Subscription<hexapod_control_interfaces::msg::KlannBodyState>::SharedPtr body_sub_;
  rclcpp::Publisher<hexapod_control_interfaces::msg::LegPhaseCommand>::SharedPtr phase_pub_;
  hexapod_control_interfaces::msg::KlannBodyState::SharedPtr latest_body_state_;
  std::string name_;
  std::array<uint8_t, 6> motor_ids_{1, 2, 3, 4, 5, 6};
  double linear_speed_limit_mps_{0.25};
};

}  // namespace hexapod_nav_cpp

#endif  // HEXAPOD_NAV_CPP__NAV2_KLANN_CONTROLLER_PLUGIN_HPP_
