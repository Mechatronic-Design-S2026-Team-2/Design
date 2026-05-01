#include "hexapod_hardware_cpp/lcd1602_i2c.hpp"

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <hexapod_control_interfaces/msg/klann_body_state.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include <iomanip>
#include <sstream>
#include <string>

namespace hexapod_hardware_cpp
{

class Lcd1602StatusNode : public rclcpp::Node
{
public:
  Lcd1602StatusNode()
  : Node("lcd1602_status_node")
  {
    i2c_device_ = declare_parameter<std::string>("i2c_device", "/dev/i2c-1");
    i2c_address_ = declare_parameter<int>("i2c_address", 0x27);

    lcd_.open(i2c_device_, i2c_address_);
    lcd_.initialize();

    body_sub_ = create_subscription<hexapod_control_interfaces::msg::KlannBodyState>(
      "hexapod/body_state", 10, std::bind(&Lcd1602StatusNode::on_body, this, std::placeholders::_1));
    plan_sub_ = create_subscription<nav_msgs::msg::Path>(
      "plan", 10, std::bind(&Lcd1602StatusNode::on_plan, this, std::placeholders::_1));
    cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel", 10, std::bind(&Lcd1602StatusNode::on_cmd, this, std::placeholders::_1));

    timer_ = create_wall_timer(std::chrono::milliseconds(250), std::bind(&Lcd1602StatusNode::refresh, this));
  }

private:
  void on_body(const hexapod_control_interfaces::msg::KlannBodyState::SharedPtr msg)
  {
    latest_body_ = msg;
  }

  void on_plan(const nav_msgs::msg::Path::SharedPtr msg)
  {
    latest_plan_size_ = msg->poses.size();
  }

  void on_cmd(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    latest_cmd_ = msg;
  }

  void refresh()
  {
    lcd_.clear();
    std::stringstream line0;
    std::stringstream line1;

    if (latest_body_) {
      line0 << "x" << std::fixed << std::setprecision(1) << latest_body_->pose.position.x
            << " y" << latest_body_->pose.position.y;
      line1 << "s" << latest_body_->stance_leg_count
            << " y" << std::fixed << std::setprecision(1) << latest_body_->yaw_rad;
      if (latest_cmd_) {
        line1 << " v" << std::fixed << std::setprecision(1) << latest_cmd_->twist.linear.x;
      } else {
        line1 << " p" << latest_plan_size_;
      }
    } else {
      line0 << "hexapod init";
      line1 << "wait state";
    }

    std::string s0 = line0.str().substr(0, 16);
    std::string s1 = line1.str().substr(0, 16);
    lcd_.set_cursor(0, 0);
    lcd_.print(s0);
    lcd_.set_cursor(1, 0);
    lcd_.print(s1);
  }

  std::string i2c_device_;
  int i2c_address_{0x27};

  Lcd1602I2c lcd_;
  hexapod_control_interfaces::msg::KlannBodyState::SharedPtr latest_body_;
  geometry_msgs::msg::TwistStamped::SharedPtr latest_cmd_;
  std::size_t latest_plan_size_{0};

  rclcpp::Subscription<hexapod_control_interfaces::msg::KlannBodyState>::SharedPtr body_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace hexapod_hardware_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hexapod_hardware_cpp::Lcd1602StatusNode>());
  rclcpp::shutdown();
  return 0;
}
