#include "hexapod_hardware_cpp/lcd1602_i2c.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <hexapod_control_interfaces/msg/klann_body_state.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>

namespace hexapod_hardware_cpp
{

namespace
{
constexpr std::size_t kLcdColumns = 16U;

std::string pad_or_truncate(std::string text)
{
  if (text.size() > kLcdColumns) {
    text.resize(kLcdColumns);
  }
  if (text.size() < kLcdColumns) {
    text.append(kLcdColumns - text.size(), ' ');
  }
  return text;
}

std::string trim_trailing_slash(std::string text)
{
  while (!text.empty() && text.back() == '/') {
    text.pop_back();
  }
  return text;
}

std::string extract_json_string(const std::string & json, const std::string & key, const std::string & fallback)
{
  const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"");
  std::smatch match;
  if (std::regex_search(json, match, pattern) && match.size() > 1U) {
    return match[1].str();
  }
  return fallback;
}

bool extract_json_number(const std::string & json, const std::string & key, double & value_out)
{
  const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*(-?[0-9]+(\\.[0-9]+)?([eE][-+]?[0-9]+)?)");
  std::smatch match;
  if (std::regex_search(json, match, pattern) && match.size() > 1U) {
    try {
      value_out = std::stod(match[1].str());
      return std::isfinite(value_out);
    } catch (const std::exception &) {
      return false;
    }
  }
  return false;
}

}  // namespace

class Lcd1602StatusNode : public rclcpp::Node
{
public:
  Lcd1602StatusNode()
  : Node("lcd1602_status_node")
  {
    enabled_ = declare_parameter<bool>("enabled", true);
    i2c_device_ = declare_parameter<std::string>("i2c_device", "/dev/i2c-1");
    i2c_address_ = declare_parameter<int>("i2c_address", 0x27);
    operator_url_ = declare_parameter<std::string>("operator_url", "http://192.168.50.1:8080/");
    static_url_only_ = declare_parameter<bool>("static_url_only", true);
    startup_line_ = declare_parameter<std::string>("startup_line", "Hexapod ready");
    status_json_topic_ = declare_parameter<std::string>("status_json_topic", "/hexapod/operator/status_json");
    battery_topic_ = declare_parameter<std::string>("battery_topic", "/hexapod/operator/battery_voltage");
    body_state_topic_ = declare_parameter<std::string>("body_state_topic", "hexapod/body_state");
    plan_topic_ = declare_parameter<std::string>("plan_topic", "plan");
    selected_cmd_topic_ = declare_parameter<std::string>("selected_cmd_topic", "/hexapod/cmd_vel_selected");
    page_period_sec_ = std::max(1.0, declare_parameter<double>("page_period_sec", 3.0));
    const int requested_refresh_period_ms = static_cast<int>(
      declare_parameter<int>("refresh_period_ms", 750));
    refresh_period_ms_ = std::max(250, requested_refresh_period_ms);
    reconnect_period_sec_ = std::max(1.0, declare_parameter<double>("reconnect_period_sec", 5.0));

    body_sub_ = create_subscription<hexapod_control_interfaces::msg::KlannBodyState>(
      body_state_topic_, 10, std::bind(&Lcd1602StatusNode::on_body, this, std::placeholders::_1));
    plan_sub_ = create_subscription<nav_msgs::msg::Path>(
      plan_topic_, 10, std::bind(&Lcd1602StatusNode::on_plan, this, std::placeholders::_1));
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      selected_cmd_topic_, 10, std::bind(&Lcd1602StatusNode::on_cmd, this, std::placeholders::_1));
    status_sub_ = create_subscription<std_msgs::msg::String>(
      status_json_topic_, 10, std::bind(&Lcd1602StatusNode::on_status_json, this, std::placeholders::_1));
    battery_sub_ = create_subscription<std_msgs::msg::Float32>(
      battery_topic_, 10, std::bind(&Lcd1602StatusNode::on_battery, this, std::placeholders::_1));

    try_connect_lcd(true);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(refresh_period_ms_),
      std::bind(&Lcd1602StatusNode::refresh, this));

    RCLCPP_INFO(
      get_logger(),
      "LCD1602 status node enabled=%s url='%s' i2c=%s addr=0x%02X status_json='%s'",
      enabled_ ? "true" : "false",
      operator_url_.c_str(),
      i2c_device_.c_str(),
      i2c_address_,
      status_json_topic_.c_str());
  }

private:
  void try_connect_lcd(bool first_attempt = false)
  {
    if (!enabled_ || lcd_ready_) {
      return;
    }

    const auto now = this->now();
    if (!first_attempt && last_connect_attempt_.nanoseconds() != 0 &&
      (now - last_connect_attempt_).seconds() < reconnect_period_sec_)
    {
      return;
    }
    last_connect_attempt_ = now;

    try {
      lcd_.open(i2c_device_, i2c_address_);
      lcd_.initialize();
      lcd_ready_ = true;
      last_line0_.clear();
      last_line1_.clear();
      RCLCPP_INFO(get_logger(), "LCD1602 connected on %s address 0x%02X", i2c_device_.c_str(), i2c_address_);
      std::string line0;
      std::string line1;
      make_url_page(line0, line1);
      write_lines(line0, line1);
    } catch (const std::exception & exc) {
      lcd_ready_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "LCD1602 unavailable on %s address 0x%02X: %s; continuing without display",
        i2c_device_.c_str(),
        i2c_address_,
        exc.what());
    }
  }

  void on_body(const hexapod_control_interfaces::msg::KlannBodyState::SharedPtr msg)
  {
    latest_body_ = msg;
  }

  void on_plan(const nav_msgs::msg::Path::SharedPtr msg)
  {
    latest_plan_size_ = msg->poses.size();
  }

  void on_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_cmd_ = msg;
  }

  void on_status_json(const std_msgs::msg::String::SharedPtr msg)
  {
    latest_status_json_ = msg->data;
    latest_mode_ = extract_json_string(msg->data, "mode", latest_mode_);
    latest_nav_status_ = extract_json_string(msg->data, "nav_goal_status", latest_nav_status_);
    latest_map_source_ = extract_json_string(msg->data, "map_source", latest_map_source_);
    double battery = 0.0;
    if (extract_json_number(msg->data, "battery_voltage_v", battery) && battery > 0.0) {
      latest_battery_v_ = battery;
      have_battery_ = true;
    }
  }

  void on_battery(const std_msgs::msg::Float32::SharedPtr msg)
  {
    const double battery = static_cast<double>(msg->data);
    if (std::isfinite(battery) && battery > 0.0) {
      latest_battery_v_ = battery;
      have_battery_ = true;
    }
  }

  void refresh()
  {
    if (!enabled_) {
      return;
    }
    if (!lcd_ready_) {
      try_connect_lcd(false);
      return;
    }

    if (static_url_only_) {
      // Keep the LCD electrically quiet after the initial URL write.
      // Repeated page updates on long I2C wiring/backpacks can leave stray
      // characters; all dynamic status is available in the browser UI.
      return;
    }

    const auto now = this->now();
    const std::int64_t page = static_cast<std::int64_t>(
      std::floor(now.seconds() / page_period_sec_)) % 4;

    std::string line0;
    std::string line1;

    if (page == 0) {
      make_url_page(line0, line1);
    } else if (page == 1) {
      make_operator_status_page(line0, line1);
    } else if (page == 2) {
      make_motion_page(line0, line1);
    } else {
      make_mapping_page(line0, line1);
    }

    write_lines(line0, line1);
  }

  void make_url_page(std::string & line0, std::string & line1) const
  {
    const std::string url = trim_trailing_slash(operator_url_);
    if (url.size() <= kLcdColumns) {
      line0 = "GUI address";
      line1 = url;
      return;
    }
    line0 = url.substr(0, kLcdColumns);
    line1 = url.substr(kLcdColumns, kLcdColumns);
  }

  void make_operator_status_page(std::string & line0, std::string & line1) const
  {
    line0 = "Mode " + latest_mode_;
    std::stringstream ss;
    if (have_battery_) {
      ss << "Bat " << std::fixed << std::setprecision(1) << latest_battery_v_ << "V";
    } else {
      ss << "Bat waiting";
    }
    line1 = ss.str();
  }

  void make_motion_page(std::string & line0, std::string & line1) const
  {
    if (latest_body_) {
      std::stringstream ss0;
      ss0 << "x" << std::fixed << std::setprecision(1) << latest_body_->pose.position.x
          << " y" << latest_body_->pose.position.y;
      line0 = ss0.str();

      std::stringstream ss1;
      ss1 << "st" << latest_body_->stance_leg_count
          << " yaw" << std::fixed << std::setprecision(1) << latest_body_->yaw_rad;
      line1 = ss1.str();
    } else {
      line0 = "Hexapod state";
      line1 = "waiting";
    }

    if (latest_cmd_) {
      std::stringstream cmd;
      cmd << "v" << std::fixed << std::setprecision(2) << latest_cmd_->linear.x
          << " w" << latest_cmd_->angular.z;
      line1 = cmd.str();
    }
  }

  void make_mapping_page(std::string & line0, std::string & line1) const
  {
    if (!latest_map_source_.empty() && latest_map_source_ != "null") {
      line0 = "Map " + latest_map_source_;
    } else {
      line0 = "Map waiting";
    }
    if (!latest_nav_status_.empty()) {
      line1 = "Nav " + latest_nav_status_;
    } else {
      std::stringstream ss;
      ss << "Plan " << latest_plan_size_;
      line1 = ss.str();
    }
  }

  void write_lines(const std::string & line0, const std::string & line1)
  {
    const std::string s0 = pad_or_truncate(line0);
    const std::string s1 = pad_or_truncate(line1);
    if (s0 == last_line0_ && s1 == last_line1_) {
      return;
    }

    try {
      lcd_.set_cursor(0, 0);
      lcd_.print(s0);
      lcd_.set_cursor(1, 0);
      lcd_.print(s1);
      last_line0_ = s0;
      last_line1_ = s1;
    } catch (const std::exception & exc) {
      lcd_ready_ = false;
      lcd_.close();
      RCLCPP_WARN(get_logger(), "LCD1602 write failed: %s; will retry", exc.what());
    }
  }

  bool enabled_{true};
  bool lcd_ready_{false};
  bool static_url_only_{true};
  std::string i2c_device_;
  int i2c_address_{0x27};
  std::string operator_url_;
  std::string startup_line_;
  std::string status_json_topic_;
  std::string battery_topic_;
  std::string body_state_topic_;
  std::string plan_topic_;
  std::string selected_cmd_topic_;
  double page_period_sec_{3.0};
  int refresh_period_ms_{750};
  double reconnect_period_sec_{5.0};

  Lcd1602I2c lcd_;
  rclcpp::Time last_connect_attempt_{0, 0, RCL_ROS_TIME};
  std::string last_line0_;
  std::string last_line1_;

  hexapod_control_interfaces::msg::KlannBodyState::SharedPtr latest_body_;
  geometry_msgs::msg::Twist::SharedPtr latest_cmd_;
  std::size_t latest_plan_size_{0};
  std::string latest_status_json_;
  std::string latest_mode_{"estop"};
  std::string latest_nav_status_{"idle"};
  std::string latest_map_source_;
  double latest_battery_v_{0.0};
  bool have_battery_{false};

  rclcpp::Subscription<hexapod_control_interfaces::msg::KlannBodyState>::SharedPtr body_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub_;
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
