#include <dsy_motor_msgs/msg/motor_rpm_array.hpp>
#include <hexapod_control_interfaces/msg/hexapod_motor_state.hpp>
#include <hexapod_control_interfaces/msg/leg_actuation_state.hpp>
#include <hexapod_control_interfaces/msg/leg_phase_command.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
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

std::array<std::size_t, 6> parse_index_map(
  const std::vector<int64_t> & values,
  const std::array<std::size_t, 6> & fallback)
{
  if (values.size() != 6U) {
    return fallback;
  }
  std::array<std::size_t, 6> out{};
  std::array<bool, 6> seen{};
  for (std::size_t i = 0; i < 6U; ++i) {
    if (values[i] < 0 || values[i] > 5) {
      return fallback;
    }
    const auto idx = static_cast<std::size_t>(values[i]);
    if (seen[idx]) {
      return fallback;
    }
    out[i] = idx;
    seen[idx] = true;
  }
  return out;
}

std::array<double, 6> parse_double_array(
  const std::vector<double> & values,
  const std::array<double, 6> & fallback)
{
  if (values.size() != 6U) {
    return fallback;
  }
  std::array<double, 6> out{};
  for (std::size_t i = 0; i < 6U; ++i) {
    if (!std::isfinite(values[i])) {
      return fallback;
    }
    out[i] = values[i];
  }
  return out;
}

constexpr std::array<std::size_t, 6> kDefaultWireToCanonicalIndex{{2, 1, 0, 3, 4, 5}};
// wire [rb, rm, rf, lf, lm, lb] -> canonical [rf, rm, rb, lf, lm, lb]
constexpr std::array<double, 6> kDefaultModelToPhysicalMotorSign{{-1.0, -1.0, -1.0, 1.0, 1.0, 1.0}};
// canonical [rf, rm, rb, lf, lm, lb]; converts model-positive phase commands to physical motor RPM signs.

}  // namespace

class PhaseCommandRouterNode : public rclcpp::Node
{
public:
  PhaseCommandRouterNode()
  : Node("phase_command_router_node")
  {
    output_topic_ = declare_parameter<std::string>("output_topic", "/motor_rpm_cmd");
    actuation_state_topic_ = declare_parameter<std::string>("actuation_state_topic", "hexapod/actuation_state");
    wire_to_canonical_index_ = parse_index_map(
      declare_parameter<std::vector<int64_t>>(
        "wire_to_canonical_index", std::vector<int64_t>{2, 1, 0, 3, 4, 5}),
      kDefaultWireToCanonicalIndex);
    model_to_physical_motor_sign_ = parse_double_array(
      declare_parameter<std::vector<double>>(
        "model_to_physical_motor_sign", std::vector<double>{-1.0, -1.0, -1.0, 1.0, 1.0, 1.0}),
      kDefaultModelToPhysicalMotorSign);
    motor_rpm_ff_is_physical_ = declare_parameter<bool>("motor_rpm_ff_is_physical", false);
    gear_ratio_ = declare_parameter<double>("gear_ratio", 50.0);
    phase_kp_rpm_per_rad_ = declare_parameter<double>("phase_kp_rpm_per_rad", 300.0);
    phase_kd_rpm_per_rad_s_ = declare_parameter<double>("phase_kd_rpm_per_rad_s", 20.0);
    phase_target_max_error_rad_ = declare_parameter<double>("phase_target_max_error_rad", 0.8);
    rpm_limit_ = declare_parameter<double>("rpm_limit", 500.0);
    rpm_slew_limit_per_s_ = declare_parameter<double>("rpm_slew_limit_per_s", 9000.0);
    command_timeout_s_ = declare_parameter<double>("command_timeout_s", 0.25);
    publish_actuation_state_ = declare_parameter<bool>("publish_actuation_state", true);
    use_delay_compensation_ = declare_parameter<bool>("use_delay_compensation", true);
    default_measurement_age_s_ = declare_parameter<double>("default_measurement_age_s", 0.02);
    default_future_apply_delay_s_ = declare_parameter<double>("default_future_apply_delay_s", 0.02);
    measurement_age_gain_ = declare_parameter<double>("measurement_age_gain", 1.0);
    future_apply_delay_gain_ = declare_parameter<double>("future_apply_delay_gain", 1.0);
    publish_only_on_change_ = declare_parameter<bool>("publish_only_on_change", true);
    max_publish_rate_hz_ = declare_parameter<double>("max_publish_rate_hz", 50.0);
    publish_actuation_state_only_on_command_ = declare_parameter<bool>("publish_actuation_state_only_on_command", true);

    motor_state_sub_ = create_subscription<hexapod_control_interfaces::msg::HexapodMotorState>(
      "hexapod/motor_state", 10, std::bind(&PhaseCommandRouterNode::on_motor_state, this, std::placeholders::_1));
    cmd_sub_ = create_subscription<hexapod_control_interfaces::msg::LegPhaseCommand>(
      "hexapod/phase_cmd", 10, std::bind(&PhaseCommandRouterNode::on_phase_cmd, this, std::placeholders::_1));

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.best_effort();
    qos.durability_volatile();
    pub_ = create_publisher<dsy_motor_msgs::msg::MotorRpmArray>(output_topic_, qos);
    actuation_pub_ = create_publisher<hexapod_control_interfaces::msg::LegActuationState>(actuation_state_topic_, 10);

    RCLCPP_INFO(
      get_logger(),
      "wire_to_canonical_index=[%zu,%zu,%zu,%zu,%zu,%zu], model_to_physical_motor_sign=[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]",
      wire_to_canonical_index_[0], wire_to_canonical_index_[1], wire_to_canonical_index_[2],
      wire_to_canonical_index_[3], wire_to_canonical_index_[4], wire_to_canonical_index_[5],
      model_to_physical_motor_sign_[0], model_to_physical_motor_sign_[1], model_to_physical_motor_sign_[2],
      model_to_physical_motor_sign_[3], model_to_physical_motor_sign_[4], model_to_physical_motor_sign_[5]);

    timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&PhaseCommandRouterNode::publish_bridge, this));
  }

private:
  void on_motor_state(const hexapod_control_interfaces::msg::HexapodMotorState::SharedPtr msg)
  {
    latest_state_ = msg;
  }

  void on_phase_cmd(const hexapod_control_interfaces::msg::LegPhaseCommand::SharedPtr msg)
  {
    latest_cmd_ = msg;
    latest_cmd_time_ = now();
  }

  double output_rad_s_to_motor_rpm(double output_rad_s) const
  {
    return output_rad_s * gear_ratio_ * 60.0 / (2.0 * M_PI);
  }

  double motor_rpm_to_output_rad_s(double motor_rpm) const
  {
    return motor_rpm * (2.0 * M_PI) / (60.0 * gear_ratio_);
  }

  double clamp_rpm(double rpm) const
  {
    if (rpm > rpm_limit_) return rpm_limit_;
    if (rpm < -rpm_limit_) return -rpm_limit_;
    return rpm;
  }

  double apply_slew_limit(std::size_t idx, double rpm_cmd, double dt)
  {
    if (!has_last_sent_) {
      return rpm_cmd;
    }
    const double max_step = std::max(0.0, rpm_slew_limit_per_s_) * dt;
    const double delta = rpm_cmd - last_sent_rpm_[idx];
    if (delta > max_step) {
      return last_sent_rpm_[idx] + max_step;
    }
    if (delta < -max_step) {
      return last_sent_rpm_[idx] - max_step;
    }
    return rpm_cmd;
  }

  void publish_bridge()
  {
    const rclcpp::Time stamp = now();
    const bool fresh_cmd = latest_cmd_ && ((stamp - latest_cmd_time_).seconds() <= command_timeout_s_);
    double dt = 0.01;
    if (last_publish_time_.nanoseconds() > 0) {
      dt = std::max(1.0e-3, (stamp - last_publish_time_).seconds());
    }

    const double measurement_age_s = (use_delay_compensation_ && latest_state_) ?
      measurement_age_gain_ * static_cast<double>(latest_state_->estimated_measurement_age_s) :
      default_measurement_age_s_;
    const double future_apply_delay_s = (use_delay_compensation_ && latest_state_) ?
      future_apply_delay_gain_ * static_cast<double>(latest_state_->estimated_future_command_apply_delay_s) :
      default_future_apply_delay_s_;
    const double total_preview_s = std::max(0.0, measurement_age_s + future_apply_delay_s);

    dsy_motor_msgs::msg::MotorRpmArray out;
    out.command_seq = next_command_seq_;

    hexapod_control_interfaces::msg::LegActuationState actuation;
    actuation.header.stamp = stamp;
    actuation.command_seq = out.command_seq;
    actuation.estimated_measurement_age_s = static_cast<float>(measurement_age_s);
    actuation.estimated_future_command_apply_delay_s = static_cast<float>(future_apply_delay_s);
    actuation.estimated_total_preview_s = static_cast<float>(total_preview_s);

    std::array<double, 6> candidate_sent_rpm = last_sent_rpm_;
    std::array<int16_t, 6> candidate_wire_rpm{};

    for (std::size_t wire_idx = 0; wire_idx < 6; ++wire_idx) {
      const std::size_t idx = wire_to_canonical_index_[wire_idx];
      actuation.motor_id[idx] = latest_cmd_ ? latest_cmd_->motor_id[idx] : static_cast<uint8_t>(idx + 1);

      double model_rpm_cmd = 0.0;
      double commanded_phase_velocity = 0.0;
      double phase_target = (latest_state_ != nullptr) ? latest_state_->phase_rad[idx] : 0.0;
      double state_phase = phase_target;
      double state_phase_rate = (latest_state_ != nullptr) ? latest_state_->phase_velocity_rad_s[idx] : 0.0;
      const double sign = model_to_physical_motor_sign_[idx] >= 0.0 ? 1.0 : -1.0;

      if (use_delay_compensation_ && latest_state_) {
        state_phase += state_phase_rate * measurement_age_s;
      }

      if (fresh_cmd && latest_cmd_) {
        if (latest_cmd_->use_motor_rpm_ff) {
          const double rpm_ff = latest_cmd_->motor_rpm_ff[idx];
          const double model_rpm_ff = motor_rpm_ff_is_physical_ ? sign * rpm_ff : rpm_ff;
          model_rpm_cmd += model_rpm_ff;
          commanded_phase_velocity += motor_rpm_to_output_rad_s(model_rpm_ff);
        }
        if (latest_cmd_->use_phase_velocity) {
          commanded_phase_velocity += latest_cmd_->phase_velocity_rad_s[idx];
          model_rpm_cmd += output_rad_s_to_motor_rpm(latest_cmd_->phase_velocity_rad_s[idx]);
        }

        if (latest_cmd_->use_phase_targets) {
          const double target_phase_now = static_cast<double>(latest_cmd_->phase_target_rad[idx]);
          const double target_phase_at_apply = target_phase_now + commanded_phase_velocity * future_apply_delay_s;
          const double predicted_phase_at_apply = state_phase + state_phase_rate * future_apply_delay_s;
          const double unclamped_phase_error = wrap_to_pi(target_phase_at_apply - predicted_phase_at_apply);
          const double phase_error = std::max(-phase_target_max_error_rad_, std::min(phase_target_max_error_rad_, unclamped_phase_error));
          const double phase_velocity_error = commanded_phase_velocity - state_phase_rate;
          model_rpm_cmd += phase_kp_rpm_per_rad_ * phase_error;
          model_rpm_cmd += phase_kd_rpm_per_rad_s_ * phase_velocity_error;
          phase_target = target_phase_at_apply;
        }
      }

      double rpm_cmd = sign * model_rpm_cmd;
      rpm_cmd = clamp_rpm(rpm_cmd);
      rpm_cmd = apply_slew_limit(idx, rpm_cmd, dt);
      rpm_cmd = clamp_rpm(rpm_cmd);

      const int16_t rounded_rpm = static_cast<int16_t>(std::lrint(rpm_cmd));
      out.motor_rpm[wire_idx] = rounded_rpm;
      candidate_wire_rpm[wire_idx] = rounded_rpm;
      candidate_sent_rpm[idx] = static_cast<double>(rounded_rpm);

      actuation.commanded_motor_rpm[idx] = static_cast<float>(rounded_rpm);
      actuation.predicted_phase_velocity_rad_s[idx] =
        static_cast<float>(sign * motor_rpm_to_output_rad_s(rounded_rpm));
      actuation.commanded_phase_velocity_rad_s[idx] = static_cast<float>(commanded_phase_velocity);
      actuation.phase_target_rad[idx] = static_cast<float>(phase_target);
      actuation.fresh[idx] = fresh_cmd;
    }

    bool changed = !has_last_published_bridge_command_;
    for (std::size_t i = 0; i < 6; ++i) {
      if (candidate_wire_rpm[i] != last_published_wire_rpm_[i]) {
        changed = true;
      }
    }

    if (!fresh_cmd && !has_last_published_bridge_command_) {
      changed = false;
    }

    const bool should_publish = !publish_only_on_change_ || changed;
    if (!should_publish) {
      return;
    }

    if (has_last_published_bridge_command_ && max_publish_rate_hz_ > 0.0) {
      const double min_publish_period_s = 1.0 / max_publish_rate_hz_;
      const double time_since_last_publish_s = (stamp - last_publish_time_).seconds();
      if (time_since_last_publish_s < min_publish_period_s) {
        return;
      }
    }

    pub_->publish(out);
    if (publish_actuation_state_ && (!publish_actuation_state_only_on_command_ || should_publish)) {
      actuation_pub_->publish(actuation);
    }

    last_sent_rpm_ = candidate_sent_rpm;
    last_published_wire_rpm_ = candidate_wire_rpm;
    has_last_sent_ = true;
    has_last_published_bridge_command_ = true;
    last_publish_time_ = stamp;
    next_command_seq_++;
  }

  std::string output_topic_;
  std::string actuation_state_topic_;
  std::array<std::size_t, 6> wire_to_canonical_index_{kDefaultWireToCanonicalIndex};
  std::array<double, 6> model_to_physical_motor_sign_{kDefaultModelToPhysicalMotorSign};
  bool motor_rpm_ff_is_physical_{false};
  double gear_ratio_{50.0};
  double phase_kp_rpm_per_rad_{300.0};
  double phase_kd_rpm_per_rad_s_{20.0};
  double phase_target_max_error_rad_{0.8};
  double rpm_limit_{500.0};
  double rpm_slew_limit_per_s_{9000.0};
  double command_timeout_s_{0.25};
  bool publish_actuation_state_{true};
  bool use_delay_compensation_{true};
  double default_measurement_age_s_{0.02};
  double default_future_apply_delay_s_{0.02};
  double measurement_age_gain_{1.0};
  double future_apply_delay_gain_{1.0};
  bool publish_only_on_change_{true};
  double max_publish_rate_hz_{50.0};
  bool publish_actuation_state_only_on_command_{true};

  uint32_t next_command_seq_{1};
  rclcpp::Subscription<hexapod_control_interfaces::msg::HexapodMotorState>::SharedPtr motor_state_sub_;
  rclcpp::Subscription<hexapod_control_interfaces::msg::LegPhaseCommand>::SharedPtr cmd_sub_;
  rclcpp::Publisher<dsy_motor_msgs::msg::MotorRpmArray>::SharedPtr pub_;
  rclcpp::Publisher<hexapod_control_interfaces::msg::LegActuationState>::SharedPtr actuation_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  hexapod_control_interfaces::msg::HexapodMotorState::SharedPtr latest_state_;
  hexapod_control_interfaces::msg::LegPhaseCommand::SharedPtr latest_cmd_;
  rclcpp::Time latest_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
  std::array<double, 6> last_sent_rpm_{};
  std::array<int16_t, 6> last_published_wire_rpm_{};
  bool has_last_sent_{false};
  bool has_last_published_bridge_command_{false};
};

}  // namespace hexapod_hardware_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hexapod_hardware_cpp::PhaseCommandRouterNode>());
  rclcpp::shutdown();
  return 0;
}
