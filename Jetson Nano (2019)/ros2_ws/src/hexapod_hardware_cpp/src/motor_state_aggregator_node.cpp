#include <dsy_motor_msgs/msg/motor_output_odometry_array.hpp>
#include <dsy_motor_msgs/msg/motor_rpm_array.hpp>
#include <hexapod_control_interfaces/msg/hexapod_motor_state.hpp>
#include <hexapod_control_interfaces/msg/leg_actuation_state.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <string>
#include <unordered_map>
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

float as_float_seconds(uint64_t dt_us)
{
  return static_cast<float>(1.0e-6 * static_cast<double>(dt_us));
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

constexpr std::array<std::size_t, 6> kDefaultCanonicalToWireIndex{{2, 1, 0, 3, 4, 5}};
// canonical [rf, rm, rb, lf, lm, lb] <- wire [rb, rm, rf, lf, lm, lb]
constexpr std::array<std::size_t, 6> kDefaultWireToCanonicalIndex{{2, 1, 0, 3, 4, 5}};
// wire [rb, rm, rf, lf, lm, lb] -> canonical [rf, rm, rb, lf, lm, lb]
constexpr std::array<double, 6> kDefaultPhysicalToModelPhaseSign{{-1.0, -1.0, -1.0, 1.0, 1.0, 1.0}};
// canonical [rf, rm, rb, lf, lm, lb]; right-side physical encoder/RPM direction is opposite model-positive phase.
constexpr std::array<double, 6> kDefaultPhaseOffsetRad{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

}  // namespace

class MotorStateAggregatorNode : public rclcpp::Node
{
public:
  MotorStateAggregatorNode()
  : Node("motor_state_aggregator_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/motor_output_odom");
    direct_rpm_command_topic_ = declare_parameter<std::string>("direct_rpm_command_topic", "/motor_rpm_cmd");
    subscribe_direct_rpm_commands_ = declare_parameter<bool>("subscribe_direct_rpm_commands", true);
    hold_last_direct_rpm_command_ = declare_parameter<bool>("hold_last_direct_rpm_command", true);
    direct_rpm_command_timeout_s_ = declare_parameter<double>("direct_rpm_command_timeout_s", 1.0);
    motor_id_map_ = declare_parameter<std::vector<int64_t>>(
      "motor_id_map", std::vector<int64_t>{3, 2, 1, 4, 5, 6});
    canonical_to_wire_index_ = parse_index_map(
      declare_parameter<std::vector<int64_t>>(
        "canonical_to_wire_index", std::vector<int64_t>{2, 1, 0, 3, 4, 5}),
      kDefaultCanonicalToWireIndex);
    wire_to_canonical_index_ = parse_index_map(
      declare_parameter<std::vector<int64_t>>(
        "wire_to_canonical_index", std::vector<int64_t>{2, 1, 0, 3, 4, 5}),
      kDefaultWireToCanonicalIndex);
    physical_to_model_phase_sign_ = parse_double_array(
      declare_parameter<std::vector<double>>(
        "physical_to_model_phase_sign", std::vector<double>{-1.0, -1.0, -1.0, 1.0, 1.0, 1.0}),
      kDefaultPhysicalToModelPhaseSign);
    phase_offset_rad_ = parse_double_array(
      declare_parameter<std::vector<double>>(
        "phase_offset_rad", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      kDefaultPhaseOffsetRad);
    actuation_state_topic_ = declare_parameter<std::string>("actuation_state_topic", "hexapod/actuation_state");
    stale_timeout_s_ = declare_parameter<double>("stale_timeout_s", 0.25);
    phase_rate_from_phase_diff_weight_ = declare_parameter<double>("phase_rate_from_phase_diff_weight", 0.75);
    phase_rate_from_command_weight_ = declare_parameter<double>("phase_rate_from_command_weight", 0.25);
    phase_rate_lowpass_alpha_ = declare_parameter<double>("phase_rate_lowpass_alpha", 0.4);
    max_phase_rate_rad_s_ = declare_parameter<double>("max_phase_rate_rad_s", 8.0);
    encoder_counts_per_motor_rev_ = declare_parameter<double>("encoder_counts_per_motor_rev", 131072.0);
    gearbox_ratio_ = declare_parameter<double>("gearbox_ratio", 50.0);
    raw_encoder_blend_alpha_ = declare_parameter<double>("raw_encoder_blend_alpha", 0.65);
    raw_encoder_count_quantization_ = declare_parameter<bool>("raw_encoder_count_quantization", true);
    raw_encoder_change_threshold_counts_ = declare_parameter<double>("raw_encoder_change_threshold_counts", 0.5);
    ignore_unchanged_raw_encoder_when_commanded_ = declare_parameter<bool>("ignore_unchanged_raw_encoder_when_commanded", true);
    motor_speed_accel_ms_ = declare_parameter<double>("motor_speed_accel_ms", 250.0);
    motor_speed_decel_ms_ = declare_parameter<double>("motor_speed_decel_ms", 250.0);
    motor_speed_accel_rpm_per_s_ = declare_parameter<double>("motor_speed_accel_rpm_per_s", 0.0);
    motor_speed_decel_rpm_per_s_ = declare_parameter<double>("motor_speed_decel_rpm_per_s", 0.0);
    phase_prediction_innovation_limit_turns_ = declare_parameter<double>("phase_prediction_innovation_limit_turns", 0.75);
    near_zero_command_phase_rate_rad_s_ = declare_parameter<double>("near_zero_command_phase_rate_rad_s", 0.05);
    transport_split_to_host_ = declare_parameter<double>("transport_split_to_host", 0.5);
    delay_lowpass_alpha_ = declare_parameter<double>("delay_lowpass_alpha", 0.25);
    default_measurement_age_s_ = declare_parameter<double>("default_measurement_age_s", 0.02);
    default_future_apply_delay_s_ = declare_parameter<double>("default_future_apply_delay_s", 0.02);
    max_history_size_ = declare_parameter<int>("max_history_size", 128);

    pub_ = create_publisher<hexapod_control_interfaces::msg::HexapodMotorState>("hexapod/motor_state", 10);

    sub_ = create_subscription<dsy_motor_msgs::msg::MotorOutputOdometryArray>(
      input_topic_, rclcpp::SensorDataQoS(), std::bind(&MotorStateAggregatorNode::on_odometry, this, std::placeholders::_1));

    actuation_sub_ = create_subscription<hexapod_control_interfaces::msg::LegActuationState>(
      actuation_state_topic_, 10, std::bind(&MotorStateAggregatorNode::on_actuation_state, this, std::placeholders::_1));

    if (subscribe_direct_rpm_commands_) {
      direct_rpm_sub_ = create_subscription<dsy_motor_msgs::msg::MotorRpmArray>(
        direct_rpm_command_topic_, rclcpp::SensorDataQoS(),
        std::bind(&MotorStateAggregatorNode::on_direct_rpm_command, this, std::placeholders::_1));
    }

    RCLCPP_INFO(
      get_logger(),
      "canonical_to_wire_index=[%zu,%zu,%zu,%zu,%zu,%zu], physical_to_model_phase_sign=[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]",
      canonical_to_wire_index_[0], canonical_to_wire_index_[1], canonical_to_wire_index_[2],
      canonical_to_wire_index_[3], canonical_to_wire_index_[4], canonical_to_wire_index_[5],
      physical_to_model_phase_sign_[0], physical_to_model_phase_sign_[1], physical_to_model_phase_sign_[2],
      physical_to_model_phase_sign_[3], physical_to_model_phase_sign_[4], physical_to_model_phase_sign_[5]);

    timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&MotorStateAggregatorNode::publish_combined, this));
  }

private:
  struct LegVelocityEstimate
  {
    double previous_output_phase_rad{0.0};
    double previous_raw_encoder_count_mod{0.0};
    double continuous_motor_counts_est{0.0};
    uint64_t previous_poll_complete_time_us{0};
    double last_estimated_phase_rate_rad_s{0.0};
    double estimated_motor_rpm{0.0};
    bool has_previous_raw_encoder_count{false};
    bool initialized{false};
  };

  struct SentCommandInfo
  {
    rclcpp::Time host_send_time{0, 0, RCL_ROS_TIME};
    std::array<double, 6> commanded_motor_rpm{};
    std::array<double, 6> predicted_phase_velocity_rad_s{};
    std::array<bool, 6> fresh{};
    float estimated_measurement_age_s{0.0f};
    float estimated_future_command_apply_delay_s{0.0f};
    float estimated_total_preview_s{0.0f};
  };

  void on_odometry(const dsy_motor_msgs::msg::MotorOutputOdometryArray::SharedPtr msg)
  {
    latest_ = msg;
    latest_receive_time_ = now();
  }

  void on_actuation_state(const hexapod_control_interfaces::msg::LegActuationState::SharedPtr msg)
  {
    SentCommandInfo info;
    info.host_send_time = rclcpp::Time(msg->header.stamp);
    info.estimated_measurement_age_s = msg->estimated_measurement_age_s;
    info.estimated_future_command_apply_delay_s = msg->estimated_future_command_apply_delay_s;
    info.estimated_total_preview_s = msg->estimated_total_preview_s;
    for (std::size_t i = 0; i < 6; ++i) {
      info.commanded_motor_rpm[i] = msg->commanded_motor_rpm[i];
      info.predicted_phase_velocity_rad_s[i] = msg->predicted_phase_velocity_rad_s[i];
      info.fresh[i] = msg->fresh[i];
    }
    sent_history_[msg->command_seq] = info;
    sent_seq_order_.push_back(msg->command_seq);
    while (static_cast<int>(sent_seq_order_.size()) > max_history_size_) {
      const auto oldest = sent_seq_order_.front();
      sent_seq_order_.pop_front();
      sent_history_.erase(oldest);
    }
  }

  void on_direct_rpm_command(const dsy_motor_msgs::msg::MotorRpmArray::SharedPtr msg)
  {
    SentCommandInfo info;
    info.host_send_time = now();
    info.estimated_measurement_age_s = static_cast<float>(default_measurement_age_s_);
    info.estimated_future_command_apply_delay_s = static_cast<float>(default_future_apply_delay_s_);
    info.estimated_total_preview_s = static_cast<float>(default_measurement_age_s_ + default_future_apply_delay_s_);

    for (std::size_t wire_idx = 0; wire_idx < 6; ++wire_idx) {
      const std::size_t canonical_idx = wire_to_canonical_index_[wire_idx];
      const double rpm = static_cast<double>(msg->motor_rpm[wire_idx]);
      info.commanded_motor_rpm[canonical_idx] = rpm;
      info.predicted_phase_velocity_rad_s[canonical_idx] = motor_rpm_to_output_phase_rate(rpm);
      info.fresh[canonical_idx] = true;
    }

    latest_direct_command_ = info;
    latest_direct_command_seq_ = msg->command_seq;
    latest_direct_command_time_ = info.host_send_time;
    has_latest_direct_command_ = true;

    direct_command_history_[msg->command_seq] = info;
    direct_seq_order_.push_back(msg->command_seq);
    while (static_cast<int>(direct_seq_order_.size()) > max_history_size_) {
      const auto oldest = direct_seq_order_.front();
      direct_seq_order_.pop_front();
      direct_command_history_.erase(oldest);
    }
  }

  double clamp_phase_rate(double phase_rate_rad_s) const
  {
    if (phase_rate_rad_s > max_phase_rate_rad_s_) {
      return max_phase_rate_rad_s_;
    }
    if (phase_rate_rad_s < -max_phase_rate_rad_s_) {
      return -max_phase_rate_rad_s_;
    }
    return phase_rate_rad_s;
  }

  double shortest_modulo_count_delta(
    double newer_count,
    double older_count,
    double counts_per_rev) const
  {
    const double safe_counts_per_rev = std::max(1.0, counts_per_rev);
    double delta = newer_count - older_count;
    delta -= safe_counts_per_rev * std::round(delta / safe_counts_per_rev);
    return delta;
  }

  double motor_rpm_to_output_phase_rate(double motor_rpm) const
  {
    const double gear_ratio = std::max(1.0, gearbox_ratio_);
    return motor_rpm * (2.0 * M_PI / 60.0) / gear_ratio;
  }

  double configured_accel_rpm_per_s() const
  {
    if (motor_speed_accel_rpm_per_s_ > 1.0e-6) {
      return motor_speed_accel_rpm_per_s_;
    }
    return (motor_speed_accel_ms_ > 1.0e-6) ? (1.0e6 / motor_speed_accel_ms_) : 1.0e9;
  }

  double configured_decel_rpm_per_s() const
  {
    if (motor_speed_decel_rpm_per_s_ > 1.0e-6) {
      return motor_speed_decel_rpm_per_s_;
    }
    return (motor_speed_decel_ms_ > 1.0e-6) ? (1.0e6 / motor_speed_decel_ms_) : 1.0e9;
  }

  double propagate_ramp_limited_motor_rpm(double current_rpm, double target_rpm, double dt_s) const
  {
    if (dt_s <= 1.0e-9) {
      return current_rpm;
    }

    const double accel_rpm_per_s = configured_accel_rpm_per_s();
    const double decel_rpm_per_s = configured_decel_rpm_per_s();

    double rate_limit = decel_rpm_per_s;
    if ((current_rpm == 0.0) || ((current_rpm > 0.0) == (target_rpm > 0.0))) {
      rate_limit = (std::abs(target_rpm) >= std::abs(current_rpm)) ? accel_rpm_per_s : decel_rpm_per_s;
    }

    const double max_delta = std::max(0.0, rate_limit) * dt_s;
    const double delta = std::clamp(target_rpm - current_rpm, -max_delta, max_delta);
    return current_rpm + delta;
  }


  double blend_phase_rate(
    std::size_t leg_index,
    double & estimated_output_phase_rad,
    double raw_encoder_count_mod,
    uint64_t poll_complete_time_us,
    const SentCommandInfo * matched_command,
    double & phase_rate_from_phase_diff,
    double & phase_rate_from_command)
  {
    phase_rate_from_phase_diff = 0.0;
    phase_rate_from_command = 0.0;

    auto & state = leg_velocity_state_[leg_index];
    const double counts_per_rev = std::max(1.0, encoder_counts_per_motor_rev_);
    const double gear_ratio = std::max(1.0, gearbox_ratio_);
    const double counts_per_output_rev = counts_per_rev * gear_ratio;
    const double output_phase_per_motor_count = 2.0 * M_PI / counts_per_output_rev;
    const double motor_counts_per_output_phase = counts_per_output_rev / (2.0 * M_PI);
    const double model_phase_sign = physical_to_model_phase_sign_[leg_index] >= 0.0 ? 1.0 : -1.0;
    const double model_phase_offset = phase_offset_rad_[leg_index];

    double raw_count = raw_encoder_count_mod;
    if (raw_encoder_count_quantization_) {
      raw_count = std::nearbyint(raw_count);
    }
    raw_count = std::fmod(raw_count, counts_per_rev);
    if (raw_count < 0.0) {
      raw_count += counts_per_rev;
    }

    if (!state.initialized) {
      state.continuous_motor_counts_est = raw_count;
      state.previous_raw_encoder_count_mod = raw_count;
      state.previous_output_phase_rad =
        model_phase_sign * state.continuous_motor_counts_est * output_phase_per_motor_count + model_phase_offset;
      state.previous_poll_complete_time_us = poll_complete_time_us;
      state.last_estimated_phase_rate_rad_s = 0.0;
      state.estimated_motor_rpm = 0.0;
      state.has_previous_raw_encoder_count = true;
      state.initialized = true;
      estimated_output_phase_rad = state.previous_output_phase_rad;
      return 0.0;
    }

    const double dt = (poll_complete_time_us > state.previous_poll_complete_time_us) ?
      (1.0e-6 * static_cast<double>(poll_complete_time_us - state.previous_poll_complete_time_us)) : 0.0;

    double target_motor_rpm = state.estimated_motor_rpm;
    if (matched_command != nullptr && matched_command->fresh[leg_index]) {
      target_motor_rpm = matched_command->commanded_motor_rpm[leg_index];
    }

    state.estimated_motor_rpm = propagate_ramp_limited_motor_rpm(state.estimated_motor_rpm, target_motor_rpm, dt);
    const double physical_phase_rate_from_command = motor_rpm_to_output_phase_rate(state.estimated_motor_rpm);
    phase_rate_from_command = model_phase_sign * physical_phase_rate_from_command;

    double predicted_motor_counts = state.continuous_motor_counts_est;
    if (dt > 1.0e-6) {
      predicted_motor_counts += physical_phase_rate_from_command * motor_counts_per_output_phase * dt;
    }

    const double raw_delta_counts = state.has_previous_raw_encoder_count ?
      shortest_modulo_count_delta(raw_count, state.previous_raw_encoder_count_mod, counts_per_rev) : 0.0;
    const bool raw_count_changed = !state.has_previous_raw_encoder_count ||
      (std::abs(raw_delta_counts) >= std::max(0.0, raw_encoder_change_threshold_counts_));
    const bool commanded_motion_expected =
      std::abs(phase_rate_from_command) >= near_zero_command_phase_rate_rad_s_;
    const bool use_raw_encoder_measurement = (dt > 1.0e-6) &&
      (!ignore_unchanged_raw_encoder_when_commanded_ || raw_count_changed || !commanded_motion_expected);

    state.continuous_motor_counts_est = predicted_motor_counts;
    if (use_raw_encoder_measurement) {
      const double nearest_turns = std::round((predicted_motor_counts - raw_count) / counts_per_rev);
      double measured_motor_counts = raw_count + counts_per_rev * nearest_turns;

      if (std::abs(phase_rate_from_command) < near_zero_command_phase_rate_rad_s_) {
        const double prev_turns = std::round((state.continuous_motor_counts_est - raw_count) / counts_per_rev);
        measured_motor_counts = raw_count + counts_per_rev * prev_turns;
      }

      const double innovation_turn_limit = std::max(0.05, phase_prediction_innovation_limit_turns_);
      const double innovation_limit_counts = innovation_turn_limit * counts_per_rev;
      const double innovation_counts = measured_motor_counts - predicted_motor_counts;
      if (std::abs(innovation_counts) > innovation_limit_counts) {
        measured_motor_counts = predicted_motor_counts + std::copysign(innovation_limit_counts, innovation_counts);
      }

      const double alpha = std::clamp(raw_encoder_blend_alpha_, 0.0, 1.0);
      state.continuous_motor_counts_est = predicted_motor_counts + alpha * (measured_motor_counts - predicted_motor_counts);
      state.previous_raw_encoder_count_mod = raw_count;
      state.has_previous_raw_encoder_count = true;
    }

    estimated_output_phase_rad =
      model_phase_sign * state.continuous_motor_counts_est * output_phase_per_motor_count + model_phase_offset;

    if (dt > 1.0e-6) {
      phase_rate_from_phase_diff = (estimated_output_phase_rad - state.previous_output_phase_rad) / dt;
    }

    double weighted_sum = 0.0;
    double weight_sum = 0.0;
    if (dt > 1.0e-6) {
      weighted_sum += phase_rate_from_phase_diff_weight_ * phase_rate_from_phase_diff;
      weight_sum += phase_rate_from_phase_diff_weight_;
    }
    weighted_sum += phase_rate_from_command_weight_ * phase_rate_from_command;
    weight_sum += phase_rate_from_command_weight_;

    double raw_estimate = state.last_estimated_phase_rate_rad_s;
    if (weight_sum > 1.0e-9) {
      raw_estimate = weighted_sum / weight_sum;
    }

    const double filtered = state.last_estimated_phase_rate_rad_s +
      phase_rate_lowpass_alpha_ * (raw_estimate - state.last_estimated_phase_rate_rad_s);

    state.previous_output_phase_rad = estimated_output_phase_rad;
    state.previous_poll_complete_time_us = poll_complete_time_us;
    state.last_estimated_phase_rate_rad_s = clamp_phase_rate(filtered);
    return state.last_estimated_phase_rate_rad_s;
  }

  void publish_combined()
  {
    const rclcpp::Time stamp = now();
    hexapod_control_interfaces::msg::HexapodMotorState out;
    out.header.stamp = stamp;

    if (!latest_) {
      pub_->publish(out);
      return;
    }

    const double age_s = (stamp - latest_receive_time_).seconds();
    const bool fresh = age_s <= stale_timeout_s_;
    const SentCommandInfo * matched_command = nullptr;
    auto it = sent_history_.find(latest_->last_cmd_seq);
    if (it != sent_history_.end()) {
      matched_command = &it->second;
    } else {
      auto direct_it = direct_command_history_.find(latest_->last_cmd_seq);
      if (direct_it != direct_command_history_.end()) {
        matched_command = &direct_it->second;
      } else if (has_latest_direct_command_) {
        const double direct_age_s = (stamp - latest_direct_command_time_).seconds();
        if (hold_last_direct_rpm_command_ || direct_age_s <= direct_rpm_command_timeout_s_) {
          matched_command = &latest_direct_command_;
        }
      }
    }

    const uint64_t t_cmd_rx = latest_->esp_cmd_rx_time_us;
    const uint64_t t_apply = latest_->esp_apply_complete_time_us;
    const uint64_t t_poll = latest_->esp_poll_complete_time_us;
    const uint64_t t_pub = latest_->esp_publish_time_us;

    const uint64_t dt_cmd_apply_us = (t_apply >= t_cmd_rx) ? (t_apply - t_cmd_rx) : 0;
    const uint64_t dt_apply_poll_us = (t_poll >= t_apply) ? (t_poll - t_apply) : 0;
    const uint64_t dt_poll_pub_us = (t_pub >= t_poll) ? (t_pub - t_poll) : 0;
    const uint64_t dt_cmd_pub_us = (t_pub >= t_cmd_rx) ? (t_pub - t_cmd_rx) : 0;

    out.last_cmd_seq = latest_->last_cmd_seq;
    out.battery_voltage_v = latest_->battery_voltage_v;
    out.internal_cmd_rx_to_apply_s = as_float_seconds(dt_cmd_apply_us);
    out.internal_apply_to_poll_s = as_float_seconds(dt_apply_poll_us);
    out.internal_poll_to_publish_s = as_float_seconds(dt_poll_pub_us);
    out.internal_cmd_rx_to_publish_s = as_float_seconds(dt_cmd_pub_us);

    if (last_poll_complete_time_us_ != 0 && t_poll > last_poll_complete_time_us_) {
      out.feedback_dt_s = as_float_seconds(t_poll - last_poll_complete_time_us_);
    } else {
      out.feedback_dt_s = 0.0f;
    }
    last_poll_complete_time_us_ = t_poll;

    double est_host_to_esp = smoothed_host_to_esp_transport_s_;
    double est_esp_to_host = smoothed_esp_to_host_transport_s_;
    double est_external_total = smoothed_total_external_transport_s_;
    double est_measurement_age = smoothed_measurement_age_s_ > 0.0 ?
      smoothed_measurement_age_s_ : default_measurement_age_s_;
    double est_future_apply_delay = smoothed_future_apply_delay_s_ > 0.0 ?
      smoothed_future_apply_delay_s_ : default_future_apply_delay_s_;
    double est_roundtrip = smoothed_roundtrip_s_;

    if (matched_command != nullptr) {
      const double host_roundtrip_s = std::max(0.0, (stamp - matched_command->host_send_time).seconds());
      const double internal_cmd_pub_s = static_cast<double>(out.internal_cmd_rx_to_publish_s);
      const double external_total_s = std::max(0.0, host_roundtrip_s - internal_cmd_pub_s);
      const double host_to_esp_s = (1.0 - transport_split_to_host_) * external_total_s;
      const double esp_to_host_s = transport_split_to_host_ * external_total_s;
      const double measurement_age_s = static_cast<double>(out.internal_poll_to_publish_s) + esp_to_host_s;
      const double future_apply_delay_s = host_to_esp_s + static_cast<double>(out.internal_cmd_rx_to_apply_s);

      smoothed_total_external_transport_s_ += delay_lowpass_alpha_ * (external_total_s - smoothed_total_external_transport_s_);
      smoothed_host_to_esp_transport_s_ += delay_lowpass_alpha_ * (host_to_esp_s - smoothed_host_to_esp_transport_s_);
      smoothed_esp_to_host_transport_s_ += delay_lowpass_alpha_ * (esp_to_host_s - smoothed_esp_to_host_transport_s_);
      smoothed_measurement_age_s_ += delay_lowpass_alpha_ * (measurement_age_s - smoothed_measurement_age_s_);
      smoothed_future_apply_delay_s_ += delay_lowpass_alpha_ * (future_apply_delay_s - smoothed_future_apply_delay_s_);
      smoothed_roundtrip_s_ += delay_lowpass_alpha_ * (host_roundtrip_s - smoothed_roundtrip_s_);

      est_external_total = smoothed_total_external_transport_s_;
      est_host_to_esp = smoothed_host_to_esp_transport_s_;
      est_esp_to_host = smoothed_esp_to_host_transport_s_;
      est_measurement_age = smoothed_measurement_age_s_;
      est_future_apply_delay = smoothed_future_apply_delay_s_;
      est_roundtrip = smoothed_roundtrip_s_;
    }

    out.estimated_total_external_transport_s = static_cast<float>(est_external_total);
    out.estimated_host_to_esp_transport_s = static_cast<float>(est_host_to_esp);
    out.estimated_esp_to_host_transport_s = static_cast<float>(est_esp_to_host);
    out.estimated_measurement_age_s = static_cast<float>(est_measurement_age);
    out.estimated_future_command_apply_delay_s = static_cast<float>(est_future_apply_delay);
    out.estimated_roundtrip_time_s = static_cast<float>(est_roundtrip);
    out.shared_enable = fresh;

    for (std::size_t i = 0; i < 6; ++i) {
      const std::size_t wire_idx = canonical_to_wire_index_[i];
      const double raw_encoder_count_mod = static_cast<double>(latest_->output_phase_rad[wire_idx]);
      double estimated_output_phase_rad = 0.0;

      out.motor_id[i] = static_cast<uint8_t>(motor_id_map_.at(i));
      out.comm_ok[i] = fresh;
      out.bus_voltage_v[i] = latest_->battery_voltage_v;
      out.phase_current_a[i] = 0.0f;
      out.servo_status_word[i] = 0U;
      out.modbus_exception_code[i] = 0U;
      out.fresh[i] = fresh;
      double phase_rate_from_phase_diff = 0.0;
      double phase_rate_from_command = 0.0;
      const double estimated_phase_rate = blend_phase_rate(
        i,
        estimated_output_phase_rad,
        raw_encoder_count_mod,
        latest_->esp_poll_complete_time_us,
        matched_command,
        phase_rate_from_phase_diff,
        phase_rate_from_command);

      out.motor_rpm[i] = static_cast<float>(leg_velocity_state_[i].estimated_motor_rpm);
      out.phase_rad[i] = static_cast<float>(estimated_output_phase_rad);
      out.phase_velocity_rad_s[i] = static_cast<float>(estimated_phase_rate);
      out.phase_velocity_from_phase_diff_rad_s[i] = static_cast<float>(phase_rate_from_phase_diff);
      out.phase_velocity_from_command_rad_s[i] = static_cast<float>(phase_rate_from_command);
    }

    pub_->publish(out);
  }

  std::string input_topic_;
  std::string direct_rpm_command_topic_;
  std::vector<int64_t> motor_id_map_;
  std::array<std::size_t, 6> canonical_to_wire_index_{kDefaultCanonicalToWireIndex};
  std::array<std::size_t, 6> wire_to_canonical_index_{kDefaultWireToCanonicalIndex};
  std::array<double, 6> physical_to_model_phase_sign_{kDefaultPhysicalToModelPhaseSign};
  std::array<double, 6> phase_offset_rad_{kDefaultPhaseOffsetRad};
  std::string actuation_state_topic_;
  bool subscribe_direct_rpm_commands_{true};
  bool hold_last_direct_rpm_command_{true};
  double direct_rpm_command_timeout_s_{1.0};
  double stale_timeout_s_{0.25};
  double phase_rate_from_phase_diff_weight_{0.75};
  double phase_rate_from_command_weight_{0.25};
  double phase_rate_lowpass_alpha_{0.4};
  double max_phase_rate_rad_s_{8.0};
  double encoder_counts_per_motor_rev_{131072.0};
  double gearbox_ratio_{50.0};
  double raw_encoder_blend_alpha_{0.65};
  bool raw_encoder_count_quantization_{true};
  double raw_encoder_change_threshold_counts_{0.5};
  bool ignore_unchanged_raw_encoder_when_commanded_{true};
  double motor_speed_accel_ms_{250.0};
  double motor_speed_decel_ms_{250.0};
  double motor_speed_accel_rpm_per_s_{0.0};
  double motor_speed_decel_rpm_per_s_{0.0};
  double phase_prediction_innovation_limit_turns_{0.75};
  double near_zero_command_phase_rate_rad_s_{0.05};
  double transport_split_to_host_{0.5};
  double delay_lowpass_alpha_{0.25};
  double default_measurement_age_s_{0.02};
  double default_future_apply_delay_s_{0.02};
  int max_history_size_{128};

  std::unordered_map<uint32_t, SentCommandInfo> sent_history_;
  std::deque<uint32_t> sent_seq_order_;
  std::unordered_map<uint32_t, SentCommandInfo> direct_command_history_;
  std::deque<uint32_t> direct_seq_order_;
  SentCommandInfo latest_direct_command_{};
  rclcpp::Time latest_direct_command_time_{0, 0, RCL_ROS_TIME};
  uint32_t latest_direct_command_seq_{0};
  bool has_latest_direct_command_{false};
  dsy_motor_msgs::msg::MotorOutputOdometryArray::SharedPtr latest_;
  rclcpp::Time latest_receive_time_{0, 0, RCL_ROS_TIME};
  uint64_t last_poll_complete_time_us_{0};
  std::array<LegVelocityEstimate, 6> leg_velocity_state_{};

  double smoothed_host_to_esp_transport_s_{0.0};
  double smoothed_esp_to_host_transport_s_{0.0};
  double smoothed_total_external_transport_s_{0.0};
  double smoothed_measurement_age_s_{0.0};
  double smoothed_future_apply_delay_s_{0.0};
  double smoothed_roundtrip_s_{0.0};

  rclcpp::Subscription<dsy_motor_msgs::msg::MotorOutputOdometryArray>::SharedPtr sub_;
  rclcpp::Subscription<hexapod_control_interfaces::msg::LegActuationState>::SharedPtr actuation_sub_;
  rclcpp::Subscription<dsy_motor_msgs::msg::MotorRpmArray>::SharedPtr direct_rpm_sub_;
  rclcpp::Publisher<hexapod_control_interfaces::msg::HexapodMotorState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace hexapod_hardware_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hexapod_hardware_cpp::MotorStateAggregatorNode>());
  rclcpp::shutdown();
  return 0;
}
