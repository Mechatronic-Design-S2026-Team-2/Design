#include "hexapod_nav_cpp/klann_controller_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>

namespace hexapod_nav_cpp
{

namespace
{
constexpr double kSupportMarginTargetM = 0.03;
}

KlannControllerCore::KlannControllerCore()
{
  ensure_nominal_sequences();
}

double KlannControllerCore::clamp(double value, double lo, double hi)
{
  return std::max(lo, std::min(value, hi));
}

double KlannControllerCore::wrap_pi(double angle_rad)
{
  while (angle_rad > M_PI) angle_rad -= 2.0 * M_PI;
  while (angle_rad < -M_PI) angle_rad += 2.0 * M_PI;
  return angle_rad;
}

std::array<int, 6> KlannControllerCore::side_signs()
{
  return {-1, -1, -1, 1, 1, 1};
}

double KlannControllerCore::nearest_path_error(const nav_msgs::msg::Path & plan, double x, double y, double & heading_target)
{
  if (plan.poses.empty()) {
    heading_target = 0.0;
    return 0.0;
  }

  double best_d2 = std::numeric_limits<double>::infinity();
  std::size_t best_idx = 0;
  for (std::size_t i = 0; i < plan.poses.size(); ++i) {
    const double dx = x - plan.poses[i].pose.position.x;
    const double dy = y - plan.poses[i].pose.position.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best_idx = i;
    }
  }

  const std::size_t next_idx = std::min(best_idx + 1, plan.poses.size() - 1);
  const double hx = plan.poses[next_idx].pose.position.x - plan.poses[best_idx].pose.position.x;
  const double hy = plan.poses[next_idx].pose.position.y - plan.poses[best_idx].pose.position.y;
  heading_target = std::atan2(hy, hx);
  return std::sqrt(best_d2);
}

double KlannControllerCore::support_margin_from_points(
  const std::array<Eigen::Vector2d, 6> & feet,
  const std::array<double, 6> & stance)
{
  std::vector<Eigen::Vector2d> pts;
  pts.reserve(6);
  Eigen::Vector2d centroid(0.0, 0.0);
  for (std::size_t i = 0; i < 6; ++i) {
    if (stance[i] > 0.5) {
      pts.push_back(feet[i]);
      centroid += feet[i];
    }
  }
  if (pts.size() < 3) {
    return 0.0;
  }
  centroid /= static_cast<double>(pts.size());
  double min_margin = std::numeric_limits<double>::infinity();
  for (const auto & p : pts) {
    min_margin = std::min(min_margin, (p - centroid).norm());
  }
  return std::isfinite(min_margin) ? min_margin : 0.0;
}

KlannControllerCore::SupportPlaneEstimate KlannControllerCore::estimate_support_plane(
  const PredictedLegState & predicted)
{
  SupportPlaneEstimate out;
  Eigen::Matrix3d ata = Eigen::Matrix3d::Zero();
  Eigen::Vector3d atb = Eigen::Vector3d::Zero();
  int used = 0;

  for (std::size_t i = 0; i < 6; ++i) {
    const double w = clamp(predicted.stance[i], 0.0, 1.0);
    if (w < 0.35) {
      continue;
    }
    const double x = predicted.foot_xy[i].x();
    const double y = predicted.foot_xy[i].y();
    const double z = predicted.foot_z[i];
    const Eigen::Vector3d row(x, y, 1.0);
    ata += w * (row * row.transpose());
    atb += w * row * z;
    ++used;
  }

  if (used < 3 || std::abs(ata.determinant()) < 1.0e-9) {
    return out;
  }

  const Eigen::Vector3d coeff = ata.ldlt().solve(atb);
  const double a = coeff.x();
  const double b = coeff.y();
  out.pitch_ref_rad = -std::atan(a);
  out.roll_ref_rad = std::atan(b);
  out.margin = support_margin_from_points(predicted.foot_xy, predicted.stance);
  out.valid = std::isfinite(out.roll_ref_rad) && std::isfinite(out.pitch_ref_rad);
  return out;
}

void KlannControllerCore::set_parameters(
  double horizon_dt_s,
  int horizon_steps,
  int sample_count,
  double nominal_phase_rate_rad_s,
  double linear_speed_limit_mps,
  double angular_speed_limit_rps,
  double temperature,
  double vx_noise_std,
  double wz_noise_std,
  double phase_rate_noise_std,
  double path_weight,
  double heading_weight,
  double progress_weight,
  double support_weight,
  double support_margin_weight,
  double slip_weight,
  double phase_sync_weight,
  double control_smooth_weight,
  double lateral_velocity_weight,
  double yaw_rate_tracking_weight,
  double roll_weight,
  double pitch_weight,
  double roll_tracking_weight,
  double pitch_tracking_weight,
  double phase_rate_limit_rad_s,
  double phase_sync_gain,
  double turn_phase_bias_gain,
  double minimum_speed_scale,
  double wave_turn_threshold_rps,
  double in_place_turn_threshold_rps)
{
  horizon_dt_s_ = horizon_dt_s;
  horizon_steps_ = horizon_steps;
  sample_count_ = sample_count;
  nominal_phase_rate_rad_s_ = nominal_phase_rate_rad_s;
  linear_speed_limit_mps_ = linear_speed_limit_mps;
  angular_speed_limit_rps_ = angular_speed_limit_rps;
  temperature_ = std::max(1.0e-3, temperature);
  vx_noise_std_ = std::max(1.0e-4, vx_noise_std);
  wz_noise_std_ = std::max(1.0e-4, wz_noise_std);
  phase_rate_noise_std_ = std::max(1.0e-4, phase_rate_noise_std);
  path_weight_ = path_weight;
  heading_weight_ = heading_weight;
  progress_weight_ = progress_weight;
  support_weight_ = support_weight;
  support_margin_weight_ = support_margin_weight;
  slip_weight_ = slip_weight;
  phase_sync_weight_ = phase_sync_weight;
  control_smooth_weight_ = control_smooth_weight;
  lateral_velocity_weight_ = lateral_velocity_weight;
  yaw_rate_tracking_weight_ = yaw_rate_tracking_weight;
  roll_weight_ = roll_weight;
  pitch_weight_ = pitch_weight;
  roll_tracking_weight_ = roll_tracking_weight;
  pitch_tracking_weight_ = pitch_tracking_weight;
  phase_rate_limit_rad_s_ = phase_rate_limit_rad_s;
  gait_.set_parameters(
    phase_sync_gain,
    turn_phase_bias_gain,
    minimum_speed_scale,
    wave_turn_threshold_rps,
    in_place_turn_threshold_rps);
  ensure_nominal_sequences();
}

bool KlannControllerCore::initialize_lookup_models(const std::string & linkage_yaml_path)
{
  linkage_yaml_path_ = linkage_yaml_path;
  if (linkage_yaml_path.empty()) {
    lookup_models_.clear();
    have_lookup_models_ = false;
    return false;
  }
  lookup_models_ = hexapod_hardware_cpp::LinkageSolver::load_models_from_yaml(linkage_yaml_path);
  have_lookup_models_ = lookup_models_.size() >= 6;
  return have_lookup_models_;
}

void KlannControllerCore::ensure_nominal_sequences()
{
  if (static_cast<int>(nominal_vx_sequence_.size()) != horizon_steps_) {
    nominal_vx_sequence_.assign(horizon_steps_, 0.0);
  }
  if (static_cast<int>(nominal_wz_sequence_.size()) != horizon_steps_) {
    nominal_wz_sequence_.assign(horizon_steps_, 0.0);
  }
  if (static_cast<int>(nominal_phase_rate_sequence_.size()) != horizon_steps_) {
    nominal_phase_rate_sequence_.assign(static_cast<std::size_t>(horizon_steps_), PhaseRateStep{});
  }
}

void KlannControllerCore::warm_start_sequences(double nominal_vx, double nominal_wz)
{
  ensure_nominal_sequences();
  if (horizon_steps_ <= 1) {
    nominal_vx_sequence_[0] = nominal_vx;
    nominal_wz_sequence_[0] = nominal_wz;
    return;
  }
  std::rotate(nominal_vx_sequence_.begin(), nominal_vx_sequence_.begin() + 1, nominal_vx_sequence_.end());
  std::rotate(nominal_wz_sequence_.begin(), nominal_wz_sequence_.begin() + 1, nominal_wz_sequence_.end());
  std::rotate(nominal_phase_rate_sequence_.begin(), nominal_phase_rate_sequence_.begin() + 1, nominal_phase_rate_sequence_.end());
  nominal_vx_sequence_.back() = nominal_vx;
  nominal_wz_sequence_.back() = nominal_wz;
}

KlannControllerCore::PredictedLegState KlannControllerCore::predict_leg_state(
  const std::array<double, 6> & phases,
  const std::array<double, 6> & phase_rates,
  const hexapod_control_interfaces::msg::KlannBodyState & body_state)
{
  PredictedLegState predicted;
  for (std::size_t i = 0; i < 6; ++i) {
    if (have_lookup_models_ && i < lookup_models_.size()) {
      const auto kin = lookup_solver_.evaluate(lookup_models_[i], phases[i], phase_rates[i], horizon_dt_s_);
      predicted.foot_xy[i] = kin.foot_body_xy;
      predicted.foot_vel_xy[i] = kin.foot_velocity_body_xy;
      predicted.foot_z[i] = kin.foot_body_z;
      predicted.stance[i] = kin.stance_confidence;
    } else {
      predicted.foot_xy[i] = Eigen::Vector2d(body_state.foot_position_body[i].x, body_state.foot_position_body[i].y);
      predicted.foot_vel_xy[i] = Eigen::Vector2d(body_state.foot_velocity_body[i].x, body_state.foot_velocity_body[i].y);
      predicted.foot_z[i] = body_state.foot_position_body[i].z;
      predicted.stance[i] = body_state.stance_confidence[i];
    }
  }
  return predicted;
}

KlannControllerCore::BodyTwistEstimate KlannControllerCore::estimate_body_twist(const PredictedLegState & predicted) const
{
  BodyTwistEstimate out;
  Eigen::Matrix3d h = 1.0e-6 * Eigen::Matrix3d::Identity();
  Eigen::Vector3d g = Eigen::Vector3d::Zero();
  int used = 0;

  for (std::size_t i = 0; i < 6; ++i) {
    const double w = clamp(predicted.stance[i], 0.0, 1.0);
    if (w < 0.2) {
      continue;
    }
    const double px = predicted.foot_xy[i].x();
    const double py = predicted.foot_xy[i].y();
    const double vfx = predicted.foot_vel_xy[i].x();
    const double vfy = predicted.foot_vel_xy[i].y();
    Eigen::Matrix<double, 2, 3> a;
    a << 1.0, 0.0, -py,
         0.0, 1.0,  px;
    Eigen::Vector2d b(-vfx, -vfy);
    h += w * a.transpose() * a;
    g += w * a.transpose() * b;
    ++used;
  }

  if (used < 2) {
    return out;
  }

  const Eigen::Vector3d xi = h.ldlt().solve(g);
  out.vx = clamp(xi.x(), -linear_speed_limit_mps_, linear_speed_limit_mps_);
  out.vy = xi.y();
  out.wz = clamp(xi.z(), -angular_speed_limit_rps_, angular_speed_limit_rps_);
  out.valid = std::isfinite(out.vx) && std::isfinite(out.vy) && std::isfinite(out.wz);

  if (!out.valid) {
    return out;
  }

  double residual = 0.0;
  for (std::size_t i = 0; i < 6; ++i) {
    const double w = clamp(predicted.stance[i], 0.0, 1.0);
    if (w < 0.2) {
      continue;
    }
    const double px = predicted.foot_xy[i].x();
    const double py = predicted.foot_xy[i].y();
    const double vfx = predicted.foot_vel_xy[i].x();
    const double vfy = predicted.foot_vel_xy[i].y();
    const double residual_x = out.vx - out.wz * py + vfx;
    const double residual_y = out.vy + out.wz * px + vfy;
    residual += w * (residual_x * residual_x + residual_y * residual_y);
  }
  out.slip_residual = residual;
  return out;
}

std::vector<KlannControllerCore::PhaseRateStep> KlannControllerCore::build_base_phase_sequence(
  const hexapod_control_interfaces::msg::KlannBodyState & body_state,
  uint8_t * first_gait_mode)
{
  std::vector<PhaseRateStep> base(static_cast<std::size_t>(horizon_steps_));
  std::array<double, 6> phases{};
  for (std::size_t i = 0; i < 6; ++i) {
    phases[i] = body_state.phase_rad[i];
  }

  uint8_t first_mode = 255U;
  for (int k = 0; k < horizon_steps_; ++k) {
    const auto gait_cmd = gait_.make_command(
      phases,
      nominal_vx_sequence_[static_cast<std::size_t>(k)],
      nominal_wz_sequence_[static_cast<std::size_t>(k)],
      nominal_phase_rate_rad_s_,
      horizon_dt_s_);
    if (k == 0) {
      first_mode = gait_cmd.gait_mode;
    }
    for (std::size_t i = 0; i < 6; ++i) {
      base[static_cast<std::size_t>(k)][i] = clamp(gait_cmd.phase_velocity_rad_s[i], -phase_rate_limit_rad_s_, phase_rate_limit_rad_s_);
      phases[i] = PhaseGaitGenerator::wrap_2pi(phases[i] + base[static_cast<std::size_t>(k)][i] * horizon_dt_s_);
    }
  }
  if (first_gait_mode != nullptr) {
    *first_gait_mode = first_mode;
  }
  return base;
}

KlannControllerCore::SequenceSample KlannControllerCore::rollout_sequence(
  const nav_msgs::msg::Path & plan,
  const hexapod_control_interfaces::msg::KlannBodyState & body_state,
  const std::vector<PhaseRateStep> & phase_rate_seq,
  const std::vector<PhaseRateStep> & base_phase_rate_seq,
  uint8_t first_gait_mode)
{
  SequenceSample sample;
  sample.phase_rates = phase_rate_seq;
  sample.valid = true;
  sample.first_gait_mode = first_gait_mode;

  std::array<double, 6> phases{};
  for (std::size_t i = 0; i < 6; ++i) {
    phases[i] = body_state.phase_rad[i];
  }

  const double preview_s = std::max(0.0f, body_state.estimated_total_preview_s);
  double x = body_state.pose.position.x +
    (std::cos(body_state.yaw_rad) * body_state.twist.linear.x - std::sin(body_state.yaw_rad) * body_state.twist.linear.y) * preview_s;
  double y = body_state.pose.position.y +
    (std::sin(body_state.yaw_rad) * body_state.twist.linear.x + std::cos(body_state.yaw_rad) * body_state.twist.linear.y) * preview_s;
  double yaw = body_state.yaw_rad + body_state.twist.angular.z * preview_s;

  PhaseRateStep prev_rates{};
  for (std::size_t i = 0; i < 6; ++i) {
    prev_rates[i] = body_state.phase_velocity_rad_s[i];
  }

  double total_cost = 0.0;
  bool first_twist_set = false;

  for (int k = 0; k < horizon_steps_; ++k) {
    std::array<double, 6> phase_rates{};
    for (std::size_t i = 0; i < 6; ++i) {
      phase_rates[i] = clamp(phase_rate_seq[static_cast<std::size_t>(k)][i], -phase_rate_limit_rad_s_, phase_rate_limit_rad_s_);
    }

    const auto predicted = predict_leg_state(phases, phase_rates, body_state);
    const auto twist = estimate_body_twist(predicted);
    if (!twist.valid) {
      total_cost += 1.0e3;
      continue;
    }

    if (!first_twist_set) {
      sample.first_vx = twist.vx;
      sample.first_vy = twist.vy;
      sample.first_wz = twist.wz;
      first_twist_set = true;
    }

    x += (std::cos(yaw) * twist.vx - std::sin(yaw) * twist.vy) * horizon_dt_s_;
    y += (std::sin(yaw) * twist.vx + std::cos(yaw) * twist.vy) * horizon_dt_s_;
    yaw = PhaseGaitGenerator::wrap_pi(yaw + twist.wz * horizon_dt_s_);

    double heading_target = 0.0;
    const double path_err = nearest_path_error(plan, x, y, heading_target);
    total_cost += path_weight_ * path_err * path_err;
    total_cost += heading_weight_ * std::pow(wrap_pi(yaw - heading_target), 2.0);
    total_cost -= progress_weight_ * twist.vx;
    total_cost += lateral_velocity_weight_ * twist.vy * twist.vy;
    total_cost += yaw_rate_tracking_weight_ * std::pow(twist.wz - static_cast<double>(body_state.imu_yaw_rate_rad_s), 2.0);

    int stance_count = 0;
    for (double s : predicted.stance) {
      if (s > 0.5) {
        ++stance_count;
      }
    }
    if (stance_count < 3) {
      total_cost += support_weight_ * static_cast<double>((3 - stance_count) * (3 - stance_count));
    }

    const auto support_plane = estimate_support_plane(predicted);
    const double margin = support_plane.valid ? support_plane.margin : support_margin_from_points(predicted.foot_xy, predicted.stance);
    if (margin < kSupportMarginTargetM) {
      total_cost += support_margin_weight_ * std::pow(kSupportMarginTargetM - margin, 2.0);
    }
    if (support_plane.valid) {
      total_cost += roll_weight_ * support_plane.roll_ref_rad * support_plane.roll_ref_rad;
      total_cost += pitch_weight_ * support_plane.pitch_ref_rad * support_plane.pitch_ref_rad;
      total_cost += roll_tracking_weight_ * std::pow(support_plane.roll_ref_rad - static_cast<double>(body_state.roll_rad), 2.0);
      total_cost += pitch_tracking_weight_ * std::pow(support_plane.pitch_ref_rad - static_cast<double>(body_state.pitch_rad), 2.0);
    } else {
      total_cost += roll_tracking_weight_ * std::pow(static_cast<double>(body_state.roll_rad), 2.0);
      total_cost += pitch_tracking_weight_ * std::pow(static_cast<double>(body_state.pitch_rad), 2.0);
    }

    total_cost += slip_weight_ * twist.slip_residual;

    for (std::size_t i = 0; i < 6; ++i) {
      const double base_rate = base_phase_rate_seq[static_cast<std::size_t>(k)][i];
      total_cost += phase_sync_weight_ * std::pow(phase_rates[i] - base_rate, 2.0);
      total_cost += control_smooth_weight_ * std::pow(phase_rates[i] - prev_rates[i], 2.0);
      prev_rates[i] = phase_rates[i];
      phases[i] = PhaseGaitGenerator::wrap_2pi(phases[i] + phase_rates[i] * horizon_dt_s_);
    }
  }

  sample.cost = total_cost;
  return sample;
}

ControllerResult KlannControllerCore::idle_result(
  const hexapod_control_interfaces::msg::KlannBodyState & body_state,
  const std::array<uint8_t, 6> & motor_ids) const
{
  ControllerResult result;
  result.cmd_vel.header.stamp = body_state.header.stamp;
  result.cmd_vel.header.frame_id = "base_link";
  result.cmd_vel.twist.linear.x = 0.0;
  result.cmd_vel.twist.linear.y = 0.0;
  result.cmd_vel.twist.linear.z = 0.0;
  result.cmd_vel.twist.angular.x = 0.0;
  result.cmd_vel.twist.angular.y = 0.0;
  result.cmd_vel.twist.angular.z = 0.0;

  result.phase_cmd.header = body_state.header;
  result.phase_cmd.motor_id = motor_ids;
  result.phase_cmd.use_phase_targets = true;
  result.phase_cmd.use_phase_velocity = true;
  result.phase_cmd.use_motor_rpm_ff = true;
  result.phase_cmd.gait_mode = 255U;
  for (std::size_t i = 0; i < 6; ++i) {
    result.phase_cmd.phase_target_rad[i] = body_state.phase_rad[i];
    result.phase_cmd.phase_velocity_rad_s[i] = 0.0f;
    result.phase_cmd.motor_rpm_ff[i] = 0.0f;
  }
  return result;
}

ControllerResult KlannControllerCore::compute(
  const nav_msgs::msg::Path & plan,
  const hexapod_control_interfaces::msg::KlannBodyState & body_state,
  const std::array<uint8_t, 6> & motor_ids)
{
  ensure_nominal_sequences();
  if (plan.poses.empty()) {
    std::fill(nominal_vx_sequence_.begin(), nominal_vx_sequence_.end(), 0.0);
    std::fill(nominal_wz_sequence_.begin(), nominal_wz_sequence_.end(), 0.0);
    for (auto & step : nominal_phase_rate_sequence_) {
      step.fill(0.0);
    }
    return idle_result(body_state, motor_ids);
  }

  double desired_heading = body_state.yaw_rad;
  const double path_err = nearest_path_error(plan, body_state.pose.position.x, body_state.pose.position.y, desired_heading);
  const double nominal_vx = clamp(
    (plan.poses.empty() ? 0.0 : linear_speed_limit_mps_ * std::exp(-2.0 * path_err)),
    -linear_speed_limit_mps_, linear_speed_limit_mps_);
  const double nominal_wz = clamp(1.5 * wrap_pi(desired_heading - body_state.yaw_rad), -angular_speed_limit_rps_, angular_speed_limit_rps_);
  warm_start_sequences(nominal_vx, nominal_wz);

  uint8_t base_first_gait_mode = 255U;
  const auto base_phase_rate_seq = build_base_phase_sequence(body_state, &base_first_gait_mode);
  for (int k = 0; k < horizon_steps_; ++k) {
    for (std::size_t i = 0; i < 6; ++i) {
      nominal_phase_rate_sequence_[static_cast<std::size_t>(k)][i] =
        0.35 * nominal_phase_rate_sequence_[static_cast<std::size_t>(k)][i] +
        0.65 * base_phase_rate_seq[static_cast<std::size_t>(k)][i];
    }
  }

  std::mt19937 rng(42U);
  std::normal_distribution<double> common_noise(0.0, vx_noise_std_);
  std::normal_distribution<double> side_noise(0.0, wz_noise_std_);
  std::normal_distribution<double> leg_noise(0.0, phase_rate_noise_std_);

  std::vector<SequenceSample> samples;
  samples.reserve(static_cast<std::size_t>(sample_count_ + 1));
  samples.push_back(rollout_sequence(plan, body_state, nominal_phase_rate_sequence_, base_phase_rate_seq, base_first_gait_mode));

  double min_cost = samples.front().cost;
  const auto side_sign = side_signs();
  for (int s = 0; s < sample_count_; ++s) {
    std::vector<PhaseRateStep> phase_seq = nominal_phase_rate_sequence_;
    double corr_common = 0.0;
    double corr_turn = 0.0;
    PhaseRateStep corr_leg{};
    for (int k = 0; k < horizon_steps_; ++k) {
      corr_common = 0.75 * corr_common + common_noise(rng);
      corr_turn = 0.75 * corr_turn + side_noise(rng);
      for (std::size_t i = 0; i < 6; ++i) {
        corr_leg[i] = 0.65 * corr_leg[i] + leg_noise(rng);
        const double perturb = corr_common + corr_turn * static_cast<double>(side_sign[i]) + corr_leg[i];
        phase_seq[static_cast<std::size_t>(k)][i] = clamp(
          phase_seq[static_cast<std::size_t>(k)][i] + perturb,
          -phase_rate_limit_rad_s_, phase_rate_limit_rad_s_);
      }
    }
    samples.push_back(rollout_sequence(plan, body_state, phase_seq, base_phase_rate_seq, base_first_gait_mode));
    min_cost = std::min(min_cost, samples.back().cost);
  }

  double weight_sum = 0.0;
  std::vector<PhaseRateStep> updated_phase_seq(static_cast<std::size_t>(horizon_steps_));
  for (const auto & sample : samples) {
    const double weight = std::exp(-(sample.cost - min_cost) / temperature_);
    weight_sum += weight;
    for (int k = 0; k < horizon_steps_; ++k) {
      for (std::size_t i = 0; i < 6; ++i) {
        updated_phase_seq[static_cast<std::size_t>(k)][i] += weight * sample.phase_rates[static_cast<std::size_t>(k)][i];
      }
    }
  }
  if (weight_sum > 1.0e-9) {
    for (int k = 0; k < horizon_steps_; ++k) {
      for (std::size_t i = 0; i < 6; ++i) {
        updated_phase_seq[static_cast<std::size_t>(k)][i] /= weight_sum;
      }
    }
    nominal_phase_rate_sequence_ = updated_phase_seq;
  }

  const auto best = rollout_sequence(plan, body_state, nominal_phase_rate_sequence_, base_phase_rate_seq, base_first_gait_mode);

  ControllerResult result;
  result.cmd_vel.header.stamp = body_state.header.stamp;
  result.cmd_vel.header.frame_id = "base_link";
  result.cmd_vel.twist.linear.x = best.first_vx;
  result.cmd_vel.twist.linear.y = best.first_vy;
  result.cmd_vel.twist.angular.z = best.first_wz;

  result.phase_cmd.header = body_state.header;
  result.phase_cmd.motor_id = motor_ids;
  result.phase_cmd.use_phase_targets = true;
  result.phase_cmd.use_phase_velocity = true;
  result.phase_cmd.use_motor_rpm_ff = false;
  result.phase_cmd.gait_mode = best.first_gait_mode;
  for (std::size_t i = 0; i < 6; ++i) {
    const double phase_rate = clamp(best.phase_rates.front()[i], -phase_rate_limit_rad_s_, phase_rate_limit_rad_s_);
    result.phase_cmd.phase_target_rad[i] = static_cast<float>(PhaseGaitGenerator::wrap_2pi(body_state.phase_rad[i] + phase_rate * horizon_dt_s_));
    result.phase_cmd.phase_velocity_rad_s[i] = static_cast<float>(phase_rate);
    result.phase_cmd.motor_rpm_ff[i] = 0.0f;
  }

  return result;
}

}  // namespace hexapod_nav_cpp
