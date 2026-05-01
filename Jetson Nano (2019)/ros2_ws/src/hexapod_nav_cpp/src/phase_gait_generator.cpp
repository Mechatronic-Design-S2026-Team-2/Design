#include "hexapod_nav_cpp/phase_gait_generator.hpp"

#include <algorithm>
#include <cmath>

namespace hexapod_nav_cpp
{

PhaseGaitGenerator::PhaseGaitGenerator() = default;

void PhaseGaitGenerator::set_parameters(
  double phase_sync_gain,
  double turn_phase_bias_gain,
  double minimum_speed_scale,
  double wave_turn_threshold_rps,
  double in_place_turn_threshold_rps)
{
  phase_sync_gain_ = phase_sync_gain;
  turn_phase_bias_gain_ = turn_phase_bias_gain;
  minimum_speed_scale_ = minimum_speed_scale;
  wave_turn_threshold_rps_ = wave_turn_threshold_rps;
  in_place_turn_threshold_rps_ = in_place_turn_threshold_rps;
}

double PhaseGaitGenerator::wrap_2pi(double angle_rad)
{
  while (angle_rad < 0.0) angle_rad += 2.0 * M_PI;
  while (angle_rad >= 2.0 * M_PI) angle_rad -= 2.0 * M_PI;
  return angle_rad;
}

double PhaseGaitGenerator::wrap_pi(double angle_rad)
{
  while (angle_rad > M_PI) angle_rad -= 2.0 * M_PI;
  while (angle_rad < -M_PI) angle_rad += 2.0 * M_PI;
  return angle_rad;
}

double PhaseGaitGenerator::smooth_stance_confidence(double phase_rad)
{
  const double p = wrap_2pi(phase_rad);
  const double center = M_PI_2;
  const double half_width = 0.95 * M_PI_2;
  const double err = std::abs(wrap_pi(p - center));
  if (err >= half_width) {
    return 0.0;
  }
  const double u = 1.0 - err / half_width;
  return u * u * (3.0 - 2.0 * u);
}

std::array<double, 6> PhaseGaitGenerator::offsets_for_mode(uint8_t gait_mode) const
{
  if (gait_mode == 2U) {
    return {0.0, M_PI / 3.0, 2.0 * M_PI / 3.0, M_PI, 4.0 * M_PI / 3.0, 5.0 * M_PI / 3.0};
  }
  return {0.0, M_PI, 0.0, M_PI, 0.0, M_PI};
}

GaitCommand PhaseGaitGenerator::make_command(
  const std::array<double, 6> & current_phase_rad,
  double desired_vx_mps,
  double desired_wz_rps,
  double nominal_phase_rate_rad_s,
  double dt_s) const
{
  GaitCommand out;
  const bool turning_in_place = std::abs(desired_vx_mps) < 0.03 && std::abs(desired_wz_rps) > in_place_turn_threshold_rps_;
  const bool wave_turn = std::abs(desired_wz_rps) > wave_turn_threshold_rps_;
  out.gait_mode = turning_in_place ? 1U : (wave_turn ? 2U : 0U);

  const auto offsets = offsets_for_mode(out.gait_mode);
  out.desired_phase_offset_rad = offsets;
  const std::array<int, 6> side_sign = {-1, -1, -1, +1, +1, +1};
  const double speed_scale = std::max(minimum_speed_scale_, std::min(1.0, std::abs(desired_vx_mps) + 0.6 * std::abs(desired_wz_rps)));
  const double direction = desired_vx_mps < 0.0 ? -1.0 : 1.0;

  double phase_anchor = 0.0;
  for (std::size_t i = 0; i < 6; ++i) {
    phase_anchor += wrap_pi(current_phase_rad[i] - offsets[i]);
  }
  phase_anchor /= 6.0;

  for (std::size_t i = 0; i < 6; ++i) {
    const double desired_phase = wrap_2pi(phase_anchor + offsets[i]);
    double phase_rate = nominal_phase_rate_rad_s * speed_scale * direction;
    if (turning_in_place) {
      phase_rate = nominal_phase_rate_rad_s * static_cast<double>(side_sign[i]) * (desired_wz_rps >= 0.0 ? 1.0 : -1.0);
    } else {
      phase_rate += turn_phase_bias_gain_ * desired_wz_rps * static_cast<double>(side_sign[i]);
    }
    const double sync_error = wrap_pi(desired_phase - current_phase_rad[i]);
    phase_rate += phase_sync_gain_ * sync_error;
    out.phase_velocity_rad_s[i] = phase_rate;
    out.phase_target_rad[i] = wrap_2pi(current_phase_rad[i] + phase_rate * dt_s);
    out.stance_confidence[i] = smooth_stance_confidence(out.phase_target_rad[i]);
  }

  return out;
}

}  // namespace hexapod_nav_cpp
