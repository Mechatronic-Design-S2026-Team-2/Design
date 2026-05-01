#ifndef HEXAPOD_NAV_CPP__PHASE_GAIT_GENERATOR_HPP_
#define HEXAPOD_NAV_CPP__PHASE_GAIT_GENERATOR_HPP_

#include <array>
#include <cstdint>

namespace hexapod_nav_cpp
{

struct GaitCommand
{
  std::array<double, 6> phase_velocity_rad_s{};
  std::array<double, 6> phase_target_rad{};
  std::array<double, 6> desired_phase_offset_rad{};
  std::array<double, 6> stance_confidence{};
  uint8_t gait_mode{0};
};

class PhaseGaitGenerator
{
public:
  PhaseGaitGenerator();

  void set_parameters(
    double phase_sync_gain,
    double turn_phase_bias_gain,
    double minimum_speed_scale,
    double wave_turn_threshold_rps,
    double in_place_turn_threshold_rps);

  GaitCommand make_command(
    const std::array<double, 6> & current_phase_rad,
    double desired_vx_mps,
    double desired_wz_rps,
    double nominal_phase_rate_rad_s,
    double dt_s) const;

  static double wrap_2pi(double angle_rad);
  static double wrap_pi(double angle_rad);
  static double smooth_stance_confidence(double phase_rad);

private:
  std::array<double, 6> offsets_for_mode(uint8_t gait_mode) const;

  double phase_sync_gain_{1.8};
  double turn_phase_bias_gain_{0.8};
  double minimum_speed_scale_{0.2};
  double wave_turn_threshold_rps_{0.35};
  double in_place_turn_threshold_rps_{0.10};
};

}  // namespace hexapod_nav_cpp

#endif  // HEXAPOD_NAV_CPP__PHASE_GAIT_GENERATOR_HPP_
