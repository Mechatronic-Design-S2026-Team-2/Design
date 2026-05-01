#ifndef HEXAPOD_NAV_CPP__KLANN_CONTROLLER_CORE_HPP_
#define HEXAPOD_NAV_CPP__KLANN_CONTROLLER_CORE_HPP_

#include <Eigen/Dense>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <hexapod_control_interfaces/msg/klann_body_state.hpp>
#include <hexapod_control_interfaces/msg/leg_phase_command.hpp>
#include <nav_msgs/msg/path.hpp>

#include "hexapod_hardware_cpp/klann_geometry.hpp"
#include "hexapod_nav_cpp/phase_gait_generator.hpp"

#include <array>
#include <string>
#include <vector>

namespace hexapod_nav_cpp
{

struct ControllerResult
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  hexapod_control_interfaces::msg::LegPhaseCommand phase_cmd;
};

class KlannControllerCore
{
public:
  KlannControllerCore();

  void set_parameters(
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
    double in_place_turn_threshold_rps);

  bool initialize_lookup_models(const std::string & linkage_yaml_path);

  ControllerResult compute(
    const nav_msgs::msg::Path & plan,
    const hexapod_control_interfaces::msg::KlannBodyState & body_state,
    const std::array<uint8_t, 6> & motor_ids);

private:
  using PhaseRateStep = std::array<double, 6>;

  struct SequenceSample
  {
    std::vector<PhaseRateStep> phase_rates;
    double cost{0.0};
    bool valid{false};
    double first_vx{0.0};
    double first_vy{0.0};
    double first_wz{0.0};
    uint8_t first_gait_mode{255U};
  };

  struct PredictedLegState
  {
    std::array<Eigen::Vector2d, 6> foot_xy{};
    std::array<Eigen::Vector2d, 6> foot_vel_xy{};
    std::array<double, 6> foot_z{};
    std::array<double, 6> stance{};
  };

  struct SupportPlaneEstimate
  {
    double margin{0.0};
    double roll_ref_rad{0.0};
    double pitch_ref_rad{0.0};
    bool valid{false};
  };

  struct BodyTwistEstimate
  {
    double vx{0.0};
    double vy{0.0};
    double wz{0.0};
    double slip_residual{0.0};
    bool valid{false};
  };

  double horizon_dt_s_{0.05};
  int horizon_steps_{15};
  int sample_count_{128};
  double nominal_phase_rate_rad_s_{2.0};
  double linear_speed_limit_mps_{0.25};
  double angular_speed_limit_rps_{0.75};
  double temperature_{0.5};
  double vx_noise_std_{0.05};
  double wz_noise_std_{0.15};
  double phase_rate_noise_std_{0.8};
  double path_weight_{8.0};
  double heading_weight_{3.0};
  double progress_weight_{1.0};
  double support_weight_{25.0};
  double support_margin_weight_{4.0};
  double slip_weight_{2.0};
  double phase_sync_weight_{1.5};
  double control_smooth_weight_{0.3};
  double lateral_velocity_weight_{2.0};
  double yaw_rate_tracking_weight_{1.0};
  double roll_weight_{1.5};
  double pitch_weight_{1.5};
  double roll_tracking_weight_{0.5};
  double pitch_tracking_weight_{0.5};
  double phase_rate_limit_rad_s_{8.0};

  std::vector<double> nominal_vx_sequence_;
  std::vector<double> nominal_wz_sequence_;
  std::vector<PhaseRateStep> nominal_phase_rate_sequence_;

  PhaseGaitGenerator gait_;
  hexapod_hardware_cpp::LinkageSolver lookup_solver_;
  std::vector<hexapod_hardware_cpp::LegModel> lookup_models_;
  bool have_lookup_models_{false};
  std::string linkage_yaml_path_;

  static double clamp(double value, double lo, double hi);
  static double wrap_pi(double angle_rad);
  static double nearest_path_error(const nav_msgs::msg::Path & plan, double x, double y, double & heading_target);
  static double support_margin_from_points(const std::array<Eigen::Vector2d, 6> & feet, const std::array<double, 6> & stance);
  static SupportPlaneEstimate estimate_support_plane(const PredictedLegState & predicted);
  static std::array<int, 6> side_signs();

  ControllerResult idle_result(
    const hexapod_control_interfaces::msg::KlannBodyState & body_state,
    const std::array<uint8_t, 6> & motor_ids) const;
  void ensure_nominal_sequences();
  void warm_start_sequences(double nominal_vx, double nominal_wz);
  PredictedLegState predict_leg_state(
    const std::array<double, 6> & phases,
    const std::array<double, 6> & phase_rates,
    const hexapod_control_interfaces::msg::KlannBodyState & body_state);
  BodyTwistEstimate estimate_body_twist(const PredictedLegState & predicted) const;
  std::vector<PhaseRateStep> build_base_phase_sequence(
    const hexapod_control_interfaces::msg::KlannBodyState & body_state,
    uint8_t * first_gait_mode);
  SequenceSample rollout_sequence(
    const nav_msgs::msg::Path & plan,
    const hexapod_control_interfaces::msg::KlannBodyState & body_state,
    const std::vector<PhaseRateStep> & phase_rate_seq,
    const std::vector<PhaseRateStep> & base_phase_rate_seq,
    uint8_t first_gait_mode);
};

}  // namespace hexapod_nav_cpp

#endif  // HEXAPOD_NAV_CPP__KLANN_CONTROLLER_CORE_HPP_
