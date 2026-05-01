#ifndef HEXAPOD_HARDWARE_CPP__KLANN_GEOMETRY_HPP_
#define HEXAPOD_HARDWARE_CPP__KLANN_GEOMETRY_HPP_

#include <Eigen/Dense>

#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace hexapod_hardware_cpp
{

struct DistanceConstraint
{
  std::string a;
  std::string b;
  double length{0.0};
};

struct AngleConstraint
{
  std::string a;
  std::string vertex;
  std::string c;
  double angle_rad{0.0};
};

struct LinkageInput
{
  std::string shaft_point;
  std::string crank_tip_name;
  double crank_radius{0.0};
  double phase_offset_rad{0.0};
};

struct LegMount
{
  double shaft_x_m{0.0};
  double shaft_y_m{0.0};
  double shaft_z_m{0.0};
  double yaw_rad{0.0};
};

struct LookupSample
{
  double phase_rad{0.0};
  double local_foot_x_m{0.0};
  double local_foot_z_m{0.0};
  double dfoot_x_dphase{0.0};
  double dfoot_z_dphase{0.0};
  double stance_confidence{0.0};
};

struct LegModel
{
  std::string name;
  LegMount mount;
  std::map<std::string, Eigen::Vector2d> fixed_points;
  std::vector<std::string> unknown_points;
  std::vector<DistanceConstraint> distance_constraints;
  std::vector<AngleConstraint> angle_constraints;
  std::unordered_map<std::string, Eigen::Vector2d> initial_guess;
  LinkageInput input;
  std::string foot_point_name{"foot"};
  double stance_phase_min_rad{0.0};
  double stance_phase_max_rad{3.14159};
  bool use_lookup_table{true};
  int lookup_resolution{1024};
  std::string lookup_file_path;
  bool lookup_generate_if_missing{true};
  bool lookup_save_generated{false};
  bool lookup_loaded_from_file{false};
  std::vector<LookupSample> lookup_table;
  double lookup_min_local_z_m{0.0};
  double lookup_max_local_z_m{0.0};
};

struct LegKinematicState
{
  Eigen::Vector2d foot_body_xy{0.0, 0.0};
  Eigen::Vector2d foot_velocity_body_xy{0.0, 0.0};
  double foot_body_z{0.0};
  double foot_velocity_body_z{0.0};
  double stance_confidence{0.0};
  bool valid{false};
};

class LinkageSolver
{
public:
  static std::vector<LegModel> load_models_from_yaml(const std::string & yaml_path);
  static bool load_lookup_table_binary(LegModel & model, const std::string & file_path);
  static bool save_lookup_table_binary(const LegModel & model, const std::string & file_path);
  static std::string resolve_path(const std::string & base_path, const std::string & maybe_relative);

  LegKinematicState evaluate(
    LegModel & model,
    double phase_rad,
    double phase_velocity_rad_s,
    double dt_hint_s);

  bool solve_points(
    LegModel & model,
    double phase_rad,
    std::unordered_map<std::string, Eigen::Vector2d> & points);

  void generate_lookup_table(LegModel & model);

  static double wrap_to_pi(double angle_rad);
  static double wrap_to_2pi(double angle_rad);

private:
  bool solve_unknown_positions(
    const LegModel & model,
    double phase_rad,
    std::unordered_map<std::string, Eigen::Vector2d> & points);
};

}  // namespace hexapod_hardware_cpp

#endif  // HEXAPOD_HARDWARE_CPP__KLANN_GEOMETRY_HPP_
