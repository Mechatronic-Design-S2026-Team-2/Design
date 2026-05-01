#include "hexapod_hardware_cpp/klann_geometry.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>

namespace hexapod_hardware_cpp
{

namespace
{
constexpr double kTwoPi = 2.0 * M_PI;
constexpr uint32_t kLookupBinaryVersion = 1U;
constexpr char kLookupMagic[8] = {'K', 'L', 'U', 'T', '1', '7', 'B', 'N'};

struct LookupBinaryHeader
{
  char magic[8];
  uint32_t version;
  uint32_t resolution;
  float min_local_z_m;
  float max_local_z_m;
};

struct LookupBinarySample
{
  float local_foot_x_m;
  float local_foot_z_m;
  float dfoot_x_dphase;
  float dfoot_z_dphase;
  float stance_confidence;
};

Eigen::Vector2d rotate2d(const Eigen::Vector2d & v, double yaw_rad)
{
  const double c = std::cos(yaw_rad);
  const double s = std::sin(yaw_rad);
  return Eigen::Vector2d(c * v.x() - s * v.y(), s * v.x() + c * v.y());
}

double angle_of(const Eigen::Vector2d & v)
{
  return std::atan2(v.y(), v.x());
}

double unwrap_phase_window_confidence(double phase_rad, double min_rad, double max_rad)
{
  auto wrap = [](double a) {
      while (a < 0.0) a += kTwoPi;
      while (a >= kTwoPi) a -= kTwoPi;
      return a;
    };
  const double p = wrap(phase_rad);
  const double a = wrap(min_rad);
  const double b = wrap(max_rad);

  bool inside = false;
  if (a <= b) {
    inside = (p >= a && p <= b);
  } else {
    inside = (p >= a || p <= b);
  }
  return inside ? 1.0 : 0.0;
}

struct InterpolatedLookup
{
  double local_x{0.0};
  double local_z{0.0};
  double dlocal_x_dphase{0.0};
  double dlocal_z_dphase{0.0};
  double stance_confidence{0.0};
  bool valid{false};
};

InterpolatedLookup interpolate_lookup(const LegModel & model, double phase_rad)
{
  InterpolatedLookup out;
  if (model.lookup_table.empty()) {
    return out;
  }

  double wrapped = phase_rad;
  while (wrapped < 0.0) wrapped += kTwoPi;
  while (wrapped >= kTwoPi) wrapped -= kTwoPi;
  const double scaled = wrapped / kTwoPi * static_cast<double>(model.lookup_resolution);
  int idx0 = static_cast<int>(std::floor(scaled));
  const double frac = scaled - static_cast<double>(idx0);
  while (idx0 < 0) idx0 += model.lookup_resolution;
  idx0 %= model.lookup_resolution;
  const int idx1 = (idx0 + 1) % model.lookup_resolution;

  const auto & a = model.lookup_table[static_cast<std::size_t>(idx0)];
  const auto & b = model.lookup_table[static_cast<std::size_t>(idx1)];
  auto lerp = [frac](double x, double y) {
      return x + frac * (y - x);
    };

  out.local_x = lerp(a.local_foot_x_m, b.local_foot_x_m);
  out.local_z = lerp(a.local_foot_z_m, b.local_foot_z_m);
  out.dlocal_x_dphase = lerp(a.dfoot_x_dphase, b.dfoot_x_dphase);
  out.dlocal_z_dphase = lerp(a.dfoot_z_dphase, b.dfoot_z_dphase);
  out.stance_confidence = lerp(a.stance_confidence, b.stance_confidence);
  out.valid = true;
  return out;
}

bool file_exists(const std::string & path)
{
  return !path.empty() && std::ifstream(path, std::ios::binary).good();
}

std::string expand_user_path(const std::string & path)
{
  if (path.empty() || path[0] != '~') {
    return path;
  }
  const char * home = std::getenv("HOME");
  if (home == nullptr || std::string(home).empty()) {
    return path;
  }
  if (path.size() == 1U) {
    return std::string(home);
  }
  if (path.size() >= 2U && path[1] == '/') {
    return std::string(home) + path.substr(1);
  }
  return path;
}
}  // namespace

double LinkageSolver::wrap_to_pi(double angle_rad)
{
  while (angle_rad > M_PI) {
    angle_rad -= kTwoPi;
  }
  while (angle_rad < -M_PI) {
    angle_rad += kTwoPi;
  }
  return angle_rad;
}

double LinkageSolver::wrap_to_2pi(double angle_rad)
{
  while (angle_rad < 0.0) {
    angle_rad += kTwoPi;
  }
  while (angle_rad >= kTwoPi) {
    angle_rad -= kTwoPi;
  }
  return angle_rad;
}

std::string LinkageSolver::resolve_path(const std::string & base_path, const std::string & maybe_relative)
{
  const std::string expanded = expand_user_path(maybe_relative);
  if (expanded.empty()) {
    return expanded;
  }
  std::filesystem::path p(expanded);
  if (p.is_absolute()) {
    return p.lexically_normal().string();
  }
  const std::filesystem::path base(expand_user_path(base_path));
  const auto combined = base.parent_path() / p;
  return combined.lexically_normal().string();
}

bool LinkageSolver::load_lookup_table_binary(LegModel & model, const std::string & file_path)
{
  std::ifstream in(file_path, std::ios::binary);
  if (!in.good()) {
    return false;
  }

  LookupBinaryHeader header{};
  in.read(reinterpret_cast<char *>(&header), sizeof(header));
  if (!in.good()) {
    return false;
  }
  if (std::memcmp(header.magic, kLookupMagic, sizeof(kLookupMagic)) != 0 ||
    header.version != kLookupBinaryVersion ||
    header.resolution == 0U)
  {
    return false;
  }

  model.lookup_resolution = static_cast<int>(header.resolution);
  model.lookup_min_local_z_m = static_cast<double>(header.min_local_z_m);
  model.lookup_max_local_z_m = static_cast<double>(header.max_local_z_m);
  model.lookup_table.clear();
  model.lookup_table.resize(static_cast<std::size_t>(header.resolution));

  for (std::size_t i = 0; i < model.lookup_table.size(); ++i) {
    LookupBinarySample sample{};
    in.read(reinterpret_cast<char *>(&sample), sizeof(sample));
    if (!in.good()) {
      model.lookup_table.clear();
      return false;
    }
    model.lookup_table[i].phase_rad = kTwoPi * static_cast<double>(i) / static_cast<double>(header.resolution);
    model.lookup_table[i].local_foot_x_m = static_cast<double>(sample.local_foot_x_m);
    model.lookup_table[i].local_foot_z_m = static_cast<double>(sample.local_foot_z_m);
    model.lookup_table[i].dfoot_x_dphase = static_cast<double>(sample.dfoot_x_dphase);
    model.lookup_table[i].dfoot_z_dphase = static_cast<double>(sample.dfoot_z_dphase);
    model.lookup_table[i].stance_confidence = static_cast<double>(sample.stance_confidence);
  }
  model.lookup_loaded_from_file = true;
  return true;
}

bool LinkageSolver::save_lookup_table_binary(const LegModel & model, const std::string & file_path)
{
  if (model.lookup_table.empty() || file_path.empty()) {
    return false;
  }

  std::filesystem::path path(file_path);
  std::filesystem::create_directories(path.parent_path());

  std::ofstream out(file_path, std::ios::binary | std::ios::trunc);
  if (!out.good()) {
    return false;
  }

  LookupBinaryHeader header{};
  std::memcpy(header.magic, kLookupMagic, sizeof(kLookupMagic));
  header.version = kLookupBinaryVersion;
  header.resolution = static_cast<uint32_t>(model.lookup_resolution);
  header.min_local_z_m = static_cast<float>(model.lookup_min_local_z_m);
  header.max_local_z_m = static_cast<float>(model.lookup_max_local_z_m);
  out.write(reinterpret_cast<const char *>(&header), sizeof(header));

  for (const auto & sample : model.lookup_table) {
    LookupBinarySample packed{};
    packed.local_foot_x_m = static_cast<float>(sample.local_foot_x_m);
    packed.local_foot_z_m = static_cast<float>(sample.local_foot_z_m);
    packed.dfoot_x_dphase = static_cast<float>(sample.dfoot_x_dphase);
    packed.dfoot_z_dphase = static_cast<float>(sample.dfoot_z_dphase);
    packed.stance_confidence = static_cast<float>(sample.stance_confidence);
    out.write(reinterpret_cast<const char *>(&packed), sizeof(packed));
  }

  return out.good();
}

std::vector<LegModel> LinkageSolver::load_models_from_yaml(const std::string & yaml_path)
{
  const std::string expanded_yaml_path = expand_user_path(yaml_path);
  YAML::Node root = YAML::LoadFile(expanded_yaml_path);
  std::vector<LegModel> models;
  LinkageSolver solver;

  for (const auto & leg_node : root["legs"]) {
    LegModel model;
    model.name = leg_node["name"].as<std::string>();
    model.mount.shaft_x_m = leg_node["mount"]["shaft_x_m"].as<double>();
    model.mount.shaft_y_m = leg_node["mount"]["shaft_y_m"].as<double>();
    model.mount.shaft_z_m = leg_node["mount"]["shaft_z_m"] ? leg_node["mount"]["shaft_z_m"].as<double>() : 0.0;
    model.mount.yaw_rad = leg_node["mount"]["yaw_rad"] ? leg_node["mount"]["yaw_rad"].as<double>() : 0.0;
    model.input.shaft_point = leg_node["input"]["shaft_point"].as<std::string>();
    model.input.crank_tip_name = leg_node["input"]["crank_tip_name"].as<std::string>();
    model.input.crank_radius = leg_node["input"]["crank_radius_m"].as<double>();
    model.input.phase_offset_rad = leg_node["input"]["phase_offset_rad"].as<double>();
    model.foot_point_name = leg_node["foot_point_name"].as<std::string>();
    model.stance_phase_min_rad = leg_node["stance_phase_window_rad"][0].as<double>();
    model.stance_phase_max_rad = leg_node["stance_phase_window_rad"][1].as<double>();
    model.use_lookup_table = leg_node["use_lookup_table"] ? leg_node["use_lookup_table"].as<bool>() : true;
    model.lookup_resolution = leg_node["lookup_resolution"] ? leg_node["lookup_resolution"].as<int>() : 1024;
    model.lookup_file_path = leg_node["lookup_file"] ? resolve_path(yaml_path, leg_node["lookup_file"].as<std::string>()) : std::string();
    model.lookup_generate_if_missing = leg_node["lookup_generate_if_missing"] ? leg_node["lookup_generate_if_missing"].as<bool>() : true;
    model.lookup_save_generated = leg_node["lookup_save_generated"] ? leg_node["lookup_save_generated"].as<bool>() : false;

    for (const auto & entry : leg_node["fixed_points"]) {
      const std::string key = entry.first.as<std::string>();
      const auto vec = entry.second.as<std::vector<double>>();
      model.fixed_points[key] = Eigen::Vector2d(vec.at(0), vec.at(1));
    }

    for (const auto & entry : leg_node["unknown_points"]) {
      model.unknown_points.push_back(entry.as<std::string>());
    }

    for (const auto & entry : leg_node["distance_constraints"]) {
      DistanceConstraint c;
      c.a = entry["a"].as<std::string>();
      c.b = entry["b"].as<std::string>();
      c.length = entry["length_m"].as<double>();
      model.distance_constraints.push_back(c);
    }

    for (const auto & entry : leg_node["angle_constraints"]) {
      AngleConstraint c;
      c.a = entry["a"].as<std::string>();
      c.vertex = entry["vertex"].as<std::string>();
      c.c = entry["c"].as<std::string>();
      c.angle_rad = entry["angle_rad"].as<double>();
      model.angle_constraints.push_back(c);
    }

    if (leg_node["initial_guess"]) {
      for (const auto & entry : leg_node["initial_guess"]) {
        const std::string key = entry.first.as<std::string>();
        const auto vec = entry.second.as<std::vector<double>>();
        model.initial_guess[key] = Eigen::Vector2d(vec.at(0), vec.at(1));
      }
    }

    if (model.use_lookup_table) {
      bool loaded = false;
      if (file_exists(model.lookup_file_path)) {
        loaded = load_lookup_table_binary(model, model.lookup_file_path);
      }
      if (!loaded) {
        if (!model.lookup_generate_if_missing) {
          throw std::runtime_error("Lookup table missing for leg " + model.name + ": " + model.lookup_file_path);
        }
        solver.generate_lookup_table(model);
        if (model.lookup_save_generated && !model.lookup_file_path.empty()) {
          (void)save_lookup_table_binary(model, model.lookup_file_path);
        }
      }
    }
    models.push_back(model);
  }

  return models;
}

bool LinkageSolver::solve_unknown_positions(
  const LegModel & model,
  double phase_rad,
  std::unordered_map<std::string, Eigen::Vector2d> & points)
{
  const std::string & shaft_name = model.input.shaft_point;
  const Eigen::Vector2d shaft = points.at(shaft_name);
  points[model.input.crank_tip_name] = shaft +
    Eigen::Vector2d(
    model.input.crank_radius * std::cos(phase_rad + model.input.phase_offset_rad),
    model.input.crank_radius * std::sin(phase_rad + model.input.phase_offset_rad));

  const int n_unknown = static_cast<int>(model.unknown_points.size());
  if (n_unknown == 0) {
    return true;
  }

  Eigen::VectorXd x(2 * n_unknown);
  for (int i = 0; i < n_unknown; ++i) {
    const auto & name = model.unknown_points[static_cast<std::size_t>(i)];
    auto it = points.find(name);
    if (it == points.end()) {
      return false;
    }
    x(2 * i) = it->second.x();
    x(2 * i + 1) = it->second.y();
  }

  auto point_for = [&](const std::string & name, const Eigen::VectorXd & xv) -> Eigen::Vector2d {
      auto fixed_it = points.find(name);
      if (fixed_it != points.end()) {
        for (int i = 0; i < n_unknown; ++i) {
          if (model.unknown_points[static_cast<std::size_t>(i)] == name) {
            return Eigen::Vector2d(xv(2 * i), xv(2 * i + 1));
          }
        }
        return fixed_it->second;
      }
      return Eigen::Vector2d::Zero();
    };

  const int residual_count = static_cast<int>(model.distance_constraints.size() + model.angle_constraints.size());
  if (residual_count == 0) {
    return true;
  }

  for (int iter = 0; iter < 20; ++iter) {
    Eigen::VectorXd r(residual_count);
    Eigen::MatrixXd J(residual_count, 2 * n_unknown);
    J.setZero();

    int row = 0;
    for (const auto & dc : model.distance_constraints) {
      const Eigen::Vector2d pa = point_for(dc.a, x);
      const Eigen::Vector2d pb = point_for(dc.b, x);
      const Eigen::Vector2d d = pa - pb;
      const double dist = d.norm();
      r(row) = dist - dc.length;

      if (dist > 1.0e-12) {
        for (int i = 0; i < n_unknown; ++i) {
          const auto & name = model.unknown_points[static_cast<std::size_t>(i)];
          if (name == dc.a) {
            J(row, 2 * i) = d.x() / dist;
            J(row, 2 * i + 1) = d.y() / dist;
          }
          if (name == dc.b) {
            J(row, 2 * i) = -d.x() / dist;
            J(row, 2 * i + 1) = -d.y() / dist;
          }
        }
      }
      ++row;
    }

    for (const auto & ac : model.angle_constraints) {
      const Eigen::Vector2d pa = point_for(ac.a, x);
      const Eigen::Vector2d pv = point_for(ac.vertex, x);
      const Eigen::Vector2d pc = point_for(ac.c, x);
      const Eigen::Vector2d va = pa - pv;
      const Eigen::Vector2d vc = pc - pv;
      const double angle = wrap_to_pi(angle_of(vc) - angle_of(va));
      r(row) = wrap_to_pi(angle - ac.angle_rad);

      const double eps = 1.0e-6;
      for (int i = 0; i < 2 * n_unknown; ++i) {
        Eigen::VectorXd x_pert = x;
        x_pert(i) += eps;

        const Eigen::Vector2d pa2 = point_for(ac.a, x_pert);
        const Eigen::Vector2d pv2 = point_for(ac.vertex, x_pert);
        const Eigen::Vector2d pc2 = point_for(ac.c, x_pert);
        const Eigen::Vector2d va2 = pa2 - pv2;
        const Eigen::Vector2d vc2 = pc2 - pv2;
        const double angle2 = wrap_to_pi(angle_of(vc2) - angle_of(va2));
        J(row, i) = wrap_to_pi(angle2 - angle) / eps;
      }
      ++row;
    }

    const Eigen::MatrixXd H = J.transpose() * J + 1.0e-8 * Eigen::MatrixXd::Identity(2 * n_unknown, 2 * n_unknown);
    const Eigen::VectorXd g = J.transpose() * r;
    const Eigen::VectorXd dx = -H.ldlt().solve(g);
    x += dx;
    if (dx.norm() < 1.0e-9) {
      break;
    }
  }

  for (int i = 0; i < n_unknown; ++i) {
    points[model.unknown_points[static_cast<std::size_t>(i)]] = Eigen::Vector2d(x(2 * i), x(2 * i + 1));
  }
  return true;
}

void LinkageSolver::generate_lookup_table(LegModel & model)
{
  model.lookup_resolution = std::max(128, model.lookup_resolution);
  model.lookup_table.clear();
  model.lookup_table.resize(static_cast<std::size_t>(model.lookup_resolution));

  std::unordered_map<std::string, Eigen::Vector2d> points;
  for (const auto & kv : model.fixed_points) {
    points[kv.first] = kv.second;
  }
  for (const auto & name : model.unknown_points) {
    auto it = model.initial_guess.find(name);
    points[name] = (it != model.initial_guess.end()) ? it->second : Eigen::Vector2d::Zero();
  }
  points[model.input.crank_tip_name] = Eigen::Vector2d::Zero();

  std::vector<double> local_x(static_cast<std::size_t>(model.lookup_resolution), 0.0);
  std::vector<double> local_z(static_cast<std::size_t>(model.lookup_resolution), 0.0);
  std::vector<double> phase_conf(static_cast<std::size_t>(model.lookup_resolution), 0.0);

  model.lookup_min_local_z_m = std::numeric_limits<double>::infinity();
  model.lookup_max_local_z_m = -std::numeric_limits<double>::infinity();
  model.lookup_loaded_from_file = false;

  for (int i = 0; i < model.lookup_resolution; ++i) {
    const double phase = kTwoPi * static_cast<double>(i) / static_cast<double>(model.lookup_resolution);
    const bool ok = solve_unknown_positions(model, phase, points);
    if (!ok) {
      if (i > 0) {
        local_x[static_cast<std::size_t>(i)] = local_x[static_cast<std::size_t>(i - 1)];
        local_z[static_cast<std::size_t>(i)] = local_z[static_cast<std::size_t>(i - 1)];
      }
    } else {
      const auto foot = points.at(model.foot_point_name);
      local_x[static_cast<std::size_t>(i)] = foot.x();
      local_z[static_cast<std::size_t>(i)] = foot.y();
      for (const auto & name : model.unknown_points) {
        model.initial_guess[name] = points.at(name);
      }
    }
    phase_conf[static_cast<std::size_t>(i)] = unwrap_phase_window_confidence(phase, model.stance_phase_min_rad, model.stance_phase_max_rad);
    model.lookup_min_local_z_m = std::min(model.lookup_min_local_z_m, local_z[static_cast<std::size_t>(i)]);
    model.lookup_max_local_z_m = std::max(model.lookup_max_local_z_m, local_z[static_cast<std::size_t>(i)]);
  }

  const double dphase = kTwoPi / static_cast<double>(model.lookup_resolution);
  const double z_span = std::max(1.0e-6, model.lookup_max_local_z_m - model.lookup_min_local_z_m);
  for (int i = 0; i < model.lookup_resolution; ++i) {
    const int ip = (i + 1) % model.lookup_resolution;
    const int im = (i - 1 + model.lookup_resolution) % model.lookup_resolution;
    const double dxdphi = (local_x[static_cast<std::size_t>(ip)] - local_x[static_cast<std::size_t>(im)]) / (2.0 * dphase);
    const double dzdphi = (local_z[static_cast<std::size_t>(ip)] - local_z[static_cast<std::size_t>(im)]) / (2.0 * dphase);
    const double height_norm = std::clamp((local_z[static_cast<std::size_t>(i)] - model.lookup_min_local_z_m) / z_span, 0.0, 1.0);
    const double height_conf = std::clamp(1.0 - height_norm / 0.22, 0.0, 1.0);
    const double vertical_motion_conf = std::clamp(1.0 - std::abs(dzdphi) / std::max(1.0e-6, z_span), 0.0, 1.0);
    LookupSample s;
    s.phase_rad = kTwoPi * static_cast<double>(i) / static_cast<double>(model.lookup_resolution);
    s.local_foot_x_m = local_x[static_cast<std::size_t>(i)];
    s.local_foot_z_m = local_z[static_cast<std::size_t>(i)];
    s.dfoot_x_dphase = dxdphi;
    s.dfoot_z_dphase = dzdphi;
    s.stance_confidence = std::clamp(phase_conf[static_cast<std::size_t>(i)] * (0.65 * height_conf + 0.35 * vertical_motion_conf), 0.0, 1.0);
    model.lookup_table[static_cast<std::size_t>(i)] = s;
  }
}

bool LinkageSolver::solve_points(
  LegModel & model,
  double phase_rad,
  std::unordered_map<std::string, Eigen::Vector2d> & points)
{
  points.clear();
  for (const auto & kv : model.fixed_points) {
    points[kv.first] = kv.second;
  }
  for (const auto & name : model.unknown_points) {
    auto it = model.initial_guess.find(name);
    points[name] = (it != model.initial_guess.end()) ? it->second : Eigen::Vector2d::Zero();
  }
  points[model.input.crank_tip_name] = Eigen::Vector2d::Zero();

  if (!solve_unknown_positions(model, phase_rad, points)) {
    return false;
  }

  for (const auto & name : model.unknown_points) {
    model.initial_guess[name] = points.at(name);
  }
  return true;
}

LegKinematicState LinkageSolver::evaluate(
  LegModel & model,
  double phase_rad,
  double phase_velocity_rad_s,
  double dt_hint_s)
{
  (void)dt_hint_s;
  LegKinematicState state;

  if (model.use_lookup_table && !model.lookup_table.empty()) {
    const auto lookup = interpolate_lookup(model, phase_rad);
    if (!lookup.valid) {
      return state;
    }

    const Eigen::Vector2d horizontal_local(lookup.local_x, 0.0);
    const auto mounted_foot_xy = rotate2d(horizontal_local, model.mount.yaw_rad) +
      Eigen::Vector2d(model.mount.shaft_x_m, model.mount.shaft_y_m);
    const Eigen::Vector2d horizontal_local_vel(lookup.dlocal_x_dphase * phase_velocity_rad_s, 0.0);
    const auto mounted_foot_vel_xy = rotate2d(horizontal_local_vel, model.mount.yaw_rad);

    state.foot_body_xy = mounted_foot_xy;
    state.foot_velocity_body_xy = mounted_foot_vel_xy;
    state.foot_body_z = model.mount.shaft_z_m + lookup.local_z;
    state.foot_velocity_body_z = lookup.dlocal_z_dphase * phase_velocity_rad_s;
    state.stance_confidence = lookup.stance_confidence;
    state.valid = true;
    return state;
  }

  std::unordered_map<std::string, Eigen::Vector2d> points;
  for (const auto & kv : model.fixed_points) {
    points[kv.first] = kv.second;
  }
  for (const auto & name : model.unknown_points) {
    auto it = model.initial_guess.find(name);
    points[name] = (it != model.initial_guess.end()) ? it->second : Eigen::Vector2d::Zero();
  }
  points[model.input.crank_tip_name] = Eigen::Vector2d::Zero();

  if (!solve_unknown_positions(model, phase_rad, points)) {
    return state;
  }

  for (const auto & name : model.unknown_points) {
    model.initial_guess[name] = points.at(name);
  }

  const auto local_foot = points.at(model.foot_point_name);
  const Eigen::Vector2d horizontal_local(local_foot.x(), 0.0);
  const auto mounted_foot_xy = rotate2d(horizontal_local, model.mount.yaw_rad) +
    Eigen::Vector2d(model.mount.shaft_x_m, model.mount.shaft_y_m);

  state.foot_body_xy = mounted_foot_xy;
  state.foot_body_z = model.mount.shaft_z_m + local_foot.y();
  state.stance_confidence = unwrap_phase_window_confidence(phase_rad, model.stance_phase_min_rad, model.stance_phase_max_rad);
  state.valid = true;

  if (std::abs(phase_velocity_rad_s) > 1.0e-9 && dt_hint_s > 0.0) {
    std::unordered_map<std::string, Eigen::Vector2d> points2;
    for (const auto & kv : model.fixed_points) {
      points2[kv.first] = kv.second;
    }
    for (const auto & name : model.unknown_points) {
      points2[name] = model.initial_guess[name];
    }
    points2[model.input.crank_tip_name] = Eigen::Vector2d::Zero();

    const double phase2 = phase_rad + phase_velocity_rad_s * dt_hint_s;
    if (solve_unknown_positions(model, phase2, points2)) {
      const auto local_foot2 = points2.at(model.foot_point_name);
      const Eigen::Vector2d horizontal_local2(local_foot2.x(), 0.0);
      const auto mounted_foot_xy2 = rotate2d(horizontal_local2, model.mount.yaw_rad) +
        Eigen::Vector2d(model.mount.shaft_x_m, model.mount.shaft_y_m);
      state.foot_velocity_body_xy = (mounted_foot_xy2 - mounted_foot_xy) / dt_hint_s;
      state.foot_velocity_body_z = ((model.mount.shaft_z_m + local_foot2.y()) - state.foot_body_z) / dt_hint_s;
    }
  }

  return state;
}

}  // namespace hexapod_hardware_cpp
