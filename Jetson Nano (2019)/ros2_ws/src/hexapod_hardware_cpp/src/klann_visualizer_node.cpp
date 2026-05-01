#include "hexapod_hardware_cpp/klann_geometry.hpp"
#include "hexapod_hardware_cpp/path_utils.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "dsy_motor_msgs/msg/motor_output_odometry_array.hpp"
#include "hexapod_control_interfaces/msg/hexapod_motor_state.hpp"
#include "hexapod_control_interfaces/msg/klann_body_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace
{
struct BBox
{
  double min_x{-0.33};
  double max_x{0.43};
  double min_y{-0.55};
  double max_y{0.55};
  double min_z{-0.09};
  double max_z{0.37};
};

geometry_msgs::msg::Point make_point(double x, double y, double z)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

Eigen::Vector3d body_from_local(const hexapod_hardware_cpp::LegModel & model, const Eigen::Vector2d & local)
{
  return {model.mount.shaft_x_m + local.x(), model.mount.shaft_y_m, model.mount.shaft_z_m + local.y()};
}

constexpr std::array<std::size_t, 6> kCanonicalToWireIndex{{2, 1, 0, 3, 4, 5}};

}  // namespace

namespace hexapod_hardware_cpp
{

class KlannVisualizerNode : public rclcpp::Node
{
public:
  KlannVisualizerNode()
  : Node("klann_visualizer_node")
  {
    linkage_yaml_ = declare_parameter<std::string>("linkage_yaml", "");
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");
    odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
    apply_body_pose_transform_ = declare_parameter<bool>("apply_body_pose_transform", true);
    mirror_model_across_y_axis_ = declare_parameter<bool>("mirror_model_across_y_axis", true);
    marker_topic_ = declare_parameter<std::string>("marker_topic", "klann_markers");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
    show_lookup_curves_ = declare_parameter<bool>("show_lookup_curves", true);
    show_bbox_ = declare_parameter<bool>("show_bbox", true);
    stamp_markers_zero_time_ = declare_parameter<bool>("stamp_markers_zero_time", true);
    animate_ = declare_parameter<bool>("animate", false);
    animate_speed_rad_s_ = declare_parameter<double>("animate_speed_rad_s", 0.5);
    subscribe_motor_state_ = declare_parameter<bool>("subscribe_motor_state", true);
    subscribe_compact_odometry_ = declare_parameter<bool>("subscribe_compact_odometry", false);
    motor_state_topic_ = declare_parameter<std::string>("motor_state_topic", "/hexapod/motor_state");
    subscribe_body_state_ = declare_parameter<bool>("subscribe_body_state", true);
    body_state_topic_ = declare_parameter<std::string>("body_state_topic", "/hexapod/body_state");
    compact_odometry_topic_ = declare_parameter<std::string>("compact_odometry_topic", "/motor_output_odom");
    std::vector<double> default_phases(6, 0.0);
    phases_ = declare_parameter<std::vector<double>>("phases_rad", default_phases);
    if (phases_.size() != 6U) {
      phases_.assign(6U, 0.0);
    }

    linkage_yaml_ = linkage_yaml_.empty() ? auto_linkage_yaml("hexapod_hardware_cpp") : expand_user_path(linkage_yaml_);
    RCLCPP_INFO(get_logger(), "Using linkage YAML: %s", linkage_yaml_.c_str());
    models_ = LinkageSolver::load_models_from_yaml(linkage_yaml_);
    load_bbox();

    marker_pub_ = create_publisher<MarkerArray>(marker_topic_, 1);
    if (subscribe_motor_state_) {
      motor_sub_ = create_subscription<hexapod_control_interfaces::msg::HexapodMotorState>(
        motor_state_topic_, rclcpp::SensorDataQoS(),
        [this](hexapod_control_interfaces::msg::HexapodMotorState::SharedPtr msg) {
          for (std::size_t i = 0; i < 6 && i < msg->phase_rad.size(); ++i) {
            phases_[i] = msg->phase_rad[i];
          }
        });
    }
    if (subscribe_body_state_) {
      body_sub_ = create_subscription<hexapod_control_interfaces::msg::KlannBodyState>(
        body_state_topic_, rclcpp::SensorDataQoS(),
        [this](hexapod_control_interfaces::msg::KlannBodyState::SharedPtr msg) {
          latest_body_state_ = msg;
          for (std::size_t i = 0; i < 6 && i < msg->phase_rad.size(); ++i) {
            phases_[i] = msg->phase_rad[i];
          }
        });
    }
    if (subscribe_compact_odometry_) {
      compact_odom_sub_ = create_subscription<dsy_motor_msgs::msg::MotorOutputOdometryArray>(
        compact_odometry_topic_, rclcpp::SensorDataQoS(),
        [this](dsy_motor_msgs::msg::MotorOutputOdometryArray::SharedPtr msg) {
          for (std::size_t i = 0; i < 6 && i < msg->output_phase_rad.size(); ++i) {
            phases_[i] = msg->output_phase_rad[kCanonicalToWireIndex[i]];
          }
        });
    }

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&KlannVisualizerNode::publish_markers, this));
  }

private:
  void load_bbox()
  {
    try {
      YAML::Node root = YAML::LoadFile(linkage_yaml_);
      auto bbox = root["robot_frame_bbox"];
      if (bbox) {
        bbox_.min_x = bbox["min_x_m"] ? bbox["min_x_m"].as<double>() : bbox_.min_x;
        bbox_.max_x = bbox["max_x_m"] ? bbox["max_x_m"].as<double>() : bbox_.max_x;
        bbox_.min_y = bbox["min_y_m"] ? bbox["min_y_m"].as<double>() : bbox_.min_y;
        bbox_.max_y = bbox["max_y_m"] ? bbox["max_y_m"].as<double>() : bbox_.max_y;
        bbox_.min_z = bbox["min_z_m"] ? bbox["min_z_m"].as<double>() : bbox_.min_z;
        bbox_.max_z = bbox["max_z_m"] ? bbox["max_z_m"].as<double>() : bbox_.max_z;
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Failed to load robot_frame_bbox from %s: %s", linkage_yaml_.c_str(), e.what());
    }
  }

  builtin_interfaces::msg::Time marker_stamp() const
  {
    builtin_interfaces::msg::Time stamp;
    if (!stamp_markers_zero_time_) {
      stamp = now();
    }
    return stamp;
  }

  std::string current_frame_id() const
  {
    if (apply_body_pose_transform_ && latest_body_state_) {
      return odom_frame_id_;
    }
    return frame_id_;
  }

  geometry_msgs::msg::Point visual_point(const Eigen::Vector3d & body_point) const
  {
    Eigen::Vector3d p = body_point;
    if (mirror_model_across_y_axis_) {
      p.x() = -p.x();
    }

    if (apply_body_pose_transform_ && latest_body_state_) {
      const auto & q_msg = latest_body_state_->pose.orientation;
      Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
      if (std::abs(q.norm() - 1.0) > 1.0e-3) {
        q.normalize();
      }
      p = q * p + Eigen::Vector3d(
        latest_body_state_->pose.position.x,
        latest_body_state_->pose.position.y,
        latest_body_state_->pose.position.z);
    }

    return make_point(p.x(), p.y(), p.z());
  }

  Marker make_line_list(int id, const std::string & ns, double scale, float r, float g, float b, float a) const
  {
    Marker m;
    m.header.frame_id = current_frame_id();
    m.header.stamp = marker_stamp();
    m.ns = ns;
    m.id = id;
    m.type = Marker::LINE_LIST;
    m.action = Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = scale;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = a;
    return m;
  }

  Marker make_line_strip(int id, const std::string & ns, double scale, float r, float g, float b, float a) const
  {
    Marker m = make_line_list(id, ns, scale, r, g, b, a);
    m.type = Marker::LINE_STRIP;
    return m;
  }

  Marker make_spheres(int id, const std::string & ns, double scale, float r, float g, float b, float a) const
  {
    Marker m;
    m.header.frame_id = current_frame_id();
    m.header.stamp = marker_stamp();
    m.ns = ns;
    m.id = id;
    m.type = Marker::SPHERE_LIST;
    m.action = Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = a;
    return m;
  }

  void publish_markers()
  {
    if (animate_ && !subscribe_motor_state_ && !subscribe_compact_odometry_) {
      const double dt = 1.0 / std::max(1.0, publish_rate_hz_);
      for (double & p : phases_) {
        p = LinkageSolver::wrap_to_2pi(p + animate_speed_rad_s_ * dt);
      }
    }

    MarkerArray array;
    Marker clear;
    clear.action = Marker::DELETEALL;
    clear.header.frame_id = current_frame_id();
    clear.header.stamp = marker_stamp();
    array.markers.push_back(clear);

    int id = 0;
    if (show_bbox_) {
      Marker box = make_line_list(id++, "bbox", 0.008, 0.9f, 0.9f, 0.9f, 1.0f);
      const auto p000 = visual_point(Eigen::Vector3d(bbox_.min_x, bbox_.min_y, bbox_.min_z));
      const auto p001 = visual_point(Eigen::Vector3d(bbox_.min_x, bbox_.min_y, bbox_.max_z));
      const auto p010 = visual_point(Eigen::Vector3d(bbox_.min_x, bbox_.max_y, bbox_.min_z));
      const auto p011 = visual_point(Eigen::Vector3d(bbox_.min_x, bbox_.max_y, bbox_.max_z));
      const auto p100 = visual_point(Eigen::Vector3d(bbox_.max_x, bbox_.min_y, bbox_.min_z));
      const auto p101 = visual_point(Eigen::Vector3d(bbox_.max_x, bbox_.min_y, bbox_.max_z));
      const auto p110 = visual_point(Eigen::Vector3d(bbox_.max_x, bbox_.max_y, bbox_.min_z));
      const auto p111 = visual_point(Eigen::Vector3d(bbox_.max_x, bbox_.max_y, bbox_.max_z));
      auto add_edge = [&](const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b) {
        box.points.push_back(a); box.points.push_back(b);
      };
      add_edge(p000, p001); add_edge(p000, p010); add_edge(p000, p100);
      add_edge(p111, p110); add_edge(p111, p101); add_edge(p111, p011);
      add_edge(p001, p011); add_edge(p001, p101);
      add_edge(p010, p011); add_edge(p010, p110);
      add_edge(p100, p101); add_edge(p100, p110);
      array.markers.push_back(box);
    }

    Marker feet = make_spheres(id++, "feet", 0.03, 1.0f, 0.35f, 0.2f, 1.0f);

    for (std::size_t i = 0; i < models_.size() && i < phases_.size(); ++i) {
      auto & model = models_[i];
      const double phase = phases_[i];

      if (show_lookup_curves_ && !model.lookup_table.empty()) {
        Marker curve = make_line_strip(id++, "lookup_curve", 0.004, 0.2f, 0.8f, 1.0f, 0.7f);
        const int stride = std::max(1, model.lookup_resolution / 256);
        for (int k = 0; k < model.lookup_resolution; k += stride) {
          const auto & s = model.lookup_table[static_cast<std::size_t>(k)];
          curve.points.push_back(visual_point(Eigen::Vector3d(model.mount.shaft_x_m + s.local_foot_x_m, model.mount.shaft_y_m, model.mount.shaft_z_m + s.local_foot_z_m)));
        }
        if (!curve.points.empty()) {
          curve.points.push_back(curve.points.front());
        }
        array.markers.push_back(curve);
      }

      std::unordered_map<std::string, Eigen::Vector2d> points;
      if (!solver_.solve_points(model, phase, points)) {
        continue;
      }

      Marker links = make_line_list(id++, "linkage", 0.006, 0.95f, 0.85f, 0.15f, 1.0f);
      auto add_seg = [&](const std::string & a, const std::string & b) {
        const auto pa = body_from_local(model, points.at(a));
        const auto pb = body_from_local(model, points.at(b));
        links.points.push_back(visual_point(pa));
        links.points.push_back(visual_point(pb));
      };
      add_seg(model.input.shaft_point, model.input.crank_tip_name);
      add_seg(model.input.crank_tip_name, "joint_k");
      add_seg("bottom_pivot", "joint_k");
      add_seg("joint_k", "joint_m");
      add_seg("top_pivot", "joint_q");
      add_seg("joint_q", "joint_m");
      add_seg("joint_m", model.foot_point_name);
      array.markers.push_back(links);

      const auto foot = body_from_local(model, points.at(model.foot_point_name));
      feet.points.push_back(visual_point(foot));
    }

    array.markers.push_back(feet);
    marker_pub_->publish(array);
  }

  LinkageSolver solver_;
  std::vector<LegModel> models_;
  BBox bbox_;
  std::string linkage_yaml_;
  std::string frame_id_;
  std::string odom_frame_id_;
  std::string marker_topic_;
  std::string motor_state_topic_;
  std::string body_state_topic_;
  std::string compact_odometry_topic_;
  double publish_rate_hz_{10.0};
  bool show_lookup_curves_{true};
  bool show_bbox_{true};
  bool stamp_markers_zero_time_{true};
  bool apply_body_pose_transform_{true};
  bool mirror_model_across_y_axis_{true};
  bool animate_{false};
  double animate_speed_rad_s_{0.5};
  bool subscribe_motor_state_{false};
  bool subscribe_body_state_{true};
  bool subscribe_compact_odometry_{true};
  std::vector<double> phases_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<hexapod_control_interfaces::msg::HexapodMotorState>::SharedPtr motor_sub_;
  rclcpp::Subscription<hexapod_control_interfaces::msg::KlannBodyState>::SharedPtr body_sub_;
  rclcpp::Subscription<dsy_motor_msgs::msg::MotorOutputOdometryArray>::SharedPtr compact_odom_sub_;
  hexapod_control_interfaces::msg::KlannBodyState::SharedPtr latest_body_state_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace hexapod_hardware_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hexapod_hardware_cpp::KlannVisualizerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
