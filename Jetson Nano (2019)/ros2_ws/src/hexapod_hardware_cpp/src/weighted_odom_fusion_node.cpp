
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
#include <memory>
#include <string>

namespace
{
double wrap_to_pi(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

void quaternion_to_rpy(const geometry_msgs::msg::Quaternion & q, double & roll, double & pitch, double & yaw)
{
  const double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
  const double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  roll = std::atan2(sinr_cosp, cosr_cosp);
  const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
  pitch = std::abs(sinp) >= 1.0 ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  yaw = std::atan2(siny_cosp, cosy_cosp);
}
}

namespace hexapod_hardware_cpp
{

class WeightedOdomFusionNode : public rclcpp::Node
{
public:
  WeightedOdomFusionNode()
  : Node("weighted_odom_fusion_node"), tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(*this))
  {
    kinematic_odom_topic_ = declare_parameter<std::string>("kinematic_odom_topic", "/hexapod/kinematic_odom");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "imu/data_raw");
    fused_odom_topic_ = declare_parameter<std::string>("fused_odom_topic", "/hexapod/fused_odom");
    generic_odom_topic_ = declare_parameter<std::string>("generic_odom_topic", "/odom");
    use_orbslam_odom_ = declare_parameter<bool>("use_orbslam_odom", true);
    orbslam_odom_topic_ = declare_parameter<std::string>("orbslam_odom_topic", "/hexapod/orbslam_odom");
    orbslam_timeout_s_ = declare_parameter<double>("orbslam_timeout_s", 0.5);
    orbslam_position_weight_xy_ = declare_parameter<double>("orbslam_position_weight_xy", 0.15);
    orbslam_yaw_weight_ = declare_parameter<double>("orbslam_yaw_weight", 0.20);
    orbslam_trans_gate_m_ = declare_parameter<double>("orbslam_trans_gate_m", 0.40);
    orbslam_yaw_gate_rad_ = declare_parameter<double>("orbslam_yaw_gate_rad", 0.70);
    use_orbslam_linear_twist_ = declare_parameter<bool>("use_orbslam_linear_twist", false);
    orbslam_use_relative_origin_ = declare_parameter<bool>("orbslam_use_relative_origin", true);
    publish_generic_odom_ = declare_parameter<bool>("publish_generic_odom", true);
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");
    yaw_from_imu_weight_ = declare_parameter<double>("yaw_from_imu_weight", 0.85);
    roll_from_imu_weight_ = declare_parameter<double>("roll_from_imu_weight", 1.0);
    pitch_from_imu_weight_ = declare_parameter<double>("pitch_from_imu_weight", 1.0);
    use_imu_angular_velocity_ = declare_parameter<bool>("use_imu_angular_velocity", true);
    force_planar_pose_ = declare_parameter<bool>("force_planar_pose", false);

    kin_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      kinematic_odom_topic_, 20, std::bind(&WeightedOdomFusionNode::on_kinematic_odom, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(), std::bind(&WeightedOdomFusionNode::on_imu, this, std::placeholders::_1));
    if (use_orbslam_odom_) {
      orbslam_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        orbslam_odom_topic_, rclcpp::SensorDataQoS(), std::bind(&WeightedOdomFusionNode::on_orbslam_odom, this, std::placeholders::_1));
    }

    fused_pub_ = create_publisher<nav_msgs::msg::Odometry>(fused_odom_topic_, 10);
    if (publish_generic_odom_) {
      generic_pub_ = create_publisher<nav_msgs::msg::Odometry>(generic_odom_topic_, 10);
    }
  }

private:
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    latest_imu_ = msg;
  }

  void on_orbslam_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_orbslam_ = msg;
  }

  void get_planar_orbslam_pose(const nav_msgs::msg::Odometry & odom, double & x, double & y, double & yaw)
  {
    double roll = 0.0;
    double pitch = 0.0;
    quaternion_to_rpy(odom.pose.pose.orientation, roll, pitch, yaw);
    x = odom.pose.pose.position.x;
    y = odom.pose.pose.position.y;

    if (!orbslam_use_relative_origin_) {
      return;
    }

    if (!has_orbslam_origin_) {
      orbslam_origin_x_ = x;
      orbslam_origin_y_ = y;
      orbslam_origin_yaw_ = yaw;
      has_orbslam_origin_ = true;
    }

    const double dx = x - orbslam_origin_x_;
    const double dy = y - orbslam_origin_y_;
    const double c = std::cos(-orbslam_origin_yaw_);
    const double s = std::sin(-orbslam_origin_yaw_);
    x = c * dx - s * dy;
    y = s * dx + c * dy;
    yaw = wrap_to_pi(yaw - orbslam_origin_yaw_);
  }

  void on_kinematic_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    nav_msgs::msg::Odometry fused = *msg;
    fused.header.frame_id = odom_frame_id_;
    fused.child_frame_id = base_frame_id_;

    double kin_roll = 0.0, kin_pitch = 0.0, kin_yaw = 0.0;
    quaternion_to_rpy(msg->pose.pose.orientation, kin_roll, kin_pitch, kin_yaw);

    double fused_roll = kin_roll;
    double fused_pitch = kin_pitch;
    double fused_yaw = kin_yaw;

    if (latest_imu_) {
      double imu_roll = 0.0, imu_pitch = 0.0, imu_yaw = 0.0;
      quaternion_to_rpy(latest_imu_->orientation, imu_roll, imu_pitch, imu_yaw);

      fused_roll = (1.0 - roll_from_imu_weight_) * kin_roll + roll_from_imu_weight_ * imu_roll;
      fused_pitch = (1.0 - pitch_from_imu_weight_) * kin_pitch + pitch_from_imu_weight_ * imu_pitch;
      const double yaw_err = wrap_to_pi(imu_yaw - kin_yaw);
      fused_yaw = wrap_to_pi(kin_yaw + yaw_from_imu_weight_ * yaw_err);

      if (use_imu_angular_velocity_) {
        fused.twist.twist.angular = latest_imu_->angular_velocity;
      }
    }

    if (use_orbslam_odom_ && latest_orbslam_) {
      const double age_s = std::abs((rclcpp::Time(msg->header.stamp) - rclcpp::Time(latest_orbslam_->header.stamp)).seconds());
      if (age_s <= orbslam_timeout_s_) {
        double orb_x = 0.0;
        double orb_y = 0.0;
        double orb_yaw = 0.0;
        get_planar_orbslam_pose(*latest_orbslam_, orb_x, orb_y, orb_yaw);
        const double dx = orb_x - fused.pose.pose.position.x;
        const double dy = orb_y - fused.pose.pose.position.y;
        const double trans_err = std::sqrt(dx * dx + dy * dy);
        const double yaw_err_orb = wrap_to_pi(orb_yaw - fused_yaw);
        if (trans_err <= orbslam_trans_gate_m_ && std::abs(yaw_err_orb) <= orbslam_yaw_gate_rad_) {
          fused.pose.pose.position.x += orbslam_position_weight_xy_ * dx;
          fused.pose.pose.position.y += orbslam_position_weight_xy_ * dy;
          fused_yaw = wrap_to_pi(fused_yaw + orbslam_yaw_weight_ * yaw_err_orb);
          if (use_orbslam_linear_twist_) {
            fused.twist.twist.linear = latest_orbslam_->twist.twist.linear;
          }
        }
      }
    }

    if (force_planar_pose_) {
      fused_roll = 0.0;
      fused_pitch = 0.0;
      fused.pose.pose.position.z = 0.0;
      fused.twist.twist.linear.z = 0.0;
      fused.twist.twist.angular.x = 0.0;
      fused.twist.twist.angular.y = 0.0;
    }

    tf2::Quaternion q;
    q.setRPY(fused_roll, fused_pitch, fused_yaw);
    fused.pose.pose.orientation.x = q.x();
    fused.pose.pose.orientation.y = q.y();
    fused.pose.pose.orientation.z = q.z();
    fused.pose.pose.orientation.w = q.w();

    fused_pub_->publish(fused);
    if (generic_pub_) {
      generic_pub_->publish(fused);
    }

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = fused.header;
      tf.header.frame_id = odom_frame_id_;
      tf.child_frame_id = base_frame_id_;
      tf.transform.translation.x = fused.pose.pose.position.x;
      tf.transform.translation.y = fused.pose.pose.position.y;
      tf.transform.translation.z = fused.pose.pose.position.z;
      tf.transform.rotation = fused.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }
  }

  std::string kinematic_odom_topic_;
  std::string imu_topic_;
  std::string fused_odom_topic_;
  std::string generic_odom_topic_;
  std::string orbslam_odom_topic_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  bool publish_generic_odom_{true};
  bool publish_tf_{true};
  bool use_orbslam_odom_{true};
  double yaw_from_imu_weight_{0.85};
  double roll_from_imu_weight_{1.0};
  double pitch_from_imu_weight_{1.0};
  bool use_imu_angular_velocity_{true};
  bool force_planar_pose_{false};
  double orbslam_timeout_s_{0.5};
  double orbslam_position_weight_xy_{0.15};
  double orbslam_yaw_weight_{0.20};
  double orbslam_trans_gate_m_{0.40};
  double orbslam_yaw_gate_rad_{0.70};
  bool use_orbslam_linear_twist_{false};
  bool orbslam_use_relative_origin_{true};
  bool has_orbslam_origin_{false};
  double orbslam_origin_x_{0.0};
  double orbslam_origin_y_{0.0};
  double orbslam_origin_yaw_{0.0};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr kin_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr orbslam_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr generic_pub_;
  sensor_msgs::msg::Imu::SharedPtr latest_imu_;
  nav_msgs::msg::Odometry::SharedPtr latest_orbslam_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace hexapod_hardware_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hexapod_hardware_cpp::WeightedOdomFusionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
