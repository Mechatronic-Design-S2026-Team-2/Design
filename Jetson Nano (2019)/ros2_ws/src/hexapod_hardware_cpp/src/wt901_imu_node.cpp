#include "hexapod_hardware_cpp/serial_port.hpp"
#include "hexapod_hardware_cpp/wt901_protocol.hpp"
#include "hexapod_hardware_cpp/path_utils.hpp"

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <thread>
#include <vector>

namespace hexapod_hardware_cpp
{

class Wt901ImuNode : public rclcpp::Node
{
public:
  Wt901ImuNode()
  : Node("wt901_imu_node")
  {
    serial_device_ = declare_parameter<std::string>("serial_device", "/dev/ttyTHS1");
    baud_rate_ = declare_parameter<int>("baud_rate", 9600);
    use_cached_baud_ = declare_parameter<bool>("use_cached_baud", true);
    cache_file_ = declare_parameter<std::string>(
      "cache_file",
      (std::filesystem::path(workspace_root_from_package_prefix("hexapod_hardware_cpp")) / "wt901_serial_cache.yaml").string());
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    publish_euler_topic_ = declare_parameter<bool>("publish_euler_topic", true);

    accel_lpf_alpha_ = declare_parameter<double>("accel_lpf_alpha", 0.35);
    gyro_lpf_alpha_ = declare_parameter<double>("gyro_lpf_alpha", 0.35);
    angle_lpf_alpha_ = declare_parameter<double>("angle_lpf_alpha", 0.25);
    yaw_offset_rad_ = declare_parameter<double>("yaw_offset_rad", 0.0);
    xy_axes_quarter_turns_cw_ = declare_parameter<int>("xy_axes_quarter_turns_cw", 1);

    accel_bias_mps2_ = read_vector3_parameter("accel_bias_mps2", {0.0, 0.0, 0.0});
    gyro_bias_rps_ = read_vector3_parameter("gyro_bias_rps", {0.0, 0.0, 0.0});
    angle_bias_rad_ = read_vector3_parameter("angle_bias_rad", {0.0, 0.0, 0.0});

    accel_axis_sign_ = read_vector3_parameter("accel_axis_sign", {1.0, 1.0, 1.0});
    gyro_axis_sign_ = read_vector3_parameter("gyro_axis_sign", {1.0, 1.0, 1.0});
    angle_axis_sign_ = read_vector3_parameter("angle_axis_sign", {1.0, 1.0, 1.0});

    orientation_covariance_diag_ = read_vector3_parameter("orientation_covariance_diag", {0.02, 0.02, 0.04});
    angular_velocity_covariance_diag_ = read_vector3_parameter("angular_velocity_covariance_diag", {0.01, 0.01, 0.02});
    linear_acceleration_covariance_diag_ = read_vector3_parameter("linear_acceleration_covariance_diag", {0.1, 0.1, 0.15});

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 20);
    euler_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("imu/euler", 20);

    if (cache_file_.empty()) {
      cache_file_ = (std::filesystem::path(workspace_root_from_package_prefix("hexapod_hardware_cpp")) / "wt901_serial_cache.yaml").string();
    }
    if (use_cached_baud_) {
      const int cached_baud = read_cached_baud(cache_file_, baud_rate_);
      if (cached_baud != baud_rate_) {
        RCLCPP_INFO(get_logger(), "Using cached WT901 baud %d from %s", cached_baud, cache_file_.c_str());
        baud_rate_ = cached_baud;
      }
    }
    RCLCPP_INFO(get_logger(), "Opening WT901 on %s at %d baud", serial_device_.c_str(), baud_rate_);
    port_.open(serial_device_, baud_rate_);
    timer_ = create_wall_timer(std::chrono::milliseconds(2), std::bind(&Wt901ImuNode::poll, this));
  }

private:
  int read_cached_baud(const std::string & path, int fallback) const
  {
    try {
      if (!std::filesystem::exists(path)) {
        return fallback;
      }
      std::ifstream in(path);
      std::string key;
      while (in >> key) {
        if (key == "last_working_baud:" || key == "last_working_baud") {
          int value = fallback;
          if (in >> value) {
            return value;
          }
        }
      }
    } catch (...) {
    }
    return fallback;
  }

  std::array<double, 3> read_vector3_parameter(
    const std::string & name,
    const std::array<double, 3> & default_value)
  {
    const std::vector<double> values = declare_parameter<std::vector<double>>(
      name,
      std::vector<double>{default_value[0], default_value[1], default_value[2]});
    if (values.size() != 3U) {
      throw std::runtime_error("Parameter " + name + " must have exactly 3 elements");
    }
    return {values[0], values[1], values[2]};
  }

  static double lowpass(double previous, double current, double alpha)
  {
    return previous + alpha * (current - previous);
  }

  std::array<double, 2> rotate_xy_cw(const std::array<double, 2> & v) const
  {
    switch (((xy_axes_quarter_turns_cw_ % 4) + 4) % 4) {
      case 0: return v;
      case 1: return {v[1], -v[0]};
      case 2: return {-v[0], -v[1]};
      case 3: return {-v[1], v[0]};
      default: return v;
    }
  }

  void poll()
  {
    uint8_t buffer[256] = {0};
    const std::size_t count = port_.read_some(buffer, sizeof(buffer));
    for (std::size_t i = 0; i < count; ++i) {
      auto sample = parser_.push_byte(buffer[i]);
      if (sample.has_value()) {
        publish(sample.value());
      }
    }
  }

  void publish(const Wt901Sample & raw_sample)
  {
    std::array<double, 3> accel_raw{
      accel_axis_sign_[0] * raw_sample.ax_mps2 - accel_bias_mps2_[0],
      accel_axis_sign_[1] * raw_sample.ay_mps2 - accel_bias_mps2_[1],
      accel_axis_sign_[2] * raw_sample.az_mps2 - accel_bias_mps2_[2]};
    std::array<double, 3> gyro_raw{
      gyro_axis_sign_[0] * raw_sample.gx_rps - gyro_bias_rps_[0],
      gyro_axis_sign_[1] * raw_sample.gy_rps - gyro_bias_rps_[1],
      gyro_axis_sign_[2] * raw_sample.gz_rps - gyro_bias_rps_[2]};
    std::array<double, 3> angle_raw{
      angle_axis_sign_[0] * raw_sample.roll_rad - angle_bias_rad_[0],
      angle_axis_sign_[1] * raw_sample.pitch_rad - angle_bias_rad_[1],
      angle_axis_sign_[2] * raw_sample.yaw_rad - angle_bias_rad_[2] + yaw_offset_rad_};

    const auto accel_xy = rotate_xy_cw({accel_raw[0], accel_raw[1]});
    const auto gyro_xy = rotate_xy_cw({gyro_raw[0], gyro_raw[1]});
    const auto angle_xy = rotate_xy_cw({angle_raw[0], angle_raw[1]});

    std::array<double, 3> accel{accel_xy[0], accel_xy[1], accel_raw[2]};
    std::array<double, 3> gyro{gyro_xy[0], gyro_xy[1], gyro_raw[2]};
    std::array<double, 3> angle{angle_xy[0], angle_xy[1], angle_raw[2]};

    if (!have_filtered_) {
      filtered_accel_ = accel;
      filtered_gyro_ = gyro;
      filtered_angle_ = angle;
      have_filtered_ = true;
    } else {
      for (std::size_t i = 0; i < 3; ++i) {
        filtered_accel_[i] = lowpass(filtered_accel_[i], accel[i], accel_lpf_alpha_);
        filtered_gyro_[i] = lowpass(filtered_gyro_[i], gyro[i], gyro_lpf_alpha_);
        filtered_angle_[i] = lowpass(filtered_angle_[i], angle[i], angle_lpf_alpha_);
      }
    }

    const auto stamp = now();

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = frame_id_;
    imu.angular_velocity.x = filtered_gyro_[0];
    imu.angular_velocity.y = filtered_gyro_[1];
    imu.angular_velocity.z = filtered_gyro_[2];
    imu.linear_acceleration.x = filtered_accel_[0];
    imu.linear_acceleration.y = filtered_accel_[1];
    imu.linear_acceleration.z = filtered_accel_[2];

    const double cr = std::cos(filtered_angle_[0] * 0.5);
    const double sr = std::sin(filtered_angle_[0] * 0.5);
    const double cp = std::cos(filtered_angle_[1] * 0.5);
    const double sp = std::sin(filtered_angle_[1] * 0.5);
    const double cy = std::cos(filtered_angle_[2] * 0.5);
    const double sy = std::sin(filtered_angle_[2] * 0.5);

    imu.orientation.w = cr * cp * cy + sr * sp * sy;
    imu.orientation.x = sr * cp * cy - cr * sp * sy;
    imu.orientation.y = cr * sp * cy + sr * cp * sy;
    imu.orientation.z = cr * cp * sy - sr * sp * cy;

    imu.orientation_covariance[0] = orientation_covariance_diag_[0];
    imu.orientation_covariance[4] = orientation_covariance_diag_[1];
    imu.orientation_covariance[8] = orientation_covariance_diag_[2];
    imu.angular_velocity_covariance[0] = angular_velocity_covariance_diag_[0];
    imu.angular_velocity_covariance[4] = angular_velocity_covariance_diag_[1];
    imu.angular_velocity_covariance[8] = angular_velocity_covariance_diag_[2];
    imu.linear_acceleration_covariance[0] = linear_acceleration_covariance_diag_[0];
    imu.linear_acceleration_covariance[4] = linear_acceleration_covariance_diag_[1];
    imu.linear_acceleration_covariance[8] = linear_acceleration_covariance_diag_[2];

    imu_pub_->publish(imu);

    if (publish_euler_topic_) {
      geometry_msgs::msg::Vector3Stamped euler;
      euler.header = imu.header;
      euler.vector.x = filtered_angle_[0];
      euler.vector.y = filtered_angle_[1];
      euler.vector.z = filtered_angle_[2];
      euler_pub_->publish(euler);
    }
  }

  std::string serial_device_;
  int baud_rate_;
  bool use_cached_baud_{true};
  std::string cache_file_;
  std::string frame_id_;
  bool publish_euler_topic_{true};

  double accel_lpf_alpha_{0.35};
  double gyro_lpf_alpha_{0.35};
  double angle_lpf_alpha_{0.25};
  double yaw_offset_rad_{0.0};
  int xy_axes_quarter_turns_cw_{1};

  std::array<double, 3> accel_bias_mps2_{};
  std::array<double, 3> gyro_bias_rps_{};
  std::array<double, 3> angle_bias_rad_{};
  std::array<double, 3> accel_axis_sign_{};
  std::array<double, 3> gyro_axis_sign_{};
  std::array<double, 3> angle_axis_sign_{};
  std::array<double, 3> orientation_covariance_diag_{};
  std::array<double, 3> angular_velocity_covariance_diag_{};
  std::array<double, 3> linear_acceleration_covariance_diag_{};

  SerialPort port_;
  Wt901Parser parser_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool have_filtered_{false};
  std::array<double, 3> filtered_accel_{};
  std::array<double, 3> filtered_gyro_{};
  std::array<double, 3> filtered_angle_{};
};

}  // namespace hexapod_hardware_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hexapod_hardware_cpp::Wt901ImuNode>());
  rclcpp::shutdown();
  return 0;
}
