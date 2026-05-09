#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <string>

namespace hexapod_hardware_cpp
{

class ImuUdpExportNode : public rclcpp::Node
{
public:
  ImuUdpExportNode()
  : Node("imu_udp_export_node")
  {
    enabled_ = declare_parameter<bool>("enabled", true);
    input_topic_ = declare_parameter<std::string>("input_topic", "/imu/data_raw");
    target_ip_ = declare_parameter<std::string>("target_ip", "127.0.0.1");
    target_port_ = declare_parameter<int>("target_port", 5015);
    use_header_stamp_ = declare_parameter<bool>("use_header_stamp", true);
    include_orientation_ = declare_parameter<bool>("include_orientation", true);
    include_frame_id_ = declare_parameter<bool>("include_frame_id", true);
    min_publish_period_s_ = declare_parameter<double>("min_publish_period_s", 0.0);
    print_every_n_packets_ = declare_parameter<int>("print_every_n_packets", 250);

    if (!enabled_) {
      RCLCPP_INFO(get_logger(), "IMU UDP export disabled");
      return;
    }

    open_socket();
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&ImuUdpExportNode::on_imu, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Exporting %s to udp://%s:%d as HXIMU3 CSV v1",
      input_topic_.c_str(),
      target_ip_.c_str(),
      target_port_);
  }

  ~ImuUdpExportNode() override
  {
    if (socket_fd_ >= 0) {
      close(socket_fd_);
      socket_fd_ = -1;
    }
  }

private:
  static std::int64_t stamp_to_ns(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
      static_cast<std::int64_t>(stamp.nanosec);
  }

  static std::string sanitized_frame_id(std::string frame_id)
  {
    for (char & c : frame_id) {
      if (c == ',' || c == '\n' || c == '\r' || c == '\t' || c == ' ') {
        c = '_';
      }
    }
    if (frame_id.empty()) {
      frame_id = "imu_link";
    }
    return frame_id;
  }

  void open_socket()
  {
    if (target_port_ <= 0 || target_port_ > 65535) {
      throw std::runtime_error("target_port must be in [1, 65535]");
    }

    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
      throw std::runtime_error(std::string("socket(AF_INET, SOCK_DGRAM) failed: ") + std::strerror(errno));
    }

    std::memset(&target_addr_, 0, sizeof(target_addr_));
    target_addr_.sin_family = AF_INET;
    target_addr_.sin_port = htons(static_cast<uint16_t>(target_port_));
    const int ok = inet_pton(AF_INET, target_ip_.c_str(), &target_addr_.sin_addr);
    if (ok != 1) {
      close(socket_fd_);
      socket_fd_ = -1;
      throw std::runtime_error("target_ip must be an IPv4 address, got: " + target_ip_);
    }
  }

  bool should_send_now(const rclcpp::Time & now)
  {
    if (min_publish_period_s_ <= 0.0 || !last_send_time_.nanoseconds()) {
      last_send_time_ = now;
      return true;
    }
    const double dt_s = (now - last_send_time_).seconds();
    if (dt_s < min_publish_period_s_) {
      return false;
    }
    last_send_time_ = now;
    return true;
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const auto recv_time = now();
    if (!should_send_now(recv_time)) {
      return;
    }

    const std::int64_t header_stamp_ns = stamp_to_ns(msg->header.stamp);
    const std::int64_t stamp_ns = use_header_stamp_ && header_stamp_ns > 0 ?
      header_stamp_ns : recv_time.nanoseconds();
    const std::int64_t recv_stamp_ns = recv_time.nanoseconds();

    std::ostringstream out;
    out << std::setprecision(17);
    out << "HXIMU3,1,";
    out << stamp_ns << ',';
    out << recv_stamp_ns << ',';
    out << sequence_++ << ',';
    if (include_frame_id_) {
      out << sanitized_frame_id(msg->header.frame_id) << ',';
    }
    out << msg->angular_velocity.x << ',';
    out << msg->angular_velocity.y << ',';
    out << msg->angular_velocity.z << ',';
    out << msg->linear_acceleration.x << ',';
    out << msg->linear_acceleration.y << ',';
    out << msg->linear_acceleration.z;
    if (include_orientation_) {
      out << ',' << msg->orientation.x;
      out << ',' << msg->orientation.y;
      out << ',' << msg->orientation.z;
      out << ',' << msg->orientation.w;
    }
    out << '\n';

    const std::string packet = out.str();
    const ssize_t sent = sendto(
      socket_fd_,
      packet.data(),
      packet.size(),
      0,
      reinterpret_cast<const sockaddr *>(&target_addr_),
      sizeof(target_addr_));

    if (sent < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "IMU UDP sendto failed: %s",
        std::strerror(errno));
      return;
    }

    ++sent_packets_;
    if (print_every_n_packets_ > 0 && (sent_packets_ % static_cast<std::uint64_t>(print_every_n_packets_)) == 0U) {
      RCLCPP_INFO(
        get_logger(),
        "Exported %lu IMU UDP packets to %s:%d",
        static_cast<unsigned long>(sent_packets_),
        target_ip_.c_str(),
        target_port_);
    }
  }

  bool enabled_{true};
  std::string input_topic_;
  std::string target_ip_;
  int target_port_{5015};
  bool use_header_stamp_{true};
  bool include_orientation_{true};
  bool include_frame_id_{true};
  double min_publish_period_s_{0.0};
  int print_every_n_packets_{250};

  int socket_fd_{-1};
  sockaddr_in target_addr_{};
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Time last_send_time_{0, 0, RCL_ROS_TIME};
  std::uint64_t sequence_{0};
  std::uint64_t sent_packets_{0};
};

}  // namespace hexapod_hardware_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hexapod_hardware_cpp::ImuUdpExportNode>());
  rclcpp::shutdown();
  return 0;
}
