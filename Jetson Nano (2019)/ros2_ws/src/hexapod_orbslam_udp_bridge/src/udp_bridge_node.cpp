#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace hexapod_orbslam_udp_bridge
{
namespace
{
constexpr std::array<std::uint8_t, 4> k_pose_magic = {'O', 'S', '2', 'P'};
constexpr std::array<std::uint8_t, 4> k_scan_magic = {'O', 'S', '2', 'S'};
constexpr std::array<std::uint8_t, 4> k_map_magic = {'O', 'S', '2', 'M'};

constexpr std::size_t k_magic_offset = 0U;
constexpr std::size_t k_pose_packet_size = 48U;
constexpr std::size_t k_scan_header_size = 38U;
constexpr std::size_t k_map_header_size = 32U;
constexpr std::size_t k_float32_size = 4U;
constexpr std::size_t k_xyz_point_size = 12U;
constexpr std::size_t k_max_udp_packet_size = 65535U;
constexpr double k_quaternion_min_norm = 1.0e-12;
constexpr double k_lost_tracking_covariance = 1.0e3;
constexpr double k_default_tracked_xy_covariance = 0.05;
constexpr double k_default_tracked_z_covariance = 0.10;
constexpr double k_default_tracked_rpy_covariance = 0.25;
constexpr double k_default_scan_time_sec = 1.0 / 30.0;

struct Quaternion
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

struct Vector3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Matrix3
{
  double m[3][3]{};
};

struct PosePacket
{
  std::uint32_t seq{0U};
  double stamp_sec{0.0};
  Vector3 translation{};
  Quaternion orientation{};
  std::int32_t tracking_state{0};
};

struct ScanPacket
{
  std::uint32_t seq{0U};
  double stamp_sec{0.0};
  float angle_min{0.0F};
  float angle_max{0.0F};
  float angle_increment{0.0F};
  float range_min{0.0F};
  float range_max{0.0F};
  std::vector<float> ranges{};
};

struct MapChunkPacket
{
  std::uint32_t seq{0U};
  double stamp_sec{0.0};
  std::uint32_t map_seq{0U};
  std::uint32_t total_points{0U};
  std::uint16_t chunk_idx{0U};
  std::uint16_t chunk_count{0U};
  std::uint16_t point_count{0U};
  std::vector<std::uint8_t> payload{};
};

/** Check whether a UDP datagram starts with the requested 4-byte magic. */
bool has_magic(const std::vector<std::uint8_t> & data, const std::array<std::uint8_t, 4> & magic)
{
  if (data.size() < magic.size()) {
    return false;
  }
  return std::equal(magic.begin(), magic.end(), data.begin() + static_cast<std::ptrdiff_t>(k_magic_offset));
}

/** Read an unsigned 16-bit little-endian integer. */
std::uint16_t read_u16_le(const std::vector<std::uint8_t> & data, const std::size_t offset)
{
  return static_cast<std::uint16_t>(
    static_cast<std::uint16_t>(data[offset]) |
    static_cast<std::uint16_t>(data[offset + 1U] << 8U));
}

/** Read an unsigned 32-bit little-endian integer. */
std::uint32_t read_u32_le(const std::vector<std::uint8_t> & data, const std::size_t offset)
{
  return static_cast<std::uint32_t>(data[offset]) |
         (static_cast<std::uint32_t>(data[offset + 1U]) << 8U) |
         (static_cast<std::uint32_t>(data[offset + 2U]) << 16U) |
         (static_cast<std::uint32_t>(data[offset + 3U]) << 24U);
}

/** Read a signed 32-bit little-endian integer. */
std::int32_t read_i32_le(const std::vector<std::uint8_t> & data, const std::size_t offset)
{
  const std::uint32_t raw = read_u32_le(data, offset);
  std::int32_t value = 0;
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

/** Read an unsigned 64-bit little-endian integer. */
std::uint64_t read_u64_le(const std::vector<std::uint8_t> & data, const std::size_t offset)
{
  return static_cast<std::uint64_t>(data[offset]) |
         (static_cast<std::uint64_t>(data[offset + 1U]) << 8U) |
         (static_cast<std::uint64_t>(data[offset + 2U]) << 16U) |
         (static_cast<std::uint64_t>(data[offset + 3U]) << 24U) |
         (static_cast<std::uint64_t>(data[offset + 4U]) << 32U) |
         (static_cast<std::uint64_t>(data[offset + 5U]) << 40U) |
         (static_cast<std::uint64_t>(data[offset + 6U]) << 48U) |
         (static_cast<std::uint64_t>(data[offset + 7U]) << 56U);
}

/** Read a float32 from little-endian bytes. */
float read_f32_le(const std::vector<std::uint8_t> & data, const std::size_t offset)
{
  const std::uint32_t raw = read_u32_le(data, offset);
  float value = 0.0F;
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

/** Write a float32 as little-endian bytes. */
void write_f32_le(std::vector<std::uint8_t> * data, const std::size_t offset, const float value)
{
  if (data == nullptr || data->size() < offset + k_float32_size) {
    return;
  }

  std::uint32_t raw = 0U;
  std::memcpy(&raw, &value, sizeof(raw));
  (*data)[offset] = static_cast<std::uint8_t>(raw & 0xFFU);
  (*data)[offset + 1U] = static_cast<std::uint8_t>((raw >> 8U) & 0xFFU);
  (*data)[offset + 2U] = static_cast<std::uint8_t>((raw >> 16U) & 0xFFU);
  (*data)[offset + 3U] = static_cast<std::uint8_t>((raw >> 24U) & 0xFFU);
}

/** Read a float64 from little-endian bytes. */
double read_f64_le(const std::vector<std::uint8_t> & data, const std::size_t offset)
{
  const std::uint64_t raw = read_u64_le(data, offset);
  double value = 0.0;
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

/** Normalize quaternion, falling back to identity for invalid or degenerate values. */
Quaternion normalize_quaternion(const Quaternion & q)
{
  const double norm = std::sqrt((q.x * q.x) + (q.y * q.y) + (q.z * q.z) + (q.w * q.w));
  if (!std::isfinite(norm) || norm < k_quaternion_min_norm) {
    return Quaternion{};
  }
  return Quaternion{q.x / norm, q.y / norm, q.z / norm, q.w / norm};
}

/** Multiply quaternions in x/y/z/w order. */
Quaternion multiply_quaternion(const Quaternion & q1, const Quaternion & q2)
{
  return Quaternion{
    (q1.w * q2.x) + (q1.x * q2.w) + (q1.y * q2.z) - (q1.z * q2.y),
    (q1.w * q2.y) - (q1.x * q2.z) + (q1.y * q2.w) + (q1.z * q2.x),
    (q1.w * q2.z) + (q1.x * q2.y) - (q1.y * q2.x) + (q1.z * q2.w),
    (q1.w * q2.w) - (q1.x * q2.x) - (q1.y * q2.y) - (q1.z * q2.z)};
}

/** Convert quaternion to rotation matrix. */
Matrix3 rotation_matrix_from_quaternion(const Quaternion & q_in)
{
  const Quaternion q = normalize_quaternion(q_in);
  const double xx = q.x * q.x;
  const double yy = q.y * q.y;
  const double zz = q.z * q.z;
  const double xy = q.x * q.y;
  const double xz = q.x * q.z;
  const double yz = q.y * q.z;
  const double wx = q.w * q.x;
  const double wy = q.w * q.y;
  const double wz = q.w * q.z;

  Matrix3 matrix{};
  matrix.m[0][0] = 1.0 - (2.0 * (yy + zz));
  matrix.m[0][1] = 2.0 * (xy - wz);
  matrix.m[0][2] = 2.0 * (xz + wy);
  matrix.m[1][0] = 2.0 * (xy + wz);
  matrix.m[1][1] = 1.0 - (2.0 * (xx + zz));
  matrix.m[1][2] = 2.0 * (yz - wx);
  matrix.m[2][0] = 2.0 * (xz - wy);
  matrix.m[2][1] = 2.0 * (yz + wx);
  matrix.m[2][2] = 1.0 - (2.0 * (xx + yy));
  return matrix;
}

/** Multiply 3x3 matrix by a vector. */
Vector3 multiply_matrix_vector(const Matrix3 & matrix, const Vector3 & vector)
{
  return Vector3{
    (matrix.m[0][0] * vector.x) + (matrix.m[0][1] * vector.y) + (matrix.m[0][2] * vector.z),
    (matrix.m[1][0] * vector.x) + (matrix.m[1][1] * vector.y) + (matrix.m[1][2] * vector.z),
    (matrix.m[2][0] * vector.x) + (matrix.m[2][1] * vector.y) + (matrix.m[2][2] * vector.z)};
}

/** Transform optical-style axes into ROS body axes: +z forward -> +x forward, +x right -> -y left, +y down -> -z up. */
Vector3 optical_axes_to_ros_axes(const Vector3 & vector)
{
  return Vector3{vector.z, -vector.x, -vector.y};
}

/** Scale a vector by a scalar. */
Vector3 scale_vector(const Vector3 & vector, const double scale)
{
  return Vector3{vector.x * scale, vector.y * scale, vector.z * scale};
}

/** Return a finite replacement for invalid LaserScan ranges. */
float sanitize_scan_range(const float range, const float replacement)
{
  if (!std::isfinite(range) || range <= 0.0F) {
    return replacement;
  }
  return range;
}

/** Scale finite LaserScan ranges while preserving infinity/NaN placeholders. */
float scale_scan_range(const float range, const double scale)
{
  if (!std::isfinite(range)) {
    return range;
  }
  return static_cast<float>(static_cast<double>(range) * scale);
}

/** Parse OS2P pose datagram. */
bool parse_pose_packet(const std::vector<std::uint8_t> & data, PosePacket * packet)
{
  if ((packet == nullptr) || data.size() < k_pose_packet_size || !has_magic(data, k_pose_magic)) {
    return false;
  }

  packet->seq = read_u32_le(data, 4U);
  packet->stamp_sec = read_f64_le(data, 8U);
  packet->translation.x = static_cast<double>(read_f32_le(data, 16U));
  packet->translation.y = static_cast<double>(read_f32_le(data, 20U));
  packet->translation.z = static_cast<double>(read_f32_le(data, 24U));
  packet->orientation.x = static_cast<double>(read_f32_le(data, 28U));
  packet->orientation.y = static_cast<double>(read_f32_le(data, 32U));
  packet->orientation.z = static_cast<double>(read_f32_le(data, 36U));
  packet->orientation.w = static_cast<double>(read_f32_le(data, 40U));
  packet->tracking_state = read_i32_le(data, 44U);
  return true;
}

/** Parse OS2S scan datagram. */
bool parse_scan_packet(const std::vector<std::uint8_t> & data, ScanPacket * packet)
{
  if ((packet == nullptr) || data.size() < k_scan_header_size || !has_magic(data, k_scan_magic)) {
    return false;
  }

  packet->seq = read_u32_le(data, 4U);
  packet->stamp_sec = read_f64_le(data, 8U);
  packet->angle_min = read_f32_le(data, 16U);
  packet->angle_max = read_f32_le(data, 20U);
  packet->angle_increment = read_f32_le(data, 24U);
  packet->range_min = read_f32_le(data, 28U);
  packet->range_max = read_f32_le(data, 32U);

  const std::uint16_t count = read_u16_le(data, 36U);
  const std::size_t expected_size = k_scan_header_size + (static_cast<std::size_t>(count) * k_float32_size);
  if (data.size() < expected_size) {
    return false;
  }

  packet->ranges.clear();
  packet->ranges.reserve(count);
  for (std::uint16_t index = 0U; index < count; ++index) {
    packet->ranges.push_back(read_f32_le(data, k_scan_header_size + (static_cast<std::size_t>(index) * k_float32_size)));
  }
  return true;
}

/** Parse OS2M map chunk datagram. */
bool parse_map_chunk_packet(const std::vector<std::uint8_t> & data, MapChunkPacket * packet)
{
  if ((packet == nullptr) || data.size() < k_map_header_size || !has_magic(data, k_map_magic)) {
    return false;
  }

  packet->seq = read_u32_le(data, 4U);
  packet->stamp_sec = read_f64_le(data, 8U);
  packet->map_seq = read_u32_le(data, 16U);
  packet->total_points = read_u32_le(data, 20U);
  packet->chunk_idx = read_u16_le(data, 24U);
  packet->chunk_count = read_u16_le(data, 26U);
  packet->point_count = read_u16_le(data, 28U);

  const std::size_t payload_size = static_cast<std::size_t>(packet->point_count) * k_xyz_point_size;
  const std::size_t expected_size = k_map_header_size + payload_size;
  if (data.size() < expected_size) {
    return false;
  }

  packet->payload.assign(data.begin() + static_cast<std::ptrdiff_t>(k_map_header_size),
                         data.begin() + static_cast<std::ptrdiff_t>(expected_size));
  return true;
}

}  // namespace

class OrbslamUdpBridge final : public rclcpp::Node
{
public:
  explicit OrbslamUdpBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("orbslam_udp_bridge", options)
  {
    declare_parameters();
    load_parameters();
    create_ros_interfaces();
    open_udp_socket();

    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() { poll_udp_socket(); });

    RCLCPP_INFO(
      get_logger(),
      "Listening UDP %s:%d; publishing odom=%s (%s->%s), tf=%s, scan=%s:%s [%s], map=%s:%s [%s], axes_convert=%s, position_scale=%.6g, scan_range_scale=%.6g",
      udp_bind_ip_.c_str(), udp_port_, odom_topic_.c_str(), odom_frame_.c_str(), base_frame_.c_str(),
      publish_tf_ ? "true" : "false",
      publish_scan_ ? "true" : "false", scan_topic_.c_str(), scan_frame_.c_str(),
      publish_map_ ? "true" : "false", map_topic_.c_str(), map_frame_.c_str(),
      convert_orbslam_to_ros_axes_ ? "true" : "false", position_scale_, scan_range_scale_);
  }

  ~OrbslamUdpBridge() override
  {
    if (socket_fd_ >= 0) {
      ::close(socket_fd_);
      socket_fd_ = -1;
    }
  }

private:
  /** Declare node parameters with defaults matching the Python package. */
  void declare_parameters()
  {
    declare_parameter<std::string>("udp_bind_ip", "0.0.0.0");
    declare_parameter<int>("udp_port", 5005);

    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("odom_topic", "/odom");

    declare_parameter<std::string>("scan_topic", "/scan");
    declare_parameter<std::string>("scan_frame", "orbslam_scan_frame");
    declare_parameter<std::string>("map_topic", "/orbslam/map_points");
    declare_parameter<std::string>("map_frame", "odom");
    declare_parameter<bool>("map_points_apply_static_offset", false);
    declare_parameter<double>("map_points_offset_x", 0.0);
    declare_parameter<double>("map_points_offset_y", 0.0);
    declare_parameter<double>("map_points_offset_z", 0.0);

    declare_parameter<bool>("apply_optical_to_base", true);
    declare_parameter<bool>("convert_orbslam_to_ros_axes", true);
    declare_parameter<double>("position_scale", 1.0);
    declare_parameter<double>("scan_range_scale", 1.0);
    declare_parameter<std::vector<double>>("camopt_to_base_xyz", std::vector<double>{0.0, 0.0, -0.2794});
    declare_parameter<double>("camopt_to_base_x", 0.0);
    declare_parameter<double>("camopt_to_base_y", 0.0);
    declare_parameter<double>("camopt_to_base_z", -0.2794);

    declare_parameter<bool>("publish_odom", true);
    declare_parameter<bool>("publish_tf", true);
    declare_parameter<bool>("publish_scan", true);
    declare_parameter<bool>("publish_map", true);
    declare_parameter<bool>("use_packet_stamp", true);
    declare_parameter<bool>("publish_when_lost", false);
    declare_parameter<bool>("publish_scan_when_lost", false);
    declare_parameter<bool>("stamp_scan_with_latest_pose", true);
    declare_parameter<bool>("convert_scan_optical_to_ros", true);
    declare_parameter<bool>("invalid_scan_range_is_infinity", true);
    declare_parameter<double>("scan_time_sec", k_default_scan_time_sec);
    declare_parameter<int>("scan_publish_decimation", 1);
  }

  /** Read validated parameter values from the ROS parameter server. */
  void load_parameters()
  {
    udp_bind_ip_ = get_parameter("udp_bind_ip").as_string();
    udp_port_ = get_parameter("udp_port").as_int();

    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();

    scan_topic_ = get_parameter("scan_topic").as_string();
    scan_frame_ = get_parameter("scan_frame").as_string();
    map_topic_ = get_parameter("map_topic").as_string();
    map_frame_ = get_parameter("map_frame").as_string();
    map_points_apply_static_offset_ = get_parameter("map_points_apply_static_offset").as_bool();
    map_points_offset_xyz_.x = get_parameter("map_points_offset_x").as_double();
    map_points_offset_xyz_.y = get_parameter("map_points_offset_y").as_double();
    map_points_offset_xyz_.z = get_parameter("map_points_offset_z").as_double();

    apply_optical_to_base_ = get_parameter("apply_optical_to_base").as_bool();
    convert_orbslam_to_ros_axes_ = get_parameter("convert_orbslam_to_ros_axes").as_bool();
    position_scale_ = get_parameter("position_scale").as_double();
    scan_range_scale_ = get_parameter("scan_range_scale").as_double();
    publish_odom_ = get_parameter("publish_odom").as_bool();
    publish_tf_ = get_parameter("publish_tf").as_bool();
    publish_scan_ = get_parameter("publish_scan").as_bool();
    publish_map_ = get_parameter("publish_map").as_bool();
    use_packet_stamp_ = get_parameter("use_packet_stamp").as_bool();
    publish_when_lost_ = get_parameter("publish_when_lost").as_bool();
    publish_scan_when_lost_ = get_parameter("publish_scan_when_lost").as_bool();
    stamp_scan_with_latest_pose_ = get_parameter("stamp_scan_with_latest_pose").as_bool();
    convert_scan_optical_to_ros_ = get_parameter("convert_scan_optical_to_ros").as_bool();
    invalid_scan_range_is_infinity_ = get_parameter("invalid_scan_range_is_infinity").as_bool();
    scan_time_sec_ = get_parameter("scan_time_sec").as_double();
    scan_publish_decimation_ = get_parameter("scan_publish_decimation").as_int();

    const auto camopt_to_base_xyz = get_parameter("camopt_to_base_xyz").as_double_array();
    camopt_to_base_xyz_.x = camopt_to_base_xyz.size() > 0U ? camopt_to_base_xyz[0] : 0.0;
    camopt_to_base_xyz_.y = camopt_to_base_xyz.size() > 1U ? camopt_to_base_xyz[1] : 0.0;
    camopt_to_base_xyz_.z = camopt_to_base_xyz.size() > 2U ? camopt_to_base_xyz[2] : -0.2794;

    camopt_to_base_xyz_.x = get_parameter("camopt_to_base_x").as_double();
    camopt_to_base_xyz_.y = get_parameter("camopt_to_base_y").as_double();
    camopt_to_base_xyz_.z = get_parameter("camopt_to_base_z").as_double();

    if (udp_port_ <= 0 || udp_port_ > 65535) {
      throw std::runtime_error("udp_port must be in range 1..65535");
    }

    if (!std::isfinite(position_scale_) || position_scale_ <= 0.0) {
      throw std::runtime_error("position_scale must be finite and greater than zero");
    }

    if (!std::isfinite(scan_range_scale_) || scan_range_scale_ <= 0.0) {
      throw std::runtime_error("scan_range_scale must be finite and greater than zero");
    }

    if (!std::isfinite(scan_time_sec_) || scan_time_sec_ < 0.0) {
      throw std::runtime_error("scan_time_sec must be finite and non-negative");
    }

    if (scan_publish_decimation_ < 1) {
      throw std::runtime_error("scan_publish_decimation must be >= 1");
    }
  }

  /** Create ROS publishers and optional TF broadcaster. */
  void create_ros_interfaces()
  {
    if (publish_odom_) {
      odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(10));
    }

    if (publish_scan_) {
      scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, rclcpp::SensorDataQoS());
    }

    if (publish_map_) {
      map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(map_topic_, rclcpp::QoS(1));
    }

    if (publish_tf_) {
      tf_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
  }

  /** Open, configure, and bind the non-blocking UDP socket. */
  void open_udp_socket()
  {
    socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
      throw std::runtime_error(std::string("socket(AF_INET, SOCK_DGRAM) failed: ") + std::strerror(errno));
    }

    int reuse = 1;
    if (::setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
      throw std::runtime_error(std::string("setsockopt(SO_REUSEADDR) failed: ") + std::strerror(errno));
    }

    const int flags = ::fcntl(socket_fd_, F_GETFL, 0);
    if (flags < 0 || ::fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
      throw std::runtime_error(std::string("fcntl(O_NONBLOCK) failed: ") + std::strerror(errno));
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<std::uint16_t>(udp_port_));

    if (udp_bind_ip_ == "0.0.0.0" || udp_bind_ip_.empty()) {
      address.sin_addr.s_addr = htonl(INADDR_ANY);
    } else if (::inet_pton(AF_INET, udp_bind_ip_.c_str(), &address.sin_addr) != 1) {
      throw std::runtime_error("udp_bind_ip must be a valid IPv4 address");
    }

    if (::bind(socket_fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
      throw std::runtime_error(std::string("bind() failed: ") + std::strerror(errno));
    }
  }

  /** Convert packet timestamp or local ROS time into a message stamp. */
  builtin_interfaces::msg::Time stamp_from_seconds(const double stamp_sec) const
  {
    if (!use_packet_stamp_ || !std::isfinite(stamp_sec) || stamp_sec < 0.0) {
      const auto now_nsec = now().nanoseconds();
      builtin_interfaces::msg::Time stamp;
      stamp.sec = static_cast<std::int32_t>(now_nsec / 1000000000LL);
      stamp.nanosec = static_cast<std::uint32_t>(now_nsec % 1000000000LL);
      return stamp;
    }

    double whole_seconds = 0.0;
    double fractional_seconds = std::modf(stamp_sec, &whole_seconds);
    auto sec = static_cast<std::int32_t>(whole_seconds);
    auto nanosec = static_cast<std::uint32_t>(std::llround(fractional_seconds * 1.0e9));

    if (nanosec >= 1000000000U) {
      ++sec;
      nanosec -= 1000000000U;
    }

    builtin_interfaces::msg::Time stamp;
    stamp.sec = sec;
    stamp.nanosec = nanosec;
    return stamp;
  }

  /** Convert a raw ORB-SLAM position/map point into ROS odom axes and meters. */
  Vector3 position_from_orbslam(const Vector3 & raw_position) const
  {
    const Vector3 axis_corrected = convert_orbslam_to_ros_axes_ ? optical_axes_to_ros_axes(raw_position) : raw_position;
    return scale_vector(axis_corrected, position_scale_);
  }

  /** Convert a raw ORB-SLAM camera-optical orientation into ROS odom axes. */
  Quaternion camera_orientation_from_orbslam(const Quaternion & raw_orientation) const
  {
    if (!convert_orbslam_to_ros_axes_) {
      return raw_orientation;
    }

    return normalize_quaternion(multiply_quaternion(q_orbslam_world_to_ros_world_, raw_orientation));
  }

  /** Publish TF and Odometry from one pose packet. */
  void publish_pose(const PosePacket & packet)
  {
    (void)packet.seq;

    const Quaternion q_cam_raw = normalize_quaternion(packet.orientation);
    last_pose_tracking_ok_ = packet.tracking_state != 0;
    if (!last_pose_tracking_ok_ && !publish_when_lost_) {
      return;
    }

    const auto stamp = stamp_from_seconds(packet.stamp_sec);
    last_pose_stamp_ = stamp;
    has_last_pose_stamp_ = true;

    const Quaternion q_cam = camera_orientation_from_orbslam(q_cam_raw);
    const Vector3 t_cam = position_from_orbslam(packet.translation);
    Quaternion q_base{};
    Vector3 t_base{};

    if (apply_optical_to_base_) {
      q_base = normalize_quaternion(multiply_quaternion(q_cam, q_camopt_base_));
      const Matrix3 r_odom_cam = rotation_matrix_from_quaternion(q_cam);
      const Vector3 offset_in_odom = multiply_matrix_vector(r_odom_cam, camopt_to_base_xyz_);
      t_base = Vector3{
        t_cam.x + offset_in_odom.x,
        t_cam.y + offset_in_odom.y,
        t_cam.z + offset_in_odom.z};
    } else {
      q_base = q_cam;
      t_base = t_cam;
    }

    if (tf_pub_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = stamp;
      transform.header.frame_id = odom_frame_;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = t_base.x;
      transform.transform.translation.y = t_base.y;
      transform.transform.translation.z = t_base.z;
      transform.transform.rotation.x = q_base.x;
      transform.transform.rotation.y = q_base.y;
      transform.transform.rotation.z = q_base.z;
      transform.transform.rotation.w = q_base.w;
      tf_pub_->sendTransform(transform);
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = t_base.x;
    odom.pose.pose.position.y = t_base.y;
    odom.pose.pose.position.z = t_base.z;
    odom.pose.pose.orientation.x = q_base.x;
    odom.pose.pose.orientation.y = q_base.y;
    odom.pose.pose.orientation.z = q_base.z;
    odom.pose.pose.orientation.w = q_base.w;
    odom.pose.covariance.fill(0.0);
    odom.pose.covariance[0] = k_default_tracked_xy_covariance;
    odom.pose.covariance[7] = k_default_tracked_xy_covariance;
    odom.pose.covariance[14] = k_default_tracked_z_covariance;
    odom.pose.covariance[21] = k_default_tracked_rpy_covariance;
    odom.pose.covariance[28] = k_default_tracked_rpy_covariance;
    odom.pose.covariance[35] = k_default_tracked_rpy_covariance;

    if (packet.tracking_state == 0) {
      odom.pose.covariance.fill(0.0);
      odom.pose.covariance[0] = k_lost_tracking_covariance;
      odom.pose.covariance[7] = k_lost_tracking_covariance;
      odom.pose.covariance[14] = k_lost_tracking_covariance;
      odom.pose.covariance[21] = k_lost_tracking_covariance;
      odom.pose.covariance[28] = k_lost_tracking_covariance;
      odom.pose.covariance[35] = k_lost_tracking_covariance;
    }

    if (odom_pub_) {
      odom_pub_->publish(odom);
    }
  }

  /** Publish LaserScan from one scan packet. */
  void publish_scan(const ScanPacket & packet)
  {
    (void)packet.seq;

    if (!scan_pub_) {
      return;
    }

    if (!publish_scan_when_lost_ && !last_pose_tracking_ok_) {
      return;
    }

    ++scan_packet_count_;
    if (scan_publish_decimation_ > 1 &&
        ((scan_packet_count_ - 1U) % static_cast<std::uint64_t>(scan_publish_decimation_)) != 0U) {
      return;
    }

    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = (stamp_scan_with_latest_pose_ && has_last_pose_stamp_) ?
      last_pose_stamp_ : stamp_from_seconds(packet.stamp_sec);
    scan.header.frame_id = scan_frame_;

    scan.range_min = static_cast<float>(static_cast<double>(packet.range_min) * scan_range_scale_);
    scan.range_max = static_cast<float>(static_cast<double>(packet.range_max) * scan_range_scale_);
    const float invalid_replacement = invalid_scan_range_is_infinity_ ?
      std::numeric_limits<float>::infinity() :
      std::numeric_limits<float>::quiet_NaN();

    if (convert_scan_optical_to_ros_) {
      scan.angle_min = -packet.angle_max;
      scan.angle_max = -packet.angle_min;
      scan.angle_increment = std::fabs(packet.angle_increment);
      scan.ranges.resize(packet.ranges.size());
      for (std::size_t index = 0U; index < packet.ranges.size(); ++index) {
        scan.ranges[index] = scale_scan_range(
          sanitize_scan_range(packet.ranges[packet.ranges.size() - 1U - index], invalid_replacement),
          scan_range_scale_);
      }
    } else {
      scan.angle_min = packet.angle_min;
      scan.angle_max = packet.angle_max;
      scan.angle_increment = packet.angle_increment;
      scan.ranges.resize(packet.ranges.size());
      for (std::size_t index = 0U; index < packet.ranges.size(); ++index) {
        scan.ranges[index] = scale_scan_range(
          sanitize_scan_range(packet.ranges[index], invalid_replacement), scan_range_scale_);
      }
    }

    scan.scan_time = static_cast<float>(scan_time_sec_);
    scan.time_increment = scan.ranges.empty() ? 0.0F :
      static_cast<float>(scan_time_sec_ / static_cast<double>(scan.ranges.size()));
    scan.intensities.clear();
    scan_pub_->publish(scan);
  }

  /** Clear all in-progress map chunk assembly state. */
  void reset_map_assembly()
  {
    map_seq_ = 0U;
    map_stamp_sec_ = 0.0;
    map_total_points_ = 0U;
    map_expected_chunks_ = 0U;
    map_chunks_.clear();
    has_active_map_ = false;
  }

  /** Store one map chunk and publish a complete PointCloud2 when all chunks arrive. */
  void consume_map_chunk(const MapChunkPacket & packet)
  {
    if (!map_pub_) {
      return;
    }

    if (!has_active_map_ || map_seq_ != packet.map_seq || map_expected_chunks_ != packet.chunk_count) {
      reset_map_assembly();
      map_seq_ = packet.map_seq;
      map_stamp_sec_ = packet.stamp_sec;
      map_total_points_ = packet.total_points;
      map_expected_chunks_ = packet.chunk_count;
      has_active_map_ = true;
    }

    if (packet.chunk_idx >= map_expected_chunks_) {
      return;
    }

    if (map_chunks_.find(packet.chunk_idx) == map_chunks_.end()) {
      map_chunks_[packet.chunk_idx] = packet.payload;
    }

    if (map_chunks_.size() != map_expected_chunks_) {
      return;
    }

    std::vector<std::uint8_t> cloud_bytes;
    cloud_bytes.reserve(static_cast<std::size_t>(map_total_points_) * k_xyz_point_size);
    for (std::uint16_t chunk_idx = 0U; chunk_idx < map_expected_chunks_; ++chunk_idx) {
      const auto chunk_iter = map_chunks_.find(chunk_idx);
      if (chunk_iter == map_chunks_.end()) {
        return;
      }
      cloud_bytes.insert(cloud_bytes.end(), chunk_iter->second.begin(), chunk_iter->second.end());
    }

    const std::size_t expected_total_bytes = static_cast<std::size_t>(map_total_points_) * k_xyz_point_size;
    if (cloud_bytes.size() > expected_total_bytes) {
      cloud_bytes.resize(expected_total_bytes);
    }

    const auto point_count_final = static_cast<std::uint32_t>(cloud_bytes.size() / k_xyz_point_size);

    std::vector<std::uint8_t> transformed_cloud_bytes;
    transformed_cloud_bytes.resize(static_cast<std::size_t>(point_count_final) * k_xyz_point_size);

    for (std::uint32_t point_idx = 0U; point_idx < point_count_final; ++point_idx) {
      const std::size_t offset = static_cast<std::size_t>(point_idx) * k_xyz_point_size;
      const Vector3 raw_point{
        static_cast<double>(read_f32_le(cloud_bytes, offset)),
        static_cast<double>(read_f32_le(cloud_bytes, offset + 4U)),
        static_cast<double>(read_f32_le(cloud_bytes, offset + 8U))};
      Vector3 output_point = position_from_orbslam(raw_point);
      if (map_points_apply_static_offset_) {
        output_point.x += map_points_offset_xyz_.x;
        output_point.y += map_points_offset_xyz_.y;
        output_point.z += map_points_offset_xyz_.z;
      }
      write_f32_le(&transformed_cloud_bytes, offset, static_cast<float>(output_point.x));
      write_f32_le(&transformed_cloud_bytes, offset + 4U, static_cast<float>(output_point.y));
      write_f32_le(&transformed_cloud_bytes, offset + 8U, static_cast<float>(output_point.z));
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp_from_seconds(map_stamp_sec_);
    cloud.header.frame_id = map_frame_;
    cloud.height = 1U;
    cloud.width = point_count_final;
    cloud.fields.resize(3U);

    cloud.fields[0].name = "x";
    cloud.fields[0].offset = 0U;
    cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[0].count = 1U;

    cloud.fields[1].name = "y";
    cloud.fields[1].offset = 4U;
    cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[1].count = 1U;

    cloud.fields[2].name = "z";
    cloud.fields[2].offset = 8U;
    cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[2].count = 1U;

    cloud.is_bigendian = false;
    cloud.point_step = static_cast<std::uint32_t>(k_xyz_point_size);
    cloud.row_step = point_count_final * cloud.point_step;
    cloud.is_dense = false;
    cloud.data = std::move(transformed_cloud_bytes);
    map_pub_->publish(cloud);

    reset_map_assembly();
  }

  /** Poll all queued UDP datagrams, consuming all map chunks and publishing only the latest pose/scan per timer tick. */
  void poll_udp_socket()
  {
    if (socket_fd_ < 0) {
      return;
    }

    std::vector<std::uint8_t> buffer(k_max_udp_packet_size);
    bool has_last_pose = false;
    bool has_last_scan = false;
    PosePacket last_pose{};
    ScanPacket last_scan{};

    while (rclcpp::ok()) {
      sockaddr_in sender_address{};
      socklen_t sender_address_len = sizeof(sender_address);
      const ssize_t bytes_received = ::recvfrom(
        socket_fd_, buffer.data(), buffer.size(), 0,
        reinterpret_cast<sockaddr *>(&sender_address), &sender_address_len);

      if (bytes_received < 0) {
        if (errno != EWOULDBLOCK && errno != EAGAIN) {
          RCLCPP_WARN(get_logger(), "UDP recvfrom() failed: %s", std::strerror(errno));
        }
        break;
      }

      if (bytes_received < 4) {
        continue;
      }

      std::vector<std::uint8_t> data(buffer.begin(), buffer.begin() + bytes_received);

      if (has_magic(data, k_pose_magic)) {
        PosePacket packet{};
        if (parse_pose_packet(data, &packet)) {
          last_pose = packet;
          has_last_pose = true;
        }
        continue;
      }

      if (has_magic(data, k_scan_magic)) {
        ScanPacket packet{};
        if (parse_scan_packet(data, &packet)) {
          last_scan = std::move(packet);
          has_last_scan = true;
        }
        continue;
      }

      if (has_magic(data, k_map_magic)) {
        MapChunkPacket packet{};
        if (parse_map_chunk_packet(data, &packet)) {
          consume_map_chunk(packet);
        }
        continue;
      }
    }

    if (has_last_pose) {
      publish_pose(last_pose);
    }

    if (has_last_scan) {
      publish_scan(last_scan);
    }
  }

  std::string udp_bind_ip_{"0.0.0.0"};
  int udp_port_{5005};

  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  std::string odom_topic_{"/odom"};
  std::string scan_topic_{"/scan"};
  std::string scan_frame_{"orbslam_scan_frame"};
  std::string map_topic_{"/orbslam/map_points"};
  std::string map_frame_{"odom"};

  bool apply_optical_to_base_{true};
  bool convert_orbslam_to_ros_axes_{true};
  double position_scale_{1.0};
  double scan_range_scale_{1.0};
  bool publish_odom_{true};
  bool publish_tf_{true};
  bool publish_scan_{true};
  bool publish_map_{true};
  bool use_packet_stamp_{true};
  bool publish_when_lost_{false};
  bool publish_scan_when_lost_{false};
  bool stamp_scan_with_latest_pose_{true};
  bool convert_scan_optical_to_ros_{true};
  bool invalid_scan_range_is_infinity_{true};
  double scan_time_sec_{k_default_scan_time_sec};
  int scan_publish_decimation_{1};
  std::uint64_t scan_packet_count_{0U};

  Vector3 camopt_to_base_xyz_{0.0, 0.0, -0.2794};
  bool map_points_apply_static_offset_{false};
  Vector3 map_points_offset_xyz_{0.0, 0.0, 0.0};
  const Quaternion q_camopt_base_{0.5, -0.5, 0.5, 0.5};
  const Quaternion q_orbslam_world_to_ros_world_{-0.5, 0.5, -0.5, 0.5};

  bool has_last_pose_stamp_{false};
  bool last_pose_tracking_ok_{false};
  builtin_interfaces::msg::Time last_pose_stamp_{};

  int socket_fd_{-1};
  rclcpp::TimerBase::SharedPtr timer_{};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_{};
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_{};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_{};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_pub_{};

  bool has_active_map_{false};
  std::uint32_t map_seq_{0U};
  double map_stamp_sec_{0.0};
  std::uint32_t map_total_points_{0U};
  std::uint16_t map_expected_chunks_{0U};
  std::map<std::uint16_t, std::vector<std::uint8_t>> map_chunks_{};
};

}  // namespace hexapod_orbslam_udp_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<hexapod_orbslam_udp_bridge::OrbslamUdpBridge>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("orbslam_udp_bridge"), "%s", exception.what());
  }
  rclcpp::shutdown();
  return 0;
}
