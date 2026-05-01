#include "hexapod_hardware_cpp/wt901_protocol.hpp"

#include <cmath>

namespace hexapod_hardware_cpp
{

namespace
{
constexpr double kGravity = 9.80665;
constexpr double kPi = 3.14159265358979323846;
}  // namespace

bool Wt901Parser::checksum_ok(const std::array<uint8_t, 11> & frame)
{
  uint16_t sum = 0;
  for (std::size_t i = 0; i < 10; ++i) {
    sum += frame[i];
  }
  return static_cast<uint8_t>(sum & 0xFF) == frame[10];
}

int16_t Wt901Parser::unpack_i16(uint8_t lo, uint8_t hi)
{
  return static_cast<int16_t>(static_cast<uint16_t>(hi) << 8U | static_cast<uint16_t>(lo));
}

std::optional<Wt901Sample> Wt901Parser::push_byte(uint8_t byte)
{
  buffer_.push_back(byte);
  while (buffer_.size() >= 11) {
    if (buffer_[0] != 0x55) {
      buffer_.pop_front();
      continue;
    }

    std::array<uint8_t, 11> frame {};
    for (std::size_t i = 0; i < 11; ++i) {
      frame[i] = buffer_[i];
    }

    if (!checksum_ok(frame)) {
      buffer_.pop_front();
      continue;
    }

    buffer_.erase(buffer_.begin(), buffer_.begin() + 11);

    const uint8_t type = frame[1];
    if (type == 0x51) {
      partial_.ax_mps2 = static_cast<double>(unpack_i16(frame[2], frame[3])) / 32768.0 * 16.0 * kGravity;
      partial_.ay_mps2 = static_cast<double>(unpack_i16(frame[4], frame[5])) / 32768.0 * 16.0 * kGravity;
      partial_.az_mps2 = static_cast<double>(unpack_i16(frame[6], frame[7])) / 32768.0 * 16.0 * kGravity;
      partial_.has_accel = true;
    } else if (type == 0x52) {
      const double deg_to_rad = kPi / 180.0;
      partial_.gx_rps = static_cast<double>(unpack_i16(frame[2], frame[3])) / 32768.0 * 2000.0 * deg_to_rad;
      partial_.gy_rps = static_cast<double>(unpack_i16(frame[4], frame[5])) / 32768.0 * 2000.0 * deg_to_rad;
      partial_.gz_rps = static_cast<double>(unpack_i16(frame[6], frame[7])) / 32768.0 * 2000.0 * deg_to_rad;
      partial_.has_gyro = true;
    } else if (type == 0x53) {
      const double deg_to_rad = kPi / 180.0;
      partial_.roll_rad = static_cast<double>(unpack_i16(frame[2], frame[3])) / 32768.0 * 180.0 * deg_to_rad;
      partial_.pitch_rad = static_cast<double>(unpack_i16(frame[4], frame[5])) / 32768.0 * 180.0 * deg_to_rad;
      partial_.yaw_rad = static_cast<double>(unpack_i16(frame[6], frame[7])) / 32768.0 * 180.0 * deg_to_rad;
      partial_.has_angle = true;
    }

    if (partial_.has_accel && partial_.has_gyro && partial_.has_angle) {
      Wt901Sample out = partial_;
      partial_ = Wt901Sample {};
      return out;
    }
  }

  return std::nullopt;
}

}  // namespace hexapod_hardware_cpp
