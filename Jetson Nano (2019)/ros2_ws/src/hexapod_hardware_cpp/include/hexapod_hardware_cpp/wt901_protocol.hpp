#ifndef HEXAPOD_HARDWARE_CPP__WT901_PROTOCOL_HPP_
#define HEXAPOD_HARDWARE_CPP__WT901_PROTOCOL_HPP_

#include <array>
#include <cstdint>
#include <deque>
#include <optional>

namespace hexapod_hardware_cpp
{

struct Wt901Sample
{
  double ax_mps2{0.0};
  double ay_mps2{0.0};
  double az_mps2{0.0};
  double gx_rps{0.0};
  double gy_rps{0.0};
  double gz_rps{0.0};
  double roll_rad{0.0};
  double pitch_rad{0.0};
  double yaw_rad{0.0};
  bool has_accel{false};
  bool has_gyro{false};
  bool has_angle{false};
};

class Wt901Parser
{
public:
  std::optional<Wt901Sample> push_byte(uint8_t byte);

private:
  std::deque<uint8_t> buffer_;
  Wt901Sample partial_;

  static bool checksum_ok(const std::array<uint8_t, 11> & frame);
  static int16_t unpack_i16(uint8_t lo, uint8_t hi);
};

}  // namespace hexapod_hardware_cpp

#endif  // HEXAPOD_HARDWARE_CPP__WT901_PROTOCOL_HPP_
