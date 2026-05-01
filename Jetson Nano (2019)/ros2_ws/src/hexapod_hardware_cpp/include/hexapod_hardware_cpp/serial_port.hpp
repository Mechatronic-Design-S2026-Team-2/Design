#ifndef HEXAPOD_HARDWARE_CPP__SERIAL_PORT_HPP_
#define HEXAPOD_HARDWARE_CPP__SERIAL_PORT_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace hexapod_hardware_cpp
{

class SerialPort
{
public:
  SerialPort();
  ~SerialPort();

  void open(const std::string & device, int baud_rate);
  void close();
  bool is_open() const;

  std::size_t read_some(uint8_t * buffer, std::size_t max_bytes);
  std::size_t write_bytes(const uint8_t * buffer, std::size_t size);

private:
  int fd_;
};

}  // namespace hexapod_hardware_cpp

#endif  // HEXAPOD_HARDWARE_CPP__SERIAL_PORT_HPP_
