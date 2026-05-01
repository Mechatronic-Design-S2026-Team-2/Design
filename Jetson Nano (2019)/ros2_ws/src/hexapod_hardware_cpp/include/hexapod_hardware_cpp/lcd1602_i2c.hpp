#ifndef HEXAPOD_HARDWARE_CPP__LCD1602_I2C_HPP_
#define HEXAPOD_HARDWARE_CPP__LCD1602_I2C_HPP_

#include <cstdint>
#include <string>

namespace hexapod_hardware_cpp
{

class Lcd1602I2c
{
public:
  Lcd1602I2c();
  ~Lcd1602I2c();

  void open(const std::string & device, int address);
  void close();

  void initialize();
  void clear();
  void set_cursor(uint8_t row, uint8_t col);
  void print(const std::string & text);

private:
  int fd_;
  int address_;

  void write4(uint8_t nibble, bool rs);
  void pulse_enable(uint8_t data);
  void write_byte(uint8_t data);
  void send_command(uint8_t command);
  void send_data(uint8_t data);
};

}  // namespace hexapod_hardware_cpp

#endif  // HEXAPOD_HARDWARE_CPP__LCD1602_I2C_HPP_
