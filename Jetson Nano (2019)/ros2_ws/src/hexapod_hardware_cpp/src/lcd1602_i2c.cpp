#include "hexapod_hardware_cpp/lcd1602_i2c.hpp"

#include <chrono>
#include <stdexcept>
#include <string>
#include <thread>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace hexapod_hardware_cpp
{

namespace
{
constexpr uint8_t kBacklight = 0x08;
constexpr uint8_t kEnable = 0x04;
}  // namespace

Lcd1602I2c::Lcd1602I2c()
: fd_(-1), address_(0x27)
{
}

Lcd1602I2c::~Lcd1602I2c()
{
  close();
}

void Lcd1602I2c::open(const std::string & device, int address)
{
  close();
  fd_ = ::open(device.c_str(), O_RDWR);
  if (fd_ < 0) {
    throw std::runtime_error("Failed to open I2C device " + device);
  }
  address_ = address;
  if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
    close();
    throw std::runtime_error("Failed to select I2C slave address");
  }
}

void Lcd1602I2c::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void Lcd1602I2c::write_byte(uint8_t data)
{
  if (fd_ < 0) {
    return;
  }
  ::write(fd_, &data, 1);
}

void Lcd1602I2c::pulse_enable(uint8_t data)
{
  write_byte(data | kEnable);
  std::this_thread::sleep_for(std::chrono::microseconds(1));
  write_byte(data & ~kEnable);
  std::this_thread::sleep_for(std::chrono::microseconds(50));
}

void Lcd1602I2c::write4(uint8_t nibble, bool rs)
{
  uint8_t data = static_cast<uint8_t>((nibble & 0xF0) | kBacklight | (rs ? 0x01 : 0x00));
  write_byte(data);
  pulse_enable(data);
}

void Lcd1602I2c::send_command(uint8_t command)
{
  write4(command & 0xF0, false);
  write4(static_cast<uint8_t>((command << 4) & 0xF0), false);
}

void Lcd1602I2c::send_data(uint8_t data)
{
  write4(data & 0xF0, true);
  write4(static_cast<uint8_t>((data << 4) & 0xF0), true);
}

void Lcd1602I2c::initialize()
{
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  write4(0x30, false);
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  write4(0x30, false);
  std::this_thread::sleep_for(std::chrono::microseconds(150));
  write4(0x30, false);
  write4(0x20, false);

  send_command(0x28);
  send_command(0x0C);
  send_command(0x06);
  send_command(0x01);
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
}

void Lcd1602I2c::clear()
{
  send_command(0x01);
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
}

void Lcd1602I2c::set_cursor(uint8_t row, uint8_t col)
{
  static const uint8_t row_offsets[2] = {0x00, 0x40};
  send_command(static_cast<uint8_t>(0x80 | (row_offsets[row % 2] + col)));
}

void Lcd1602I2c::print(const std::string & text)
{
  for (char ch : text) {
    send_data(static_cast<uint8_t>(ch));
  }
}

}  // namespace hexapod_hardware_cpp
