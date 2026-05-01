#include "hexapod_hardware_cpp/serial_port.hpp"

#include <fcntl.h>
#include <stdexcept>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <sstream>

namespace hexapod_hardware_cpp
{

namespace
{
speed_t to_speed_t(int baud_rate)
{
  switch (baud_rate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: throw std::runtime_error("Unsupported baud rate");
  }
}
}  // namespace

SerialPort::SerialPort()
: fd_(-1)
{
}

SerialPort::~SerialPort()
{
  close();
}

void SerialPort::open(const std::string & device, int baud_rate)
{
  close();

  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    throw std::runtime_error("Failed to open serial device " + device);
  }

  termios tty {};
  if (tcgetattr(fd_, &tty) != 0) {
    close();
    throw std::runtime_error("tcgetattr failed");
  }

  cfmakeraw(&tty);
  const speed_t speed = to_speed_t(baud_rate);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    close();
    throw std::runtime_error("tcsetattr failed");
  }
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::is_open() const
{
  return fd_ >= 0;
}

std::size_t SerialPort::read_some(uint8_t * buffer, std::size_t max_bytes)
{
  if (fd_ < 0) {
    return 0;
  }
  const ssize_t count = ::read(fd_, buffer, max_bytes);
  if (count <= 0) {
    return 0;
  }
  return static_cast<std::size_t>(count);
}

std::size_t SerialPort::write_bytes(const uint8_t * buffer, std::size_t size)
{
  if (fd_ < 0) {
    return 0;
  }
  const ssize_t count = ::write(fd_, buffer, size);
  if (count <= 0) {
    return 0;
  }
  return static_cast<std::size_t>(count);
}

}  // namespace hexapod_hardware_cpp
