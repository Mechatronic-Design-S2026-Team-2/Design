#include "hexapod_hardware_cpp/serial_port.hpp"
#include "hexapod_hardware_cpp/wt901_protocol.hpp"
#include "hexapod_hardware_cpp/path_utils.hpp"

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;
using namespace std::chrono_literals;

namespace
{
constexpr char kPackageName[] = "hexapod_hardware_cpp";
constexpr std::array<int, 8> kSupportedBauds = {9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};

struct Options
{
  std::string device{"/dev/ttyTHS1"};
  std::string cache_file;
  std::optional<int> target_baud;
  std::optional<int> target_rate_hz;
  bool detect_only{false};
  bool highest_test_preset{false};
  bool practical_high_preset{false};
  std::optional<int> current_baud;
  int detect_timeout_ms{1200};
};

std::string default_cache_file()
{
  return (fs::path(hexapod_hardware_cpp::workspace_root_from_package_prefix(kPackageName)) / "wt901_serial_cache.yaml").string();
}

void print_usage()
{
  std::cout <<
    "Usage: ros2 run hexapod_hardware_cpp wt901_config_tool -- [options]\n"
    "Options:\n"
    "  --device <tty>              Serial device (default /dev/ttyTHS1)\n"
    "  --cache-file <path>         Cache file for last working baud\n"
    "  --set-baud <baud>           Target baud (9600..921600)\n"
    "  --set-rate-hz <hz>          Target output rate (1,2,5,10,20,50,100,125,200)\n"
    "  --highest-test-preset       Set 200 Hz and 921600 baud\n"
    "  --practical-high-preset     Set 200 Hz and 115200 baud\n"
    "  --detect-only               Only detect current baud and cache it\n"
    "  --current-baud <baud>       Skip auto-detect and use known current baud\n"
    "  --detect-timeout-ms <ms>    Detect timeout per baud (default 1200)\n";
}

bool is_supported_baud(int baud)
{
  for (int b : kSupportedBauds) {
    if (b == baud) return true;
  }
  return false;
}

std::optional<uint8_t> baud_to_code(int baud)
{
  switch (baud) {
    case 2400: return 0x00;
    case 4800: return 0x01;
    case 9600: return 0x02;
    case 19200: return 0x03;
    case 38400: return 0x04;
    case 57600: return 0x05;
    case 115200: return 0x06;
    case 230400: return 0x07;
    case 460800: return 0x08;
    case 921600: return 0x09;
    default: return std::nullopt;
  }
}

std::optional<uint8_t> rate_hz_to_code(int hz)
{
  switch (hz) {
    case 1: return 0x03;
    case 2: return 0x04;
    case 5: return 0x05;
    case 10: return 0x06;
    case 20: return 0x07;
    case 50: return 0x08;
    case 100: return 0x09;
    case 125: return 0x0A;
    case 200: return 0x0B;
    default: return std::nullopt;
  }
}

Options parse_args(int argc, char ** argv)
{
  Options opt;
  opt.cache_file = default_cache_file();
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    auto require_value = [&](const std::string & name) -> std::string {
        if (i + 1 >= argc) {
          throw std::runtime_error("Missing value for " + name);
        }
        ++i;
        return std::string(argv[i]);
      };
    if (arg == "--device") {
      opt.device = require_value(arg);
    } else if (arg == "--cache-file") {
      opt.cache_file = require_value(arg);
    } else if (arg == "--set-baud") {
      opt.target_baud = std::stoi(require_value(arg));
    } else if (arg == "--set-rate-hz") {
      opt.target_rate_hz = std::stoi(require_value(arg));
    } else if (arg == "--detect-only") {
      opt.detect_only = true;
    } else if (arg == "--highest-test-preset") {
      opt.highest_test_preset = true;
    } else if (arg == "--practical-high-preset") {
      opt.practical_high_preset = true;
    } else if (arg == "--current-baud") {
      opt.current_baud = std::stoi(require_value(arg));
    } else if (arg == "--detect-timeout-ms") {
      opt.detect_timeout_ms = std::stoi(require_value(arg));
    } else if (arg == "--help" || arg == "-h") {
      print_usage();
      std::exit(0);
    } else {
      throw std::runtime_error("Unknown argument: " + arg);
    }
  }
  opt.device = hexapod_hardware_cpp::expand_user_path(opt.device);
  opt.cache_file = hexapod_hardware_cpp::expand_user_path(opt.cache_file);
  if (opt.highest_test_preset) {
    opt.target_rate_hz = 200;
    opt.target_baud = 921600;
  }
  if (opt.practical_high_preset) {
    opt.target_rate_hz = 200;
    opt.target_baud = 115200;
  }
  return opt;
}

std::optional<int> load_cached_baud(const std::string & path)
{
  try {
    if (!fs::exists(path)) {
      return std::nullopt;
    }
    const YAML::Node root = YAML::LoadFile(path);
    if (root["last_working_baud"]) {
      return root["last_working_baud"].as<int>();
    }
  } catch (...) {
  }
  return std::nullopt;
}

void save_cached_baud(const std::string & path, int baud, const std::string & device)
{
  fs::create_directories(fs::path(path).parent_path());
  YAML::Node root;
  root["last_working_baud"] = baud;
  root["device"] = device;
  root["note"] = "Updated by wt901_config_tool";
  std::ofstream out(path, std::ios::trunc);
  out << root;
}


uint8_t checksum11(const uint8_t * frame)
{
  uint16_t sum = 0;
  for (int i = 0; i < 10; ++i) {
    sum += frame[i];
  }
  return static_cast<uint8_t>(sum & 0xFF);
}

bool wait_for_any_valid_frame(hexapod_hardware_cpp::SerialPort & port, int timeout_ms)
{
  auto start = std::chrono::steady_clock::now();
  std::array<uint8_t, 512> buf{};
  std::vector<uint8_t> window;
  window.reserve(64);
  while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() < timeout_ms) {
    const auto n = port.read_some(buf.data(), buf.size());
    for (std::size_t i = 0; i < n; ++i) {
      window.push_back(buf[i]);
      while (window.size() >= 11) {
        if (window[0] != 0x55) {
          window.erase(window.begin());
          continue;
        }
        const uint8_t type = window[1];
        if (!(type == 0x51 || type == 0x52 || type == 0x53 || type == 0x54)) {
          window.erase(window.begin());
          continue;
        }
        if (checksum11(window.data()) == window[10]) {
          return true;
        }
        window.erase(window.begin());
      }
    }
    std::this_thread::sleep_for(5ms);
  }
  return false;
}

bool wait_for_sample(hexapod_hardware_cpp::SerialPort & port, int timeout_ms)
{
  hexapod_hardware_cpp::Wt901Parser parser;
  auto start = std::chrono::steady_clock::now();
  std::array<uint8_t, 512> buf{};
  while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() < timeout_ms) {
    const auto n = port.read_some(buf.data(), buf.size());
    for (std::size_t i = 0; i < n; ++i) {
      auto sample = parser.push_byte(buf[i]);
      if (sample.has_value()) {
        return true;
      }
    }
    std::this_thread::sleep_for(5ms);
  }
  return false;
}

std::optional<int> detect_baud(const Options & opt)
{
  std::vector<int> trials;
  if (auto cached = load_cached_baud(opt.cache_file); cached.has_value()) {
    trials.push_back(*cached);
  }
  for (int baud : kSupportedBauds) {
    bool present = false;
    for (int existing : trials) {
      if (existing == baud) { present = true; break; }
    }
    if (!present) trials.push_back(baud);
  }

  for (int baud : trials) {
    try {
      hexapod_hardware_cpp::SerialPort port;
      port.open(opt.device, baud);
      std::this_thread::sleep_for(300ms);
      if (wait_for_any_valid_frame(port, opt.detect_timeout_ms) || wait_for_sample(port, opt.detect_timeout_ms)) {
        save_cached_baud(opt.cache_file, baud, opt.device);
        return baud;
      }
    } catch (...) {
    }
  }
  return std::nullopt;
}

void write_raw_cmd(hexapod_hardware_cpp::SerialPort & port, const uint8_t (&frame)[5], int repeat_count = 1)
{
  for (int i = 0; i < repeat_count; ++i) {
    const auto written = port.write_bytes(frame, sizeof(frame));
    if (written != sizeof(frame)) {
      throw std::runtime_error("Failed to write full WT901 command frame");
    }
    std::this_thread::sleep_for(80ms);
  }
}

void write_cmd(hexapod_hardware_cpp::SerialPort & port, uint8_t reg, uint8_t d1, uint8_t d2, int repeat_count = 1)
{
  const uint8_t frame[5] = {0xFF, 0xAA, reg, d1, d2};
  write_raw_cmd(port, frame, repeat_count);
}

void unlock_configuration(hexapod_hardware_cpp::SerialPort & port)
{
  const uint8_t unlock_frame[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
  write_raw_cmd(port, unlock_frame, 2);
}

void save_configuration(hexapod_hardware_cpp::SerialPort & port)
{
  write_cmd(port, 0x00, 0x00, 0x00, 2);
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto opt = parse_args(argc, argv);

    if (opt.target_baud.has_value() && !is_supported_baud(*opt.target_baud)) {
      throw std::runtime_error("Unsupported target baud");
    }
    if (opt.target_rate_hz.has_value() && !rate_hz_to_code(*opt.target_rate_hz).has_value()) {
      throw std::runtime_error("Unsupported target rate; use 1,2,5,10,20,50,100,125,200");
    }

    std::optional<int> detected = opt.current_baud;
    if (!detected.has_value()) {
      detected = detect_baud(opt);
    }
    if (!detected.has_value()) {
      std::cerr << "Failed to detect working WT901 baud on " << opt.device << "\n";
      return 2;
    }

    std::cout << "Detected WT901 on " << opt.device << " at " << *detected << " baud\n";
    if (opt.detect_only || (!opt.target_baud.has_value() && !opt.target_rate_hz.has_value())) {
      std::cout << "Detect-only complete. Cached baud saved to " << opt.cache_file << "\n";
      return 0;
    }

    hexapod_hardware_cpp::SerialPort port;
    port.open(opt.device, *detected);
    std::this_thread::sleep_for(350ms);

    unlock_configuration(port);
    std::cout << "Unlock command sent\n";

    if (opt.target_rate_hz.has_value()) {
      const auto rate_code = rate_hz_to_code(*opt.target_rate_hz).value();
      write_cmd(port, 0x03, rate_code, 0x00, 2);
      std::cout << "Requested return rate " << *opt.target_rate_hz << " Hz\n";
    }

    if (opt.target_baud.has_value()) {
      const auto baud_code = baud_to_code(*opt.target_baud).value();
      write_cmd(port, 0x04, baud_code, 0x00, 2);
      std::cout << "Requested baud " << *opt.target_baud << "\n";
    }

    save_configuration(port);
    std::cout << "Save command sent\n";
    port.close();

    const int verify_baud = opt.target_baud.value_or(*detected);
    std::this_thread::sleep_for(250ms);
    try {
      hexapod_hardware_cpp::SerialPort verify_port;
      verify_port.open(opt.device, verify_baud);
      std::this_thread::sleep_for(350ms);
      if (wait_for_any_valid_frame(verify_port, std::max(opt.detect_timeout_ms, 1500)) || wait_for_sample(verify_port, std::max(opt.detect_timeout_ms, 1500))) {
        save_cached_baud(opt.cache_file, verify_baud, opt.device);
        std::cout << "Verified WT901 output at " << verify_baud << " baud\n";
        return 0;
      }
    } catch (...) {
    }

    std::cerr << "Configuration commands sent, but live verification at the target baud failed. "
                 "Power-cycle the WT901, then run --detect-only. The last verified baud remains cached.\n";
    save_cached_baud(opt.cache_file, *detected, opt.device);
    return 1;
  } catch (const std::exception & e) {
    std::cerr << "wt901_config_tool error: " << e.what() << "\n";
    print_usage();
    return 2;
  }
}
