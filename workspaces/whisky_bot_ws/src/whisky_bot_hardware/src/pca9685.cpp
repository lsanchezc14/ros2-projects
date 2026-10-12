#include "whisky_bot_hardware/pca9685.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <sstream>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>

#include <linux/i2c-dev.h>

namespace whisky_bot_hardware
{
namespace
{
constexpr std::uint8_t kMode1 = 0x00;
constexpr std::uint8_t kMode2 = 0x01;
constexpr std::uint8_t kLed0OnLow = 0x06;
constexpr std::uint8_t kPrescale = 0xfe;
constexpr std::uint8_t kMode1Sleep = 0x10;
constexpr std::uint8_t kMode1AutoIncrement = 0x20;
constexpr std::uint8_t kMode1Restart = 0x80;
constexpr std::uint8_t kMode2OutDriver = 0x04;
constexpr double kPwmResolution = 4096.0;

std::string system_error(const std::string & operation)
{
  return operation + ": " + std::strerror(errno);
}
}  // namespace

LinuxI2cTransport::~LinuxI2cTransport()
{
  close();
}

bool LinuxI2cTransport::open(int bus, int address, std::string & error)
{
  close();
  const std::string device = "/dev/i2c-" + std::to_string(bus);
  file_descriptor_ = ::open(device.c_str(), O_RDWR | O_CLOEXEC);
  if (file_descriptor_ < 0) {
    error = system_error("Could not open " + device);
    return false;
  }
  if (ioctl(file_descriptor_, I2C_SLAVE, address) < 0) {
    error = system_error("Could not select PCA9685 address 0x" + [&address]() {
      std::ostringstream address_stream;
      address_stream << std::hex << address;
      return address_stream.str();
    }());
    close();
    return false;
  }
  return true;
}

void LinuxI2cTransport::close()
{
  if (file_descriptor_ >= 0) {
    ::close(file_descriptor_);
    file_descriptor_ = -1;
  }
}

bool LinuxI2cTransport::write_registers(
  std::uint8_t start_register, const std::uint8_t * data, std::size_t length,
  std::string & error)
{
  if (file_descriptor_ < 0) {
    error = "I2C device is not open";
    return false;
  }
  std::array<std::uint8_t, 5> message{};
  if (length > message.size() - 1) {
    error = "I2C register write is too large";
    return false;
  }
  message[0] = start_register;
  std::copy(data, data + length, message.begin() + 1);
  const auto expected = static_cast<ssize_t>(length + 1);
  if (::write(file_descriptor_, message.data(), length + 1) != expected) {
    error = system_error("Could not write PCA9685 register");
    return false;
  }
  return true;
}

Pca9685::Pca9685(I2cTransport & transport)
: transport_(transport)
{
}

bool Pca9685::calculate_prescale(
  double frequency_hz, double oscillator_frequency_hz, std::uint8_t & prescale,
  std::string & error)
{
  if (!std::isfinite(frequency_hz) || !std::isfinite(oscillator_frequency_hz) ||
    frequency_hz <= 0.0 || oscillator_frequency_hz <= 0.0)
  {
    error = "PWM and oscillator frequencies must be finite positive values";
    return false;
  }

  const auto calculated = std::round(oscillator_frequency_hz / (kPwmResolution * frequency_hz)) - 1.0;
  if (calculated < 3.0 || calculated > 255.0) {
    error = "PWM frequency produces a PCA9685 prescale outside [3, 255]";
    return false;
  }
  prescale = static_cast<std::uint8_t>(calculated);
  return true;
}

bool Pca9685::pulse_width_to_ticks(
  double pulse_width_us, double frequency_hz, std::uint16_t & ticks, std::string & error)
{
  if (!std::isfinite(pulse_width_us) || !std::isfinite(frequency_hz) ||
    pulse_width_us <= 0.0 || frequency_hz <= 0.0)
  {
    error = "Pulse width and PWM frequency must be finite positive values";
    return false;
  }
  const auto calculated = std::round(pulse_width_us * frequency_hz * kPwmResolution / 1'000'000.0);
  if (calculated < 1.0 || calculated > 4095.0) {
    error = "Pulse width produces a PCA9685 tick count outside [1, 4095]";
    return false;
  }
  ticks = static_cast<std::uint16_t>(calculated);
  return true;
}

bool Pca9685::write_register(std::uint8_t address, std::uint8_t value, std::string & error)
{
  return transport_.write_registers(address, &value, 1, error);
}

bool Pca9685::configure(
  double frequency_hz, double oscillator_frequency_hz, std::string & error)
{
  std::uint8_t prescale{};
  if (!calculate_prescale(frequency_hz, oscillator_frequency_hz, prescale, error)) {
    return false;
  }

  if (!write_register(kMode1, kMode1Sleep, error) ||
    !write_register(kPrescale, prescale, error) ||
    !write_register(kMode2, kMode2OutDriver, error) ||
    !write_register(kMode1, kMode1AutoIncrement, error))
  {
    return false;
  }
  std::this_thread::sleep_for(std::chrono::microseconds(500));
  if (!write_register(kMode1, kMode1AutoIncrement | kMode1Restart, error)) {
    return false;
  }
  frequency_hz_ = frequency_hz;
  return true;
}

bool Pca9685::set_pulse_width(
  std::uint8_t channel, double pulse_width_us, std::string & error)
{
  if (channel > 15) {
    error = "PCA9685 channel must be in [0, 15]";
    return false;
  }
  std::uint16_t ticks{};
  if (!pulse_width_to_ticks(pulse_width_us, frequency_hz_, ticks, error)) {
    return false;
  }

  const std::array<std::uint8_t, 4> values{
    0x00, 0x00,
    static_cast<std::uint8_t>(ticks & 0xff),
    static_cast<std::uint8_t>((ticks >> 8) & 0x0f)};
  return transport_.write_registers(
    static_cast<std::uint8_t>(kLed0OnLow + (4 * channel)), values.data(), values.size(), error);
}

}  // namespace whisky_bot_hardware
