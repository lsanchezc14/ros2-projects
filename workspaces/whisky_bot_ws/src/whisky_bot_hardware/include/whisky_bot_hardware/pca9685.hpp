#ifndef WHISKY_BOT_HARDWARE__PCA9685_HPP_
#define WHISKY_BOT_HARDWARE__PCA9685_HPP_

#include <cstdint>
#include <string>

namespace whisky_bot_hardware
{

  class I2cTransport
  {
  public:
    virtual ~I2cTransport() = default;

    virtual bool open(int bus, int address, std::string &error) = 0;
    virtual void close() = 0;
    virtual bool write_registers(
        std::uint8_t start_register, const std::uint8_t *data, std::size_t length,
        std::string &error) = 0;
  };

  class LinuxI2cTransport final : public I2cTransport
  {
  public:
    ~LinuxI2cTransport() override;

    bool open(int bus, int address, std::string &error) override;
    void close() override;
    bool write_registers(
        std::uint8_t start_register, const std::uint8_t *data, std::size_t length,
        std::string &error) override;

  private:
    int file_descriptor_{-1};
  };

  class Pca9685
  {
  public:
    explicit Pca9685(I2cTransport &transport);

    bool configure(
        double frequency_hz, double oscillator_frequency_hz, std::string &error);
    bool set_pulse_width(std::uint8_t channel, double pulse_width_us, std::string &error);

    static bool calculate_prescale(
        double frequency_hz, double oscillator_frequency_hz, std::uint8_t &prescale,
        std::string &error);
    static bool pulse_width_to_ticks(
        double pulse_width_us, double frequency_hz, std::uint16_t &ticks, std::string &error);

  private:
    bool write_register(std::uint8_t address, std::uint8_t value, std::string &error);

    I2cTransport &transport_;
    double frequency_hz_{0.0};
  };

}

#endif
