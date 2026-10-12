#include <cstdint>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "whisky_bot_hardware/pca9685.hpp"

namespace
{
class RecordingTransport : public whisky_bot_hardware::I2cTransport
{
public:
  struct Write
  {
    std::uint8_t start_register;
    std::vector<std::uint8_t> data;
  };

  bool open(int, int, std::string &) override
  {
    return true;
  }

  void close() override {}

  bool write_registers(
    std::uint8_t start_register, const std::uint8_t * data, std::size_t length,
    std::string &) override
  {
    writes.push_back({start_register, std::vector<std::uint8_t>(data, data + length)});
    return true;
  }

  std::vector<Write> writes;
};
}  // namespace

TEST(Pca9685, calculates_prescale_for_50_hz)
{
  std::uint8_t prescale{};
  std::string error;

  EXPECT_TRUE(
    whisky_bot_hardware::Pca9685::calculate_prescale(50.0, 25'000'000.0, prescale, error));
  EXPECT_EQ(prescale, 121);
}

TEST(Pca9685, converts_pulse_width_to_ticks)
{
  std::uint16_t ticks{};
  std::string error;

  EXPECT_TRUE(whisky_bot_hardware::Pca9685::pulse_width_to_ticks(1500.0, 50.0, ticks, error));
  EXPECT_EQ(ticks, 307);
}

TEST(Pca9685, rejects_an_invalid_pulse_width)
{
  std::uint16_t ticks{};
  std::string error;

  EXPECT_FALSE(whisky_bot_hardware::Pca9685::pulse_width_to_ticks(0.0, 50.0, ticks, error));
  EXPECT_FALSE(error.empty());
}

TEST(Pca9685, writes_the_requested_channel_registers)
{
  RecordingTransport transport;
  whisky_bot_hardware::Pca9685 controller(transport);
  std::string error;

  ASSERT_TRUE(controller.configure(50.0, 25'000'000.0, error));
  ASSERT_TRUE(controller.set_pulse_width(4, 1500.0, error));

  ASSERT_EQ(transport.writes.size(), 6U);
  EXPECT_EQ(transport.writes.back().start_register, 0x16);
  EXPECT_EQ(transport.writes.back().data, (std::vector<std::uint8_t>{0x00, 0x00, 0x33, 0x01}));
}
