#ifndef WHISKY_BOT_HARDWARE__PCA9685_SYSTEM_INTERFACE_HPP_
#define WHISKY_BOT_HARDWARE__PCA9685_SYSTEM_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include "whisky_bot_hardware/pca9685.hpp"

namespace whisky_bot_hardware
{

  class Pca9685SystemInterface : public hardware_interface::SystemInterface
  {
  public:
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams &params) override;
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    struct JointConfiguration
    {
      std::string name;
      std::uint8_t channel;
      double position_min_rad;
      double position_max_rad;
      double pulse_min_us;
      double pulse_max_us;
      double home_position_rad;
      bool invert;
    };

    bool parse_hardware_parameters(std::string &error);
    bool write_positions(const std::vector<double> &positions, std::string &error);
    bool position_to_pulse_width(
        const JointConfiguration &joint, double position_rad, double &pulse_width_us,
        std::string &error) const;

    int i2c_bus_{1};
    int i2c_address_{0x40};
    double pwm_frequency_hz_{50.0};
    double oscillator_frequency_hz_{25'000'000.0};
    bool configured_{false};
    std::vector<JointConfiguration> joints_;
    std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces_;
    std::vector<hardware_interface::StateInterface::SharedPtr> state_interfaces_;
    std::unique_ptr<LinuxI2cTransport> transport_;
    std::unique_ptr<Pca9685> controller_;
  };

}

#endif
