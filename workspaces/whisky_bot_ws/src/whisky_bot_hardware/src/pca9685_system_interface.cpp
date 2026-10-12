#include "whisky_bot_hardware/pca9685_system_interface.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace whisky_bot_hardware
{
namespace
{
template<typename T>
bool parse_number(
  const std::unordered_map<std::string, std::string> & parameters, const std::string & name,
  T & value, std::string & error)
{
  const auto parameter = parameters.find(name);
  if (parameter == parameters.end()) {
    error = "Missing required parameter '" + name + "'";
    return false;
  }
  try {
    std::size_t parsed_length{};
    if constexpr (std::is_same_v<T, int>) {
      value = std::stoi(parameter->second, &parsed_length, 0);
    } else {
      value = std::stod(parameter->second, &parsed_length);
    }
    if (parsed_length != parameter->second.size() || !std::isfinite(value)) {
      error = "Parameter '" + name + "' must be a finite number";
      return false;
    }
  } catch (const std::exception &) {
    error = "Parameter '" + name + "' must be a valid number";
    return false;
  }
  return true;
}

bool parse_bool(
  const std::unordered_map<std::string, std::string> & parameters, const std::string & name,
  bool & value, std::string & error)
{
  const auto parameter = parameters.find(name);
  if (parameter == parameters.end()) {
    error = "Missing required parameter '" + name + "'";
    return false;
  }
  if (parameter->second == "true") {
    value = true;
    return true;
  }
  if (parameter->second == "false") {
    value = false;
    return true;
  }
  error = "Parameter '" + name + "' must be 'true' or 'false'";
  return false;
}

void log_error(const std::string & message)
{
  RCLCPP_ERROR(rclcpp::get_logger("whisky_bot_hardware"), "%s", message.c_str());
}
}  // namespace

hardware_interface::CallbackReturn Pca9685SystemInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  const auto result = hardware_interface::SystemInterface::on_init(params);
  if (result != hardware_interface::CallbackReturn::SUCCESS) {
    return result;
  }

  std::string error;
  if (!parse_hardware_parameters(error)) {
    log_error(error);
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool Pca9685SystemInterface::parse_hardware_parameters(std::string & error)
{
  if (info_.joints.size() != 5) {
    error = "Pca9685SystemInterface requires exactly five configured joints";
    return false;
  }
  if (!parse_number(info_.hardware_parameters, "i2c_bus", i2c_bus_, error) ||
    !parse_number(info_.hardware_parameters, "i2c_address", i2c_address_, error) ||
    !parse_number(info_.hardware_parameters, "pwm_frequency_hz", pwm_frequency_hz_, error) ||
    !parse_number(
      info_.hardware_parameters, "oscillator_frequency_hz", oscillator_frequency_hz_, error))
  {
    return false;
  }
  if (i2c_bus_ < 0 || i2c_address_ < 0x03 || i2c_address_ > 0x77) {
    error = "I2C bus or 7-bit address is outside its valid range";
    return false;
  }

  std::set<std::uint8_t> channels;
  joints_.clear();
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces.front().name != hardware_interface::HW_IF_POSITION ||
      joint.state_interfaces.size() != 1 ||
      joint.state_interfaces.front().name != hardware_interface::HW_IF_POSITION)
    {
      error = "Joint '" + joint.name + "' must expose exactly one position command and state interface";
      return false;
    }

    JointConfiguration configuration{};
    configuration.name = joint.name;
    int channel{};
    if (!parse_number(joint.parameters, "channel", channel, error) ||
      !parse_number(joint.parameters, "position_min_rad", configuration.position_min_rad, error) ||
      !parse_number(joint.parameters, "position_max_rad", configuration.position_max_rad, error) ||
      !parse_number(joint.parameters, "pulse_min_us", configuration.pulse_min_us, error) ||
      !parse_number(joint.parameters, "pulse_max_us", configuration.pulse_max_us, error) ||
      !parse_number(joint.parameters, "home_position_rad", configuration.home_position_rad, error) ||
      !parse_bool(joint.parameters, "invert", configuration.invert, error))
    {
      error = "Joint '" + joint.name + "': " + error;
      return false;
    }
    if (channel < 0 || channel > 15 || !channels.insert(static_cast<std::uint8_t>(channel)).second) {
      error = "Joint '" + joint.name + "' has a duplicate or invalid PCA9685 channel";
      return false;
    }
    configuration.channel = static_cast<std::uint8_t>(channel);
    if (configuration.position_min_rad >= configuration.position_max_rad ||
      configuration.pulse_min_us <= 0.0 ||
      configuration.pulse_min_us >= configuration.pulse_max_us ||
      configuration.home_position_rad < configuration.position_min_rad ||
      configuration.home_position_rad > configuration.position_max_rad)
    {
      error = "Joint '" + joint.name + "' has invalid calibration or home-position values";
      return false;
    }
    joints_.push_back(configuration);
  }
  return true;
}

hardware_interface::CallbackReturn Pca9685SystemInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  std::string error;
  transport_ = std::make_unique<LinuxI2cTransport>();
  if (!transport_->open(i2c_bus_, i2c_address_, error)) {
    log_error(error);
    transport_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }
  controller_ = std::make_unique<Pca9685>(*transport_);
  if (!controller_->configure(pwm_frequency_hz_, oscillator_frequency_hz_, error)) {
    log_error(error);
    controller_.reset();
    transport_->close();
    transport_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }
  configured_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Pca9685SystemInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  std::vector<double> home_positions;
  home_positions.reserve(joints_.size());
  for (const auto & joint : joints_) {
    home_positions.push_back(joint.home_position_rad);
  }

  std::string error;
  if (!configured_ || !write_positions(home_positions, error)) {
    log_error(error.empty() ? "PCA9685 is not configured" : error);
    return hardware_interface::CallbackReturn::ERROR;
  }
  command_interfaces_.clear();
  state_interfaces_.clear();
  for (std::size_t index = 0; index < joints_.size(); ++index) {
    const auto interface_name = joints_[index].name + "/" + hardware_interface::HW_IF_POSITION;
    const auto & command_interface = get_command_interface_handle(interface_name);
    const auto & state_interface = get_state_interface_handle(interface_name);
    command_interfaces_.push_back(command_interface);
    state_interfaces_.push_back(state_interface);
    set_command(command_interface, home_positions[index], true);
    set_state(state_interface, home_positions[index], true);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Pca9685SystemInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  configured_ = false;
  command_interfaces_.clear();
  state_interfaces_.clear();
  controller_.reset();
  if (transport_) {
    transport_->close();
    transport_.reset();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Pca9685SystemInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Pca9685SystemInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!configured_) {
    return hardware_interface::return_type::ERROR;
  }
  if (command_interfaces_.size() != joints_.size() || state_interfaces_.size() != joints_.size()) {
    log_error("PCA9685 joint interfaces are unavailable");
    return hardware_interface::return_type::ERROR;
  }
  std::vector<double> position_commands;
  position_commands.reserve(joints_.size());
  for (const auto & command_interface : command_interfaces_) {
    double position_command{};
    if (!get_command(command_interface, position_command, false)) {
      log_error("Could not read a PCA9685 position command");
      return hardware_interface::return_type::ERROR;
    }
    position_commands.push_back(position_command);
  }

  std::string error;
  if (!write_positions(position_commands, error)) {
    log_error(error);
    return hardware_interface::return_type::ERROR;
  }
  for (std::size_t index = 0; index < joints_.size(); ++index) {
    if (!set_state(state_interfaces_[index], position_commands[index], false)) {
      log_error("Could not publish a PCA9685 position state");
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

bool Pca9685SystemInterface::write_positions(
  const std::vector<double> & positions, std::string & error)
{
  if (positions.size() != joints_.size()) {
    error = "Position command count does not match configured joints";
    return false;
  }
  for (std::size_t index = 0; index < joints_.size(); ++index) {
    double pulse_width_us{};
    if (!position_to_pulse_width(joints_[index], positions[index], pulse_width_us, error) ||
      !controller_->set_pulse_width(joints_[index].channel, pulse_width_us, error))
    {
      return false;
    }
  }
  return true;
}

bool Pca9685SystemInterface::position_to_pulse_width(
  const JointConfiguration & joint, double position_rad, double & pulse_width_us,
  std::string & error) const
{
  if (!std::isfinite(position_rad)) {
    error = "Joint '" + joint.name + "' received a non-finite position command";
    return false;
  }
  const auto bounded_position = std::clamp(
    position_rad, joint.position_min_rad, joint.position_max_rad);
  double ratio = (bounded_position - joint.position_min_rad) /
    (joint.position_max_rad - joint.position_min_rad);
  if (joint.invert) {
    ratio = 1.0 - ratio;
  }
  pulse_width_us = joint.pulse_min_us + ratio * (joint.pulse_max_us - joint.pulse_min_us);
  return true;
}

}  // namespace whisky_bot_hardware

PLUGINLIB_EXPORT_CLASS(
  whisky_bot_hardware::Pca9685SystemInterface, hardware_interface::SystemInterface)
