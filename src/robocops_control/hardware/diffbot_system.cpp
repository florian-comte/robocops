#include "diffbot_system.hpp"
#include "utils.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robocops_control
{

  /**
   * @brief Initializes the hardware interface with configuration parameters and joint info.
   *
   * Validates joint interfaces and reads required parameters like wheel names, serial settings, etc.
   *
   * @param info The hardware info provided from the URDF.
   * @return CallbackReturn::SUCCESS if initialization succeeds, otherwise CallbackReturn::ERROR.
   */
  hardware_interface::CallbackReturn RobocopsSystemHardware::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Get the params
    left_wheel_name_ = info_.hardware_parameters["left_wheel_name"];
    right_wheel_name_ = info_.hardware_parameters["right_wheel_name"];

    device_ = info_.hardware_parameters["device"];
    timeout_ms_ = std::stoi(info_.hardware_parameters["timeout_ms"]);
    gearbox_ratio_ = std::stoi(info_.hardware_parameters["gearbox_ratio"]);

    use_encoders_ = (info_.hardware_parameters["use_encoders"] == "true");

    left_wheel_encoder_ = 0;
    right_wheel_encoder_ = 0;

    lift_authorized_ = false;
    lift_active_ = false;
    unload_active_ = false;
    brushes_active_ = false;

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
            joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' has '%s' command interface. '%s' expected.",
            joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
            hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  /**
   * @brief Configures the hardware, establishing the serial connection.
   *
   * @param previous_state Lifecycle state prior to configuration.
   * @return CallbackReturn indicating success or failure.
   */
  hardware_interface::CallbackReturn RobocopsSystemHardware::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_logger(), "Configuring ...please wait...");
    if (comms_.connected())
    {
      comms_.disconnect();
    }
    comms_.connect(device_, 57600, timeout_ms_);

    for (const auto &[name, descr] : joint_state_interfaces_)
    {
      set_state(name, 0.0);
    }
    for (const auto &[name, descr] : joint_command_interfaces_)
    {
      set_command(name, 0.0);
    }

    for (const auto &[name, descr] : gpio_state_interfaces_)
    {
      set_state(name, 0.0);
    }
    for (const auto &[name, descr] : gpio_command_interfaces_)
    {
      set_command(name, 0.0);
    }

    RCLCPP_INFO(get_logger(), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  /**
   * @brief Activates the hardware, ensuring the serial connection is open.
   *
   * @param previous_state Lifecycle state prior to activation.
   * @return CallbackReturn indicating success or failure.
   */
  hardware_interface::CallbackReturn RobocopsSystemHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_logger(), "Activating ...please wait...");
    if (!comms_.connected())
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    for (const auto &[name, descr] : joint_state_interfaces_)
    {
      set_command(name, get_state(name));
    }
    for (const auto &[name, descr] : gpio_command_interfaces_)
    {
      set_command(name, get_state(name));
    }

    RCLCPP_INFO(get_logger(), "Successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  /**
   * @brief Deactivates the hardware. No teardown logic is required here.
   *
   * @param previous_state Lifecycle state prior to deactivation.
   * @return CallbackReturn indicating success or failure.
   */
  hardware_interface::CallbackReturn RobocopsSystemHardware::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {

    if (comms_.connected())
    {
      comms_.disconnect();
    }

    RCLCPP_INFO(get_logger(), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  /**
   * @brief Reads the encoder data from the hardware or mirrors command values if encoders are disabled.
   *
   * @param time Current time (unused).
   * @param period Duration since the last read (unused).
   * @return return_type::OK if successful, return_type::ERROR if communication failed.
   */
  hardware_interface::return_type RobocopsSystemHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &)
  {
    if (!comms_.connected())
    {
      return hardware_interface::return_type::ERROR;
    }

    // Set state left/right encoder value
    if (use_encoders_)
    {
      set_state(left_wheel_name_ + "/velocity", left_wheel_encoder_ / gearbox_ratio_);
      set_state(right_wheel_name_ + "/velocity", right_wheel_encoder_ / gearbox_ratio_);
    }
    else
    {
      set_state(left_wheel_name_ + "/velocity", get_command(left_wheel_name_ + "/velocity"));
      set_state(right_wheel_name_ + "/velocity", get_command(left_wheel_name_ + "/velocity"));
    }

    // Set state authorized/active lift
    set_state("lift/authorized", lift_authorized_ ? 1.0 : 0.0);
    set_state("lift/active", lift_active_ ? 1.0 : 0.0);

    // Set state active unload
    set_state("unload/active", unload_active_ ? 1.0 : 0.0);

    // Set state active brushes
    set_state("brushes/active", brushes_active_ ? 1.0 : 0.0);

    return hardware_interface::return_type::OK;
  }

  /**
   * @brief Writes the velocity command values to the motors via Arduino.
   *
   * @param time Current time (unused).
   * @param period Duration since last write (unused).
   * @return return_type::OK if successful, return_type::ERROR if communication failed.
   */
  hardware_interface::return_type RobocopsSystemHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    if (!comms_.connected())
    {
      return hardware_interface::return_type::ERROR;
    }

    comms_.send_command(
        static_cast<int>(rad_per_sec_to_rpm(gearbox_ratio_ * get_command(left_wheel_name_ + "/velocity"))),
        static_cast<int>(rad_per_sec_to_rpm(gearbox_ratio_ * get_command(right_wheel_name_ + "/velocity"))),
        static_cast<bool>(get_command("brushes/active")),
        static_cast<bool>(get_command("unload/active")),
        static_cast<bool>(get_command("lift/authorized")),
        &left_wheel_encoder_,
        &right_wheel_encoder_,
        &lift_authorized_,
        &lift_active_,
        &unload_active_,
        &brushes_active_,
        true);

    return hardware_interface::return_type::OK;
  }

} // namespace robocops_control

#include "pluginlib/class_list_macros.hpp"
/// @brief Export the plugin to be dynamically loaded by the ROS 2 control system.
PLUGINLIB_EXPORT_CLASS(
    robocops_control::RobocopsSystemHardware, hardware_interface::SystemInterface)