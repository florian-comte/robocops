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
  hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
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

    loop_rate_ = std::stoi(info_.hardware_parameters["loop_rate"]);
    baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
    timeout_ms_ = std::stoi(info_.hardware_parameters["timeout_ms"]);
    gearbox_ratio_ = std::stoi(info_.hardware_parameters["gearbox_ratio"]);

    use_encoders_ = (info_.hardware_parameters["use_encoders"] == "true");

    // Setup wheels
    wheel_l_.setup(left_wheel_name_);
    wheel_r_.setup(right_wheel_name_);

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

      // Removed check for 2 state interfaces (position and velocity) since we don't have acces to it
      // since passing through maxon controllers.
      // We could check for state_interfaces.size() == 1 (but not really needed in our context)
        
      // if (joint.state_interfaces.size() != 2)
      // {
      //   RCLCPP_FATAL(
      //       get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
      //       joint.state_interfaces.size());
      //   return hardware_interface::CallbackReturn::ERROR;
      // }

      // if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      // {
      //   RCLCPP_FATAL(
      //       get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
      //       joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
      //       hardware_interface::HW_IF_POSITION);
      //   return hardware_interface::CallbackReturn::ERROR;
      // }

      // if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      // {
      //   RCLCPP_FATAL(
      //       get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
      //       joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
      //       hardware_interface::HW_IF_VELOCITY);
      //   return hardware_interface::CallbackReturn::ERROR;
      // }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  /**
   * @brief Configures the hardware, establishing the serial connection.
   *
   * @param previous_state Lifecycle state prior to configuration.
   * @return CallbackReturn indicating success or failure.
   */
  hardware_interface::CallbackReturn DiffBotSystemHardware::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_logger(), "Configuring ...please wait...");
    if (comms_.connected())
    {
      comms_.disconnect();
    }
    comms_.connect(device_, timeout_ms_);
    RCLCPP_INFO(get_logger(), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  /**
   * @brief Cleans up the hardware, closing the serial connection if open.
   *
   * @param previous_state Lifecycle state prior to cleanup.
   * @return CallbackReturn indicating success or failure.
   */
  hardware_interface::CallbackReturn DiffBotSystemHardware::on_cleanup(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_logger(), "Cleaning up ...please wait...");
    if (comms_.connected())
    {
      comms_.disconnect();
    }
    RCLCPP_INFO(get_logger(), "Successfully cleaned up!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  /**
   * @brief Activates the hardware, ensuring the serial connection is open.
   *
   * @param previous_state Lifecycle state prior to activation.
   * @return CallbackReturn indicating success or failure.
   */
  hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_logger(), "Activating ...please wait...");
    if (!comms_.connected())
    {
      return hardware_interface::CallbackReturn::ERROR;
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
  hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");
    RCLCPP_INFO(get_logger(), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  /**
   * @brief Exports state interfaces for each wheel's velocity.
   *
   * @return A vector of StateInterface objects for both wheels.
   */
  std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.encoder_speed));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.encoder_speed));

    return state_interfaces;
  }

  /**
   * @brief Exports command interfaces for each wheel's velocity.
   *
   * @return A vector of CommandInterface objects for both wheels.
   */
  std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.command_speed));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.command_speed));

    return command_interfaces;
  }

  /**
   * @brief Reads the encoder data from the hardware or mirrors command values if encoders are disabled.
   *
   * @param time Current time (unused).
   * @param period Duration since the last read (unused).
   * @return return_type::OK if successful, return_type::ERROR if communication failed.
   */
  hardware_interface::return_type DiffBotSystemHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &)
  {
    if (!comms_.connected())
    {
      return hardware_interface::return_type::ERROR;
    }

    if (use_encoders_)
    {
      comms_.read_encoder_values(&wheel_r_.encoder_speed, &wheel_l_.encoder_speed);

      wheel_r_.encoder_speed /= gearbox_ratio_;
      wheel_l_.encoder_speed /= gearbox_ratio_;
    }
    else
    {
      wheel_l_.encoder_speed = wheel_l_.command_speed;
      wheel_r_.encoder_speed = wheel_r_.command_speed;
    }

    return hardware_interface::return_type::OK;
  }

  /**
   * @brief Writes the velocity command values to the motors via Arduino.
   *
   * @param time Current time (unused).
   * @param period Duration since last write (unused).
   * @return return_type::OK if successful, return_type::ERROR if communication failed.
   */
  hardware_interface::return_type DiffBotSystemHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    if (!comms_.connected())
    {
      return hardware_interface::return_type::ERROR;
    }

    // send command
    comms_.send_command(
      static_cast<int16_t>(rad_per_sec_to_rpm(gearbox_ratio_ * command_interfaces_.left_wheel_command_speed)),
      static_cast<int16_t>(rad_per_sec_to_rpm(gearbox_ratio_ * command_interfaces_.right_wheel_command_speed)),
      // static_cast<bool>(gpio_commands_.brush),
      // static_cast<bool>(gpio_commands_.unload),
      // static_cast<bool>(gpio_commands_.unload),
      false,
      false,
      false,
      &state_interfaces_.left_wheel_encoder_speed,
      &state_interfaces_.right_wheel_encoder_speed,
      true);

    return hardware_interface::return_type::OK;
  }

} // namespace robocops_control

#include "pluginlib/class_list_macros.hpp"
/// @brief Export the plugin to be dynamically loaded by the ROS 2 control system.
PLUGINLIB_EXPORT_CLASS(
    robocops_control::DiffBotSystemHardware, hardware_interface::SystemInterface)