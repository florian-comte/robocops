#ifndef ROBOCOPS_CONTROL_DIFFBOT_SYSTEM_HEADER
#define ROBOCOPS_CONTROL_DIFFBOT_SYSTEM_HEADER

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "arduino_comms.hpp"
#include "wheel.hpp"

namespace robocops_control
{
  /**
   * @class DiffBotSystemHardware
   * @brief Hardware interface implementation for a differential drive robot using Arduino.
   *
   * This class integrates with ROS 2 control and communicates with the robot hardware
   * (Arduino-based) to control wheels and read encoder data.
   */
  class DiffBotSystemHardware : public hardware_interface::SystemInterface
  {
  public:
    /// @brief Shared pointer type definition for this class.
    RCLCPP_SHARED_PTR_DEFINITIONS(DiffBotSystemHardware)

    /**
     * @brief Initializes the hardware with configuration data from the URDF/XACRO.
     *
     * @param info Hardware info loaded from the robot description.
     * @return CallbackReturn::SUCCESS or CallbackReturn::ERROR.
     */
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;

    /**
     * @brief Configures the hardware before activation.
     *
     * @param previous_state The previous lifecycle state.
     * @return CallbackReturn indicating success or failure.
     */
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    /**
     * @brief Activates the hardware interface.
     *
     * @param previous_state The previous lifecycle state.
     * @return CallbackReturn indicating success or failure.
     */
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    /**
     * @brief Deactivates the hardware interface.
     *
     * @param previous_state The previous lifecycle state.
     * @return CallbackReturn indicating success or failure.
     */
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    /**
     * @brief Reads sensor data from the hardware.
     *
     * @param time The current time.
     * @param period The time elapsed since the last read.
     * @return return_type::OK or return_type::ERROR.
     */
    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    /**
     * @brief Sends commands to the hardware.
     *
     * @param time The current time.
     * @param period The time elapsed since the last write.
     * @return return_type::OK or return_type::ERROR.
     */
    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    /// Name of the left wheel joint.
    std::string left_wheel_name_;

    /// Name of the right wheel joint.
    std::string right_wheel_name_;

    /// Control loop rate in Hz.
    int loop_rate_;

    /// Serial device path (e.g., "/dev/ttyUSB0").
    std::string device_;

    /// Baud rate for serial communication.
    int baud_rate_;

    /// Timeout in milliseconds for serial communication.
    int timeout_ms_;

    /// Whether to use encoder feedback.
    bool use_encoders_;

    /// Gearbox ratio used for converting encoder ticks to wheel rotation.
    int gearbox_ratio_;

    /// Arduino communication interface.
    ArduinoComms comms_;

    /// Left wheel abstraction.
    Wheel wheel_l_;

    /// Right wheel abstraction.
    Wheel wheel_r_;

    bool lift_authorized_;
    bool lift_active_;
    bool unload_active_;
    bool brushes_active_;
  };

} // namespace robocops_control

#endif // ROBOCOPS_CONTROL_DIFFBOT_SYSTEM_HEADER
