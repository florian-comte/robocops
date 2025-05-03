#ifndef DIFFDRIVE_ARDUINO_WHEEL_HPP
#define DIFFDRIVE_ARDUINO_WHEEL_HPP

#include <string>

/**
 * @class Wheel
 * @brief Represents a wheel in a differential drive robot, holding state and command speeds.
 *
 * Stores the name of the wheel, current encoder-reported speed, and the target command speed.
 * Speeds are expressed in radians per second.
 */
class Wheel
{
public:
    /// Name identifier for the wheel.
    std::string name = "";

    /// Speed reported by the encoder (in radians per second).
    double encoder_speed = 0;

    /// Speed command to be sent to the wheel (in radians per second).
    double command_speed = 0;

    /**
     * @brief Default constructor.
     */
    Wheel();

    /**
     * @brief Constructor with wheel name initialization.
     * @param wheel_name Name to assign to the wheel.
     */
    Wheel(const std::string &wheel_name);

    /**
     * @brief Sets up the wheel with a given name.
     * @param wheel_name The name to assign to the wheel.
     */
    void setup(const std::string &wheel_name);
};

#endif // DIFFDRIVE_ARDUINO_WHEEL_HPP
