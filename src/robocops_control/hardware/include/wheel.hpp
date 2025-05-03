#ifndef DIFFDRIVE_ARDUINO_WHEEL_HPP
#define DIFFDRIVE_ARDUINO_WHEEL_HPP

#include <string>

class Wheel
{
public:
    std::string name = "";

    // Values are in rad/s
    double encoder_speed = 0;
    double command_speed = 0;

    Wheel();
    Wheel(const std::string &wheel_name);

    void setup(const std::string &wheel_name);
};



#endif // DIFFDRIVE_ARDUINO_WHEEL_HPP
