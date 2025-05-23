#include "arduino_comms.hpp"

#include <iostream>
#include <stdexcept>
#include <string>
#include <serial/serial.h>

void ArduinoComms::connect(const std::string &serial_device, int32_t timeout_ms)
{
    timeout_ms_ = timeout_ms;

    try
    {
        serial::Timeout to = serial::Timeout::simpleTimeout(timeout_ms_);
        serial_.setPort(serial_device);
        serial_.setBaudrate(57600);
        serial_.setTimeout(to);
        serial_.open();

        if (!serial_.isOpen())
        {
            throw std::runtime_error("Failed to open serial port.");
        }

        std::cout << "[Serial] Connected to " << serial_device << " at 57600 baud." << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[Serial] Connection exception: " << e.what() << std::endl;
    }
}

void ArduinoComms::disconnect()
{
    if (serial_.isOpen())
    {
        serial_.close();
        std::cout << "[Serial] Disconnected from serial port." << std::endl;
    }
}

bool ArduinoComms::connected() const
{
    return serial_.isOpen();
}

void ArduinoComms::send_command(int16_t maxon_left,
                                int16_t maxon_right,
                                bool brush_signal,
                                bool activate_unload_routine,
                                bool authorized_lift_routine,
                                double *encoder_maxon_left,
                                double *encoder_maxon_right,
                                bool print_output)
{
    if (!connected())
    {
        std::cerr << "[Serial] Port not connected." << std::endl;
        return;
    }

    uint8_t cmd[5];

    // Offset values to be in range (0 - 20000)
    maxon_left += 10000;
    maxon_right += 10000;

    cmd[0] = (maxon_left >> 8) & 0xFF;
    cmd[1] = maxon_left & 0xFF;
    cmd[2] = (maxon_right >> 8) & 0xFF;
    cmd[3] = maxon_right & 0xFF;
    cmd[4] = 0;
    cmd[4] |= (brush_signal & 0x01);
    cmd[4] |= ((activate_unload_routine & 0x01) << 1);
    cmd[4] |= ((authorized_lift_routine & 0x01) << 2);

    try
    {
        // Send command
        serial_.write(cmd, 5);

        // Read response
        std::string response = serial_.read(5);

        if (response.size() != 5)
        {
            std::cerr << "[Serial] Incomplete response received: " << response.size() << " bytes." << std::endl;
            return;
        }

        // Parse encoder values
        *encoder_maxon_left = static_cast<int16_t>((static_cast<uint8_t>(response[0]) << 8) |
                                                   static_cast<uint8_t>(response[1]));

        *encoder_maxon_right = static_cast<int16_t>((static_cast<uint8_t>(response[2]) << 8) |
                                                    static_cast<uint8_t>(response[3]));

        if (print_output)
        {
            std::cout << "[Serial] Sent command: ";
            for (int i = 0; i < 5; ++i)
            {
                std::cout << "0x" << std::hex << static_cast<int>(cmd[i]) << " ";
            }
            std::cout << std::dec << "\n[Serial] Encoders: L=" << *encoder_maxon_left << " R=" << *encoder_maxon_right << std::endl;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "[Serial] Exception during send_command: " << e.what() << std::endl;
    }
}
