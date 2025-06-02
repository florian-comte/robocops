#include "arduino_comms.hpp"
#include <iostream>
#include <sstream>
#include <cstdlib>
#include "utils.hpp"

/**
 * @brief Converts an integer baud rate to a LibSerial::BaudRate enum.
 *
 * @param baud_rate The integer baud rate to convert.
 * @return Corresponding LibSerial::BaudRate value, or BAUD_57600 as default for unsupported rates.
 */
LibSerial::BaudRate ArduinoComms::convert_baud_rate(int baud_rate)
{
    switch (baud_rate)
    {
    case 1200:
        return LibSerial::BaudRate::BAUD_1200;
    case 1800:
        return LibSerial::BaudRate::BAUD_1800;
    case 2400:
        return LibSerial::BaudRate::BAUD_2400;
    case 4800:
        return LibSerial::BaudRate::BAUD_4800;
    case 9600:
        return LibSerial::BaudRate::BAUD_9600;
    case 19200:
        return LibSerial::BaudRate::BAUD_19200;
    case 38400:
        return LibSerial::BaudRate::BAUD_38400;
    case 57600:
        return LibSerial::BaudRate::BAUD_57600;
    case 115200:
        return LibSerial::BaudRate::BAUD_115200;
    case 230400:
        return LibSerial::BaudRate::BAUD_230400;
    default:
        std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
        return LibSerial::BaudRate::BAUD_57600;
    }
}

/**
 * @brief Opens and configures the serial connection.
 *
 * @param serial_device Path to the serial device (e.g., "/dev/ttyUSB0").
 * @param baud_rate Baud rate for communication.
 * @param timeout_ms Timeout for reading responses, in milliseconds.
 */
void ArduinoComms::connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
}

/**
 * @brief Closes the serial connection if it is open.
 */
void ArduinoComms::disconnect()
{
    serial_conn_.Close();
}

/**
 * @brief Checks whether the serial port is currently open.
 *
 * @return true if connected, false otherwise.
 */
bool ArduinoComms::connected() const
{
    return serial_conn_.IsOpen();
}

/**
 * @brief Sends motor speed commands to the Arduino.
 *
 * The command format is "m <rear_motor_speed> <left_motor_speed>\n".
 *
 * @param rear_motor_speed Speed command for the rear motor.
 * @param left_motor_speed Speed command for the left motor.
 */
void ArduinoComms::send_command(int16_t maxon_left,
                                int16_t maxon_right,
                                bool brushes_activate,
                                bool unload_activate,
                                bool lift_authorize,
                                double *encoder_left,
                                double *encoder_right,
                                bool *lift_authorized,
                                bool *lift_active,
                                bool *unload_active,
                                bool *brushes_active,
                                bool print_output)
{

    std::stringstream ss;
    ss << maxon_left << " " << maxon_right << " " << brushes_activate << " " << unload_activate << " " << lift_authorize << "\n";

    serial_conn_.FlushIOBuffers();
    serial_conn_.Write(ss.str());

    std::string response = "";
    try
    {
        // Read includes the \n.
        serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout &)
    {
        std::cerr << "[ROBOCOPS_CONTROL] ReadLine timed out." << std::endl;
    }

    if (print_output)
    {
        std::cout << "Sent: " << ss.str() << std::endl;
        std::cout << "Received: " << response << std::endl;
    }
}