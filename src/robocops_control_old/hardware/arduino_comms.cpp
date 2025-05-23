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
 * @brief Sends a message to the Arduino and optionally prints the response.
 * 
 * @param msg_to_send The string message to send.
 * @param print_output Whether to print the sent and received messages.
 * @return The response string received from the Arduino.
 */
std::string ArduinoComms::send_msg(const std::string &msg_to_send, bool print_output)
{
    serial_conn_.FlushIOBuffers();
    serial_conn_.Write(msg_to_send);

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
        std::cout << "Sent: " << msg_to_send << "\n Received: " << response << std::endl;
    }

    return response;
}

void ArduinoComms::send_command(int16_t maxon_left,
                                int16_t maxon_right,
                                bool brush_signal,
                                bool activate_unload_routine,
                                bool authorized_lift_routine,

                                double *encoder_maxon_left,
                                double *encoder_maxon_right,
                                // double *are_brushes_activated,
                                // double *is_unload_routine_activated,
                                // double *is_lift_routine_authorized,
                                bool print_output)
{
    uint8_t cmd[5];

    std::cout << maxon_left << std::endl;

    // Offset encoding
    maxon_left += 10000;
    maxon_right += 10000;

    // Compose command
    cmd[0] = (maxon_left >> 8) & 0xFF;
    cmd[1] = maxon_left & 0xFF;
    cmd[2] = (maxon_right >> 8) & 0xFF;
    cmd[3] = maxon_right & 0xFF;

    cmd[4] = 0;
    cmd[4] |= (brush_signal & 0x01);
    cmd[4] |= ((activate_unload_routine & 0x01) << 1);
    cmd[4] |= ((authorized_lift_routine & 0x01) << 2);

    if (print_output)
    {
        std::cout << "[DEBUG] Command to send: ";
        for (int i = 0; i < 5; ++i)
        {
            std::cout << "0x" << std::hex << std::uppercase
                      << static_cast<int>(cmd[i]) << " ";
        }
        std::cout << std::dec << std::endl;
    }

    if (!connected())
    {
        std::cerr << "[ROBOCOPS_CONTROL] Lib connection error." << std::endl;
    }

    // Send command
    LibSerial::DataBuffer writeDataBuffer(cmd, cmd + 5);
    serial_conn_.FlushIOBuffers();
    serial_conn_.Write(writeDataBuffer);

    LibSerial::DataBuffer readDataBuffer(5);

    try
    {
        serial_conn_.Read(readDataBuffer);

        *encoder_maxon_left = static_cast<double>(readDataBuffer[0] << 8 | readDataBuffer[1]);
        *encoder_maxon_right = static_cast<double>(readDataBuffer[2] << 8 | readDataBuffer[3]);

        // Decode flags
        uint8_t flags = readDataBuffer[4];
        // *are_brushes_activated = static_cast<double> (flags & 0x01);
        // *is_unload_routine_activated = static_cast<double>((flags >> 1) & 0x01);
        // *is_lift_routine_authorized = static_cast<double>((flags >> 2) & 0x01);

        if (print_output)
        {
            std::cout << "[ROBOCOPS_CONTROL] Encoder Left: " << *encoder_maxon_left
                      << ", Right: " << *encoder_maxon_right << std::endl;
            //   << "Brushes: " << *are_brushes_activated
            //   << ", Unload: " << *is_unload_routine_activated
            //   << ", Lift authorized: " << *is_lift_routine_authorized
        }
    }
    catch (const LibSerial::ReadTimeout &)
    {
        std::cerr << "[ROBOCOPS_CONTROL] Read timeout occurred." << std::endl;
    }
}

/**
 * @brief Requests and parses encoder speed values from the Arduino.
 * 
 * The Arduino is expected to return two space-separated RPM values. They are converted
 * to radians per second and stored in the provided pointers.
 * 
 * @param val_1 Pointer to store the first encoder value (rad/s).
 * @param val_2 Pointer to store the second encoder value (rad/s).
 */
void ArduinoComms::read_encoder_values(double *val_1, double *val_2)
{
    std::string response = send_msg("e\n");

    std::string delimiter = " ";
    size_t del_pos = response.find(delimiter);
    std::string token_1 = response.substr(0, del_pos);
    std::string token_2 = response.substr(del_pos + delimiter.length());

    *val_1 = rpm_to_rad_per_sec(std::atol(token_1.c_str()));
    *val_2 = rpm_to_rad_per_sec(std::atol(token_2.c_str()));
}

/**
 * @brief Sends motor speed commands to the Arduino.
 * 
 * The command format is "m <rear_motor_speed> <left_motor_speed>\n".
 * 
 * @param rear_motor_speed Speed command for the rear motor.
 * @param left_motor_speed Speed command for the left motor.
 */
void ArduinoComms::set_motor_values(int rear_motor_speed, int left_motor_speed)
{
    std::stringstream ss;
    ss << "m " << rear_motor_speed << " " << left_motor_speed << "\n";
    send_msg(ss.str());
}
