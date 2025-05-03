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
