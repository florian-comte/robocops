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
    if (!connected())
    {
        std::cerr << "[Serial] Port not connected." << std::endl;
        return;
    }

    uint8_t cmd[5];

    // Offset motor speeds to avoid negative numbers (since Arduino expects unsigned)
    maxon_left += 10000;
    maxon_right += 10000;

    // Encode speeds into 2 bytes each (big-endian)
    cmd[0] = (maxon_left >> 8) & 0xFF;
    cmd[1] = maxon_left & 0xFF;
    cmd[2] = (maxon_right >> 8) & 0xFF;
    cmd[3] = maxon_right & 0xFF;

    // Encode control signals into bit flags in cmd[4]
    cmd[4] = 0;
    cmd[4] |= (brushes_activate & 1);
    cmd[4] |= ((unload_activate & 1) << 1);
    cmd[4] |= ((lift_authorize & 1) << 2);

    fd_set write_fds;
    FD_ZERO(&write_fds);
    FD_SET(serial_fd_, &write_fds);
    struct timeval tv_write = {timeout_ms_ / 1000, (timeout_ms_ % 1000) * 1000};

    if (select(serial_fd_ + 1, nullptr, &write_fds, nullptr, &tv_write) > 0)
    {
        if (write(serial_fd_, cmd, 5) != 5)
        {
            perror("[Serial] Failed to write full command");
            return;
        }
    }
    else
    {
        std::cerr << "[Serial] Write timeout or error" << std::endl;
        return;
    }

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(serial_fd_, &read_fds);
    struct timeval tv_read = {timeout_ms_ / 1000, (timeout_ms_ % 1000) * 1000};

    if (select(serial_fd_ + 1, &read_fds, nullptr, nullptr, &tv_read) > 0)
    {
        uint8_t response[10];
        int n = read(serial_fd_, response, 5);
        if (n != 10)
        {
            perror("[Serial] Failed to read full response");
            return;
        }

        // Extract encoder values
        *encoder_left = static_cast<int16_t>((response[0] << 8) | response[1]);
        *encoder_right = static_cast<int16_t>((response[2] << 8) | response[3]);

        // Extract GPIO states from response[4]
        *brushes_active = static_cast<bool>((response[4] >> 0) & 1);
        *unload_active = static_cast<bool>((response[4] >> 1) & 1);
        *lift_authorized = static_cast<bool>((response[4] >> 2) & 1);
        *lift_active = static_cast<bool>((response[4] >> 3) & 1);

        uint16_t distance_mm = static_cast<int16_t>((response[5] << 8) | response[6]);
        double distance_cm = distance_mm / 10.0;

        uint16_t lift_convoyer_speed = static_cast<int16_t>((response[5] << 8) | response[6]);

        // Print debug if wanted
        if (print_output)
        {
            std::cout << "[Serial] Sent command: ";
            for (int i = 0; i < 5; ++i)
                std::cout << "0x" << std::hex << static_cast<int>(cmd[i]) << " ";

            std::cout << std::dec; // switch back to decimal output
            std::cout << "\n[Serial] Encoders: L=" << *encoder_left
                      << " R=" << *encoder_right << std::endl;

            std::cout << "[Serial] GPIO States: "
                      << "Brushes=" << *brushes_active
                      << ", Unload=" << *unload_active
                      << ", Lift Authorized=" << *lift_authorized
                      << ", Lift Active=" << *lift_active
                      << ", Lift ultrasound distance: " << distance_cm
                      << ", Lift convoyer speed: " << lift_convoyer_speed
                      << std::endl;
        }
    }
    else
    {
        std::cerr << "[Serial] Read timeout or error" << std::endl;
    }
}