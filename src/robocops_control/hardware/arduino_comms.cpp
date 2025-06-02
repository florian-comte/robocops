#include "arduino_comms.hpp"
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <algorithm>
#include <cstdint>
#include <sys/select.h>

void ArduinoComms::connect(const std::string &serial_device, int32_t timeout_ms)
{
    timeout_ms_ = timeout_ms;

    // Open the serial device with read/write access, no controlling terminal, and synchronous I/O
    // Here serial_fd_ is the file descriptor, small integer that uniquely identifies an open file or I/O resource within a process
    serial_fd_ = open(serial_device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd_ < 0)
    {
        perror("Error opening serial port");
        return;
    }

    // Create a termios struct and clear it
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(serial_fd_, &tty) != 0)
    {
        perror("Error from tcgetattr");
        return;
    }

    // Set baud rate to 57600 (input and output)
    cfsetospeed(&tty, B57600);
    cfsetispeed(&tty, B57600);

    // Configure terminal settings:
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;         // 8 data bits
    tty.c_iflag &= ~IGNBRK;                             // Disable break processing
    tty.c_lflag = 0;                                    // No signaling chars, no echo, no canonical input
    tty.c_oflag = 0;                                    // No remapping, no delays
    tty.c_cc[VMIN] = 5;                                 // Minimum number of bytes to read
    tty.c_cc[VTIME] = std::min(timeout_ms_ / 100, 255); // Read timeout in 0.1s units

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // No software flow control
    tty.c_cflag |= (CLOCAL | CREAD);        // Enable receiver, ignore modem control lines
    tty.c_cflag &= ~(PARENB | PARODD);      // No parity
    tty.c_cflag &= ~CSTOPB;                 // One stop bit
    tty.c_cflag &= ~CRTSCTS;                // No hardware flow control

    // Apply the settings immediately
    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
    {
        perror("Error from tcsetattr");
        return;
    }
}

void ArduinoComms::disconnect()
{
    if (connected())
    {
        close(serial_fd_);
        serial_fd_ = -1;
    }
}

bool ArduinoComms::connected() const
{
    return serial_fd_ >= 0;
}

void ArduinoComms::send_command(int16_t maxon_left,
                                int16_t maxon_right,
                                bool brushes_activate,
                                bool unload_activate,
                                bool lift_authorize,
                                bool button_activate,
                                bool emergency_activate,
                                double *encoder_left,
                                double *encoder_right,
                                bool *lift_authorized,
                                bool *lift_active,
                                bool *unload_active,
                                bool *brushes_active,
                                bool *button_active,
                                bool *emergency_active,
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
    cmd[4] |= ((button_activate & 1) << 3);
    cmd[4] |= ((emergency_activate & 1) << 4);

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
        uint8_t response[9];
        int n = read(serial_fd_, response, 9);
        if (n != 9)
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
        *button_active = static_cast<bool>((response[4] >> 4) & 1);
        *emergency_active = static_cast<bool>((response[4] >> 5) & 1);

        uint16_t distance_mm = static_cast<int16_t>((response[5] << 8) | response[6]);
        double distance_cm = distance_mm / 10.0;

        int16_t lift_convoyer_speed = static_cast<int16_t>((response[7] << 8) | response[8]);

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
                      << ", Button Active=" << *button_active
                      << ", Emergency active: " << *emergency_active
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