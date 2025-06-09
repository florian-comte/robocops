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
    tty.c_cc[VMIN] = 0;                                 // Non-blocking read
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
                                bool capture_activate,
                                bool unload_activate,
                                bool button_activate,
                                bool slope_up_activate,
                                bool slope_down_activate,
                                bool emergency_activate,
                                double *encoder_left,
                                double *encoder_right,
                                bool *capture_active,
                                bool *unload_active,
                                bool *button_active,
                                bool *slope_up_active,
                                bool *slope_down_active,
                                bool *emergency_active,
                                int *nb_captured_duplos,
                                int *back_ultrasound_distance,
                                bool print_output)
{
    if (!connected())
    {
        std::cerr << "[Serial] Port not connected." << std::endl;
        return;
    }

    // Prepare command buffer with markers
    uint8_t cmd[7]; // START_MARKER + 5 bytes payload + END_MARKER
    cmd[0] = 0xAA;  // START_MARKER

    // Offset motor speeds to avoid negative numbers (since Arduino expects unsigned)
    maxon_left += 10000;
    maxon_right += 10000;

    // Encode speeds into 2 bytes each (big-endian)
    cmd[1] = (maxon_left >> 8) & 0xFF;
    cmd[2] = maxon_left & 0xFF;
    cmd[3] = (maxon_right >> 8) & 0xFF;
    cmd[4] = maxon_right & 0xFF;

    // Encode control signals into bit flags in cmd[5]
    cmd[5] = 0;
    cmd[5] |= (capture_activate & 1);
    cmd[5] |= ((unload_activate & 1) << 1);
    cmd[5] |= ((button_activate & 1) << 2);
    cmd[5] |= ((slope_up_activate & 1) << 3);
    cmd[5] |= ((slope_down_activate & 1) << 4);
    cmd[5] |= ((emergency_activate & 1) << 5);

    cmd[6] = 0x55; // END_MARKER

    fd_set write_fds;
    FD_ZERO(&write_fds);
    FD_SET(serial_fd_, &write_fds);
    struct timeval tv_write = {timeout_ms_ / 1000, (timeout_ms_ % 1000) * 1000};

    if (select(serial_fd_ + 1, nullptr, &write_fds, nullptr, &tv_write) > 0)
    {
        if (write(serial_fd_, cmd, 7) != 7)
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

    // Now read the response with markers
    uint8_t response[13]; // START_MARKER + 11 bytes payload + END_MARKER
    int bytes_read = 0;
    bool start_marker_found = false;
    bool end_marker_found = false;
    // struct timeval tv_read = {timeout_ms_ / 1000, (timeout_ms_ % 1000) * 1000};
    time_t start_time = time(nullptr);

    while (!end_marker_found && (time(nullptr) - start_time) * 1000 < timeout_ms_)
    {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(serial_fd_, &read_fds);
        struct timeval tv = {0, 10000}; // 10ms timeout for each select call

        if (select(serial_fd_ + 1, &read_fds, nullptr, nullptr, &tv) > 0)
        {
            std::cout << "[Serial] Received byte: 0x"
                      << std::hex << static_cast<int>(byte) << std::dec << std::endl;
            uint8_t byte;
            int n = read(serial_fd_, &byte, 1);
            if (n == 1)
            {
                if (!start_marker_found)
                {
                    if (byte == 0xAA)
                    {
                        start_marker_found = true;
                        bytes_read = 0;
                    }
                }
                else
                {
                    if (byte == 0x55)
                    {
                        end_marker_found = true;
                    }
                    else
                    {
                        if (bytes_read < 11)
                        {
                            response[bytes_read++] = byte;
                        }
                        else
                        {
                            // Buffer overflow, reset
                            start_marker_found = false;
                        }
                    }
                }
            }
        }
    }

    if (!end_marker_found || bytes_read != 11)
    {
        std::cerr << "[Serial] Read timeout or incomplete response" << std::endl;
        return;
    }

    // Extract encoder values
    *encoder_left = static_cast<int16_t>((response[0] << 8) | response[1]) - 10000;
    *encoder_right = static_cast<int16_t>((response[2] << 8) | response[3]) - 10000;

    // Extract GPIO states from response[4]
    *capture_active = static_cast<bool>((response[4] >> 0) & 1);
    *unload_active = static_cast<bool>((response[4] >> 1) & 1);
    *button_active = static_cast<bool>((response[4] >> 2) & 1);
    *slope_up_active = static_cast<bool>((response[4] >> 3) & 1);
    *slope_down_active = static_cast<bool>((response[4] >> 4) & 1);
    *emergency_active = static_cast<bool>((response[4] >> 5) & 1);

    // Extract nb duplos
    *nb_captured_duplos = static_cast<int16_t>((response[5] << 8) | response[6]);

    // Extract other values
    *back_ultrasound_distance = static_cast<int16_t>((response[7] << 8) | response[8]);
    int16_t other_value = static_cast<int16_t>((response[9] << 8) | response[10]);

    // Print debug if wanted
    if (print_output)
    {
        std::cout << "[Serial] Sent command: ";
        for (int i = 0; i < 7; ++i)
            std::cout << "0x" << std::hex << static_cast<int>(cmd[i]) << " ";

        std::cout << std::dec;
        std::cout << "\n[Serial] Encoders: L=" << *encoder_left
                  << " R=" << *encoder_right << std::endl;

        std::cout << "[Serial] GPIO States: "
                  << "Capture: " << *capture_active
                  << ", Unload: " << *unload_active
                  << ", Button Active: " << *button_active
                  << ", Slope up active: " << *slope_up_active
                  << ", Slope down active: " << *slope_down_active
                  << ", Emergency active: " << *emergency_active
                  << ", Nb duplos captured: " << *nb_captured_duplos
                  << ", Back ultrasound distance: " << *back_ultrasound_distance
                  << std::endl;

        std::cout << "[Serial] Debug values: "
                  << "Other value 1: " << other_value
                  << std::endl;
    }
}