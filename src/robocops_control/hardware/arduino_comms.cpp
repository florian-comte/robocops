#include "arduino_comms.hpp"
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <algorithm>
#include <cstdint>

void ArduinoComms::connect(const std::string &serial_device, int32_t timeout_ms)
{
    timeout_ms_ = timeout_ms;

    serial_fd_ = open(serial_device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd_ < 0)
    {
        perror("Error opening serial port");
        return;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(serial_fd_, &tty) != 0)
    {
        perror("Error from tcgetattr");
        return;
    }

    cfsetospeed(&tty, B57600);
    cfsetispeed(&tty, B57600);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 5;
    tty.c_cc[VTIME] = std::min(timeout_ms_ / 100, 255); // Timeout in deciseconds

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

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

#include <sys/select.h>

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

    // Wait for the port to be ready for writing
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

    // Wait for response to be ready
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(serial_fd_, &read_fds);
    struct timeval tv_read = {timeout_ms_ / 1000, (timeout_ms_ % 1000) * 1000};

    if (select(serial_fd_ + 1, &read_fds, nullptr, nullptr, &tv_read) > 0)
    {
        uint8_t response[5];
        int n = read(serial_fd_, response, 5);
        if (n != 5)
        {
            perror("[Serial] Failed to read full response");
            return;
        }

        *encoder_maxon_left = static_cast<int16_t>((response[0] << 8) | response[1]);
        *encoder_maxon_right = static_cast<int16_t>((response[2] << 8) | response[3]);

        if (print_output)
        {
            std::cout << "[Serial] Sent command: ";
            for (int i = 0; i < 5; ++i)
                std::cout << "0x" << std::hex << static_cast<int>(cmd[i]) << " ";
            std::cout << std::dec << "\n[Serial] Encoders: L=" << *encoder_maxon_left << " R=" << *encoder_maxon_right << std::endl;
        }
    }
    else
    {
        std::cerr << "[Serial] Read timeout or error" << std::endl;
    }
}
