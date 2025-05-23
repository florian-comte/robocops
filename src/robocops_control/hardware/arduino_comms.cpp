#include "arduino_comms.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <cstring>
#include <sys/ioctl.h>

/**
 * Opens and configures the serial port.
 */
void ArduinoComms::connect(const std::string &device, int baud_rate, int timeout_ms)
{
    timeout_ms_ = timeout_ms;

    serial_fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
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

    cfsetospeed(&tty, B57600); // hardcoded baudrate
    cfsetispeed(&tty, B57600);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                     // disable break processing
    tty.c_lflag = 0;                            // no signaling chars, no echo
    tty.c_oflag = 0;                            // no remapping, no delays
    tty.c_cc[VMIN] = 5;                         // read blocks until 5 bytes
    tty.c_cc[VTIME] = timeout_ms_ / 100;        // timeout in deciseconds

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);        // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // no parity
    tty.c_cflag &= ~CSTOPB;                 // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                // no flow control

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
    {
        perror("Error from tcsetattr");
        return;
    }
}

bool ArduinoComms::connected() const
{
    return serial_fd_ >= 0;
}

void ArduinoComms::disconnect()
{
    if (connected())
    {
        close(serial_fd_);
        serial_fd_ = -1;
    }
}

void ArduinoComms::send_command(int16_t maxon_left,
                                int16_t maxon_right,
                                bool brush,
                                bool unload,
                                bool lift,
                                double *encoder_left,
                                double *encoder_right,
                                bool print_output)
{
    if (!connected())
    {
        std::cerr << "[Serial] Port not connected." << std::endl;
        return;
    }

    // Encode
    uint8_t cmd[5];
    uint16_t left_enc = maxon_left + 10000;
    uint16_t right_enc = maxon_right + 10000;

    cmd[0] = (left_enc >> 8) & 0xFF;
    cmd[1] = left_enc & 0xFF;
    cmd[2] = (right_enc >> 8) & 0xFF;
    cmd[3] = right_enc & 0xFF;
    cmd[4] = 0;
    cmd[4] |= brush ? 0x01 : 0x00;
    cmd[4] |= unload ? 0x02 : 0x00;
    cmd[4] |= lift ? 0x04 : 0x00;

    if (write(serial_fd_, cmd, 5) != 5)
    {
        perror("[Serial] Failed to write full command");
        return;
    }

    // Read 5-byte response
    uint8_t response[5];
    int n = read(serial_fd_, response, 5);
    if (n != 5)
    {
        perror("[Serial] Failed to read response");
        return;
    }

    *encoder_left = static_cast<int16_t>((response[0] << 8) | response[1]);
    *encoder_right = static_cast<int16_t>((response[2] << 8) | response[3]);

    if (print_output)
    {
        std::cout << "[Serial] Sent command: ";
        for (int i = 0; i < 5; ++i)
            std::cout << "0x" << std::hex << static_cast<int>(cmd[i]) << " ";
        std::cout << std::dec << "\n[Serial] Encoders: L=" << *encoder_left << " R=" << *encoder_right << std::endl;
    }
}
