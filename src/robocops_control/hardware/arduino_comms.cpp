#include "arduino_comms.hpp"
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <algorithm>
#include <cstdint>
#include <sys/select.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <iostream>
#include <cstring>
#include "ArduinoComms.h"

void ArduinoComms::connect(const std::string &device, int timeout_ms)
{
    disconnect(); 

    serial_fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd_ < 0)
    {
        std::cerr << "Error opening " << device << ": " << strerror(errno) << std::endl;
        return;
    }

    this->timeout_ms_ = timeout_ms;

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(serial_fd_, &tty) != 0)
    {
        std::cerr << "Error from tcgetattr: " << strerror(errno) << std::endl;
        disconnect();
        return;
    }

    // Set baud rate (57600)
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                     // disable break processing
    tty.c_lflag = 0;                            // no signaling chars, no echo
    tty.c_oflag = 0;                            // no remapping, no delays
    tty.c_cc[VMIN] = 0;                         // non-blocking read
    tty.c_cc[VTIME] = timeout_ms_ / 100;        // read timeout (tenths of second)

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);        // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // no parity
    tty.c_cflag &= ~CSTOPB;                 // one stop bit
    tty.c_cflag &= ~CRTSCTS;                // no flow control

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
    {
        std::cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
        disconnect();
        return;
    }

    // Flush any old data
    tcflush(serial_fd_, TCIOFLUSH);
}

void ArduinoComms::disconnect()
{
    if (serial_fd_ >= 0)
    {
        close(serial_fd_);
        serial_fd_ = -1;
    }
}

bool ArduinoComms::connected() const
{
    return serial_fd_ >= 0;
}

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <cstdint>
#include "ArduinoComms.h"

#define START_MARKER 0xAA
#define END_MARKER 0x55
#define COMMANDS_BUFFER_SIZE 5
#define STATES_BUFFER_SIZE 11

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
    if (serial_fd_ < 0)
    {
        std::cerr << "Serial port not open.\n";
        return;
    }

    uint8_t tx_buffer[COMMANDS_BUFFER_SIZE + 2]; // command + markers
    uint8_t rx_buffer[STATES_BUFFER_SIZE + 2];   // state + markers

    // Format command (offset by 10000 as required)
    int16_t adjusted_left = maxon_left + 10000;
    int16_t adjusted_right = maxon_right + 10000;

    tx_buffer[0] = START_MARKER;
    tx_buffer[1] = (adjusted_left >> 8) & 0xFF;
    tx_buffer[2] = adjusted_left & 0xFF;
    tx_buffer[3] = (adjusted_right >> 8) & 0xFF;
    tx_buffer[4] = adjusted_right & 0xFF;

    // Pack control bits
    uint8_t flags = 0;
    flags |= (capture_activate & 0x01) << 0;
    flags |= (unload_activate & 0x01) << 1;
    flags |= (button_activate & 0x01) << 2;
    flags |= (slope_up_activate & 0x01) << 3;
    flags |= (slope_down_activate & 0x01) << 4;
    flags |= (emergency_activate & 0x01) << 5;
    tx_buffer[5] = flags;
    tx_buffer[6] = END_MARKER;

    // Write to serial
    if (write(serial_fd_, tx_buffer, sizeof(tx_buffer)) != sizeof(tx_buffer))
    {
        std::cerr << "Failed to write command to serial.\n";
        return;
    }

    // Read response (wait until buffer fills or timeout)
    uint8_t b;
    bool started = false;
    int index = 0;
    auto start_time = static_cast<int>(time(nullptr));

    while (true)
    {
        int n = read(serial_fd_, &b, 1);
        if (n <= 0)
        {
            if (static_cast<int>(time(nullptr)) - start_time > timeout_ms_ / 1000)
            {
                std::cerr << "Timeout waiting for response.\n";
                return;
            }
            continue;
        }

        if (!started)
        {
            if (b == START_MARKER)
            {
                started = true;
                index = 0;
            }
        }
        else
        {
            rx_buffer[index++] = b;
            if (b == END_MARKER || index >= STATES_BUFFER_SIZE + 1)
            {
                break;
            }
        }
    }

    if (rx_buffer[index - 1] != END_MARKER || index != STATES_BUFFER_SIZE + 1)
    {
        std::cerr << "Invalid response.\n";
        return;
    }

    // Parse received data
    uint8_t *data = rx_buffer;

    int16_t left_speed = (data[0] << 8) | data[1];
    int16_t right_speed = (data[2] << 8) | data[3];

    *encoder_left = left_speed - 10000;
    *encoder_right = right_speed - 10000;

    uint8_t status = data[4];
    *capture_active = (status >> 0) & 0x01;
    *unload_active = (status >> 1) & 0x01;
    *button_active = (status >> 2) & 0x01;
    *slope_up_active = (status >> 3) & 0x01;
    *slope_down_active = (status >> 4) & 0x01;
    *emergency_active = (status >> 5) & 0x01;

    *nb_captured_duplos = (data[5] << 8) | data[6];
    *back_ultrasound_distance = (data[7] << 8) | data[8];
    // state_other (data[9-10]) is unused here

    if (print_output)
    {
        std::cout << "Left: " << *encoder_left << " Right: " << *encoder_right << "\n";
        std::cout << "Capture: " << *capture_active << " Unload: " << *unload_active
                  << " Button: " << *button_active << " SlopeUp: " << *slope_up_active
                  << " SlopeDown: " << *slope_down_active << " Emergency: " << *emergency_active << "\n";
        std::cout << "Duplos: " << *nb_captured_duplos << " Ultrasound: " << *back_ultrasound_distance << "\n";
    }
}
