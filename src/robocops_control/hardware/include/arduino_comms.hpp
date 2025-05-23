#pragma once

#include <string>
#include <serial/serial.h>

class ArduinoComms
{
public:
    void connect(const std::string &serial_device, int32_t timeout_ms);
    void disconnect();
    bool connected() const;

    void send_command(int16_t maxon_left,
                      int16_t maxon_right,
                      bool brush_signal,
                      bool activate_unload_routine,
                      bool authorized_lift_routine,
                      double *encoder_maxon_left,
                      double *encoder_maxon_right,
                      bool print_output = false);

private:
    serial::Serial serial_;
    int32_t timeout_ms_;
};
