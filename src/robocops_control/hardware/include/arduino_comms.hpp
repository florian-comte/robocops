#pragma once

#include <string>

class ArduinoComms
{
public:
    void connect(const std::string &device, int timeout_ms = 100);
    void disconnect();
    bool connected() const;

    void send_command(int16_t maxon_left,
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
                      bool print_output = false);

private:
    int serial_fd_ = -1;
    int timeout_ms_ = 100;
};