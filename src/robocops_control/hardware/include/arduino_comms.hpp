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
                    bool brush,
                    bool unload,
                    bool lift,
                    double *encoder_left,
                    double *encoder_right,
                    bool print_output = false);

private:
  int serial_fd_ = -1;
  int timeout_ms_ = 100;
};
