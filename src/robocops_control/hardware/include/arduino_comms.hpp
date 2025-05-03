#ifndef ARDUINOCOMMS_HPP
#define ARDUINOCOMMS_HPP

#include <string>
#include <libserial/SerialPort.h>

class ArduinoComms
{
public:
    ArduinoComms() = default;

    void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);
    void disconnect();
    bool connected() const;

    // todo send raw byte msg

    std::string send_msg(const std::string &msg_to_send, bool print_output = false);
    void read_encoder_values(double *val_1, double *val_2);
    void set_motor_values(int val_1, int val_2);

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;

    LibSerial::BaudRate convert_baud_rate(int baud_rate);
};

#endif // ARDUINOCOMMS_HPP
