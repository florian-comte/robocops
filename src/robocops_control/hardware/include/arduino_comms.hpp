#ifndef ROBOCOPS_CONTROL_ARDUINOCOMMS_HPP
#define ROBOCOPS_CONTROL_ARDUINOCOMMS_HPP

#include <string>
#include <libserial/SerialPort.h>

/**
 * @class ArduinoComms
 * @brief Handles serial communication with an Arduino using LibSerial.
 */
class ArduinoComms
{
public:
    /**
     * @brief Default constructor.
     */
    ArduinoComms() = default;

    /**
     * @brief Establishes a serial connection to an Arduino.
     *
     * @param serial_device The path to the serial device (e.g., "/dev/ttyUSB0").
     * @param baud_rate The baud rate for communication (e.g., 9600).
     * @param timeout_ms Timeout duration in milliseconds for read/write operations.
     */
    void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);

    /**
     * @brief Closes the serial connection if it is open.
     */
    void disconnect();

    /**
     * @brief Checks if the serial connection is active.
     *
     * @return true if connected, false otherwise.
     */
    bool connected() const;

    void send_command(int16_t maxon_left,
                      int16_t maxon_right,
                      bool brush_signal,
                      bool activate_unload_routine,
                      bool authorized_lift_routine,

                      double *encoder_maxon_left,
                      double *encoder_maxon_right,
                    //   double *are_brushes_activated,
                    //   double *is_unload_routine_activated,
                    //   double *is_lift_routine_authorized,

                      bool print_output);

private:
    /**
     * @brief LibSerial SerialPort instance used for communication.
     */
    LibSerial::SerialPort serial_conn_;

    /**
     * @brief Timeout value in milliseconds for communication operations.
     */
    int timeout_ms_;

    /**
     * @brief Converts an integer baud rate into a LibSerial::BaudRate enum value.
     *
     * @param baud_rate The integer baud rate (e.g., 9600, 115200).
     * @return The corresponding LibSerial::BaudRate enum.
     */
    LibSerial::BaudRate convert_baud_rate(int baud_rate);
};

#endif // ROBOCOPS_CONTROL_ARDUINOCOMMS_HPP
