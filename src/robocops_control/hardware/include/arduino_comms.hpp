#ifndef ARDUINOCOMMS_HPP
#define ARDUINOCOMMS_HPP

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

    // TODO: Implement sending raw byte messages

    /**
     * @brief Sends a string message over the serial connection and returns the response.
     * 
     * @param msg_to_send The message to send.
     * @param print_output If true, prints the response to standard output.
     * @return The response string from the Arduino.
     */
    std::string send_msg(const std::string &msg_to_send, bool print_output = false);

    /**
     * @brief Reads encoder values from the Arduino.
     * 
     * @param val_1 Pointer to store the first encoder value.
     * @param val_2 Pointer to store the second encoder value.
     */
    void read_encoder_values(double *val_1, double *val_2);

    /**
     * @brief Sends motor control values to the Arduino.
     * 
     * @param val_1 Value for motor 1.
     * @param val_2 Value for motor 2.
     */
    void set_motor_values(int val_1, int val_2);

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

#endif // ARDUINOCOMMS_HPP
