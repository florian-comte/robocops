#ifndef MAXON_ENCODER_H
#define MAXON_ENCODER_H

/**
 * @file maxon_encoder.h
 * @brief Interface for reading speed feedback from Maxon motor encoders.
 *
 * This header provides constants and functions to interface with the analog
 * encoder outputs from Maxon motor drivers. It allows reading the speed of
 * each motor using analog voltage levels corresponding to RPM values.
 */

#include <Arduino.h>
#include "maxon_driver.h"

/**
 * @brief Minimum encoder speed in RPM.
 * Corresponds to 0V from the encoder output, defined in firmware of maxon driver.
 */
#define MAXON_MIN_ENCODER_SPEED -10000

/**
 * @brief Maximum encoder speed in RPM.
 * Corresponds to 4V from the encoder output, defined in firmware of maxon driver.
 */
#define MAXON_MAX_ENCODER_SPEED 10000

/**
 * @brief Minimum voltage from encoder (in Volts), defined in firmware of maxon driver.
 */
#define MAXON_MIN_ENCODER_VOLTAGE 0

/**
 * @brief Maximum voltage from encoder (in Volts), defined in firmware of maxon driver.
 */
#define MAXON_MAX_ENCODER_VOLTAGE 4

// @brief The maximum value between 0 and 1023 of the analogRead. Here is it 1023*4/5=818.4 because we decided to put 4v in maxon firmware
#define MAXON_MAX_ANALOG_VALUE 818.4

/**
 * @brief Initialize the encoder system (e.g., configure analog input pins).
 */
void init_maxon_motor_encoders();

/**
 * @brief Read the current speed of a motor from its encoder.
 *
 * The encoder outputs a voltage between MIN_ENCODER_VOLTAGE and MAX_ENCODER_VOLTAGE, corresponding linearly to
 * a speed between MIN_ENCODER_SPEED and MAX_ENCODER_SPEED RPM, defined in firmware of maxon driver.
 *
 * @param motor The motor whose encoder speed should be read.
 * @return The motor speed in RPM as a floating point value.
 */
float read_maxon_encoder(maxon_motor_position motor);

#endif // MAXON_ENCODER_H
