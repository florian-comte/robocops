#ifndef MAXON_DRIVER_H
#define MAXON_DRIVER_H

/**
 * @file maxon_driver.h
 * @brief Motor driver interface for controlling Maxon motors on Arduino.
 *
 * This header defines motor positions, control pin mappings, and function
 * prototypes for initializing and controlling Maxon motor drivers.
 */

#include <Arduino.h>
#include "utils.h"

/**
 * @brief Minimum PWM value to activate the motor.
 */
#define MIN_PWM 30  // Approx. 10% of full scale (255), defined in firmware of maxon driver

/**
 * @brief Maximum PWM value to limit power to the motor.
 */
#define MAX_PWM 220 // Approx. 90% of full scale (255), defined in firmware of maxon driver

/**
 * @brief Maximum speed value in RPM.
 */
#define MAX_MOTOR_SPEED 7500

/**
 * @enum motor_position
 * @brief Enumerates the positions of motors on a 4-wheeled robot.
 */
enum motor_position {
    //FRONT_LEFT = 0,
    //FRONT_RIGHT,
    //REAR_LEFT,
    REAR_RIGHT = 0,
    MOTOR_COUNT  ///< Total number of motors
};

/**
 * @brief Initialize all motor drivers (e.g., pin modes).
 */
void init_motor_drivers();

/**
 * @brief Set the state of an individual motor.
 *
 * @param motor     Motor to control (use motor_position enum).
 * @param enabled   Whether the motor should be active (1) or off (0).
 * @param direction Direction of rotation (1 for forward, -1 for reverse).
 * @param pwm       PWM value (within MIN_PWM and MAX_PWM).
 */
void set_motor_state(motor_position motor, int enabled, int direction, int pwm);

#endif // MAXON_DRIVER_H
