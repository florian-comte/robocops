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

/**
 * @brief Minimum PWM value to activate the motor.
 */
#define MAXON_MIN_PWM 30  // Approx. 10% of full scale (255), defined in firmware of maxon driver

/**
 * @brief Maximum PWM value to limit power to the motor.
 */
#define MAXON_MAX_PWM 220 // Approx. 90% of full scale (255), defined in firmware of maxon driver

/**
 * @brief Maximum speed value in RPM.
 */
#define MAXON_MIN_MOTOR_SPEED 0

/**
 * @brief Maximum speed value in RPM.
 */
#define MAXON_MAX_MOTOR_SPEED 10000

// Total number of motors
#define MAXON_MOTOR_COUNT 2

/**
 * @enum motor_position
 * @brief Enumerates the positions of motors on a 4-wheeled robot.
 */
enum maxon_motor_position {
    MAXON_REAR_RIGHT = 0,
    MAXON_REAR_LEFT = 1
    //FRONT_LEFT = 0,
    //FRONT_RIGHT,
};

/**
 * @brief Tell if the motor is inversed
 * True if the motor is inversed, false if not.
 */
extern const int IS_INVERSED_MAXON_MOTOR[MAXON_MOTOR_COUNT];

extern double maxon_target_speeds[MAXON_MOTOR_COUNT];

/**
 * @brief Initialize all motor drivers (e.g., pin modes).
 */
void init_maxon_motor_drivers();

/**
 * @brief Set the state of an individual motor.
 *
 * @param motor     Motor to control (use maxon_motor_position enum).
 * @param enabled   Whether the motor should be active (1) or off (0).
 * @param direction Direction of rotation (1 for forward, -1 for reverse).
 * @param pwm       PWM value (within MIN_PWM and MAX_PWM).
 */
void set_maxon_motor_state(maxon_motor_position motor, int enabled, int direction, int pwm);

#endif // MAXON_DRIVER_H
