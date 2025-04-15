#ifndef COMMANDS_H
#define COMMANDS_H

/**
 * @file commands.h
 * @brief Command definitions for motor control.
 *
 * This header defines the single-character command codes used to control
 * motors via a serial or other communication interface.
 *
 * Each command is expected to be followed by a set of parameters depending
 * on the number of motors (MOTOR_COUNT).
 */

// Command to set target motor speeds (between MIN_RPM and MAX_RPM, see maxon_driver.h).
// Usage: 'm' MOTOR_SPEED_1, MOTOR_SPEED_2, ..., MOTOR_SPEED_(MOTOR_COUNT)
#define MOTOR_SPEEDS 'm'

// Command to set target motor speeds grouped: left wheels and right wheels are one group (between MIN_RPM and MAX_RPM, see maxon_driver.h).
// Usage: 'g' LEFT_MOTORS_SPEED, RIGHT_MOTORS_SPEED
#define MOTOR_GROUPED_SPEEDS 'g'

// Command to get the current encoder values in RPM
// Usage: 'e' 
#define ENCODERS_FEEDBACK 'e'

// Command to define the PID values
// Usage 'p k_p k_d k_i k_o'
#define PID_VALUES 'p'

#endif // COMMANDS_H
