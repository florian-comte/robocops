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
// Usage: 's' MOTOR_SPEED_1, MOTOR_SPEED_2, ..., MOTOR_SPEED_(MOTOR_COUNT)
#define MOTOR_SPEEDS   's'

#endif // COMMANDS_H
