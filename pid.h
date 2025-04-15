#ifndef PID_H
#define PID_H

/**
 * @file pid.h
 * @brief Defines variables and constants used for PID speed control of Maxon motors.
 *
 * This header contains the proportional, integral, and derivative constants,
 * as well as runtime variables used to perform PID-based speed regulation
 * of each motor in the system.
 */

#include "maxon_encoder.h"
#include <Arduino.h>
#include "utils.h"

extern int is_moving[MOTOR_COUNT];
extern float target_speeds[MOTOR_COUNT];

extern int k_p;
extern int k_d;
extern int k_i;
extern int k_o;

void reset_pid(motor_position motor);
void do_pid(motor_position motor, float speed_target);
void update_pid(motor_position motor);

#endif // PID_H
