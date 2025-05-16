#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <Arduino.h>
#include <Servo.h>

#define SERVO_MOTOR_COUNT 1

enum servo_motor_position {
    UNLOAD_DOOR_MOTOR = 0,
    UNLOAD_LATCH_MOTOR = 0
};

extern const int SERVO_MOTOR_PINS[SERVO_MOTOR_COUNT];

void init_servo_motors_drivers();
void set_servo_motor_angle(servo_motor_position motor, int angle);

#endif // SERVO_DRIVER_H
