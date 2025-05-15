#include "servo_driver.h"

Servo servoMotors[SERVO_MOTOR_COUNT];

const int SERVO_MOTOR_PINS[SERVO_MOTOR_COUNT] = {
    8 // SERVO_LEFT
       // SERVO_RIGHT
};

void init_servo_motors() {
    for (int i = 0; i < SERVO_MOTOR_COUNT; i++) {
        servoMotors[i].attach(SERVO_MOTOR_PINS[i]);
        servoMotors[i].write(90); // Center position on startup
    }
}

void set_servo_motor_angle(servo_motor_position motor, int angle) {
    // Clamp angle between 0 and 180
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    servoMotors[motor].write(angle);
}
