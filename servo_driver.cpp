#include "servo_driver.h"

Servo servoMotors[SERVO_MOTOR_COUNT];

const int SERVO_MOTOR_PINS[SERVO_MOTOR_COUNT] = {
    8, // UNLOAD_DOOR_MOTOR
    36 // UNLOAD_LATCH_MOTOR
};

double servo_target_angles[SERVO_MOTOR_COUNT];

void init_servo_motors_drivers() {
    for (int i = 0; i < SERVO_MOTOR_COUNT; i++) {
        servoMotors[i].attach(SERVO_MOTOR_PINS[i]);

        servo_target_angles[i] = 0;
    }
}

void set_servo_motor_angle(servo_motor_position motor, int angle) {
    // Clamp angle between 0 and 180
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    servoMotors[motor].write(angle);
}
