#ifndef L298N_H
#define L298N_H

#include <Arduino.h>

#define L298N_MOTOR_COUNT 2

#define L298N_MIN_PWM 0
#define L298N_MAX_PWM 255

enum l298n_motor_position {
    BRUSH_LEFT = 0,
    BRUSH_RIGHT = 1,
};

extern const int IS_INVERSED_L298N_MOTOR[L298N_MOTOR_COUNT];

extern double l298n_target_speeds[L298N_MOTOR_COUNT];

void init_l298n_motor_drivers();
void set_l298n_motor_state(l298n_motor_position motor, int direction, int pwm);

#endif // L298N_H
