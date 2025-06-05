#ifndef DRI_H
#define DRI_H

#include <Arduino.h>

#define DRI_MOTOR_COUNT 2

#define DRI_MIN_PWM 0
#define DRI_MAX_PWM 255

enum dri_motor_position {
    CONVOYER_BACK = 0,
    CONVOYER_LIFT = 1
};

extern int dri_target_speeds[DRI_MOTOR_COUNT];

void init_dri_motor_drivers();
void set_dri_motor_state(dri_motor_position motor, int direction, int pwm);

#endif // DRI_H
