#include "dri_driver.h"

const int DRI_MOTOR_EN_PINS[DRI_MOTOR_COUNT] = {
  //9, // BRUSH_LEFT
  //10 // BRUSH_RIGHT
};

const int DRI_MOTOR_PWM_PINS[DRI_MOTOR_COUNT] = {
 // 9, // BRUSH_LEFT
  //10 // BRUSH_RIGHT
};

const int IS_INVERSED_DRI_MOTOR[DRI_MOTOR_COUNT] = {
  //0, // BRUSH_LEFT
  //0 // BRUSH_RIGHT
};

double dri_target_speeds[DRI_MOTOR_COUNT];

void init_dri_motor_drivers() {
    for (int i = 0; i < DRI_MOTOR_COUNT; i++) {
        pinMode(DRI_MOTOR_EN_PINS[i], OUTPUT);
        pinMode(DRI_MOTOR_PWM_PINS[i], OUTPUT);
        
        // Disable motor at startup
        digitalWrite(DRI_MOTOR_EN_PINS[i], LOW);
        analogWrite(DRI_MOTOR_PWM_PINS[i], DRI_MIN_PWM);

        dri_target_speeds[i] = DRI_MIN_PWM;
    }
}

void set_dri_motor_state(dri_motor_position motor, int pwm){
    // Clamp PWM value to allowed range
    if(pwm < DRI_MIN_PWM)
        pwm = DRI_MIN_PWM;
    if(pwm > DRI_MAX_PWM)
        pwm = DRI_MAX_PWM;

    analogWrite(DRI_MOTOR_PWM_PINS[motor], pwm);

    if(pwm == DRI_MIN_PWM){
      digitalWrite(DRI_MOTOR_EN_PINS[motor], LOW);
    } else {
      digitalWrite(DRI_MOTOR_EN_PINS[motor], HIGH);
    }
}
