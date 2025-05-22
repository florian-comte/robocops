#include "l298n_driver.h"

const int L298N_MOTOR_IN1_PINS[L298N_MOTOR_COUNT] = {
  44, // BRUSH_LEFT
  26// BRUSH_RIGHT

};

const int L298N_MOTOR_IN2_PINS[L298N_MOTOR_COUNT] = {
  28, // BRUSH_LEFT
  50// BRUSH_RIGHT
};

const int L298N_MOTOR_PWM_PINS[L298N_MOTOR_COUNT] = {
  9, // BRUSH_LEFT
  10 // BRUSH_RIGHT
};

const int IS_INVERSED_L298N_MOTOR[L298N_MOTOR_COUNT] = {
  0, // BRUSH_LEFT
  0 // BRUSH_RIGHT
};

double l298n_target_speeds[L298N_MOTOR_COUNT];

void init_l298n_motor_drivers() {
    for (int i = 0; i < L298N_MOTOR_COUNT; i++) {
        pinMode(L298N_MOTOR_PWM_PINS[i], OUTPUT);
        pinMode(L298N_MOTOR_IN1_PINS[i], OUTPUT);
        pinMode(L298N_MOTOR_IN2_PINS[i], OUTPUT);
        
        // Disable motor at startup
        analogWrite(L298N_MOTOR_PWM_PINS[i], L298N_MIN_PWM);
        digitalWrite(L298N_MOTOR_IN1_PINS[i], LOW);
        digitalWrite(L298N_MOTOR_IN2_PINS[i], LOW);

        l298n_target_speeds[i] = L298N_MIN_PWM;
    }
}

void set_l298n_motor_state(l298n_motor_position motor, int direction, int pwm){
    // Clamp PWM value to allowed range
    if(pwm < L298N_MIN_PWM)
        pwm = L298N_MIN_PWM;
    if(pwm > L298N_MAX_PWM)
        pwm = L298N_MAX_PWM;

    analogWrite(L298N_MOTOR_PWM_PINS[motor], pwm);

    if(pwm == L298N_MIN_PWM){
      digitalWrite(L298N_MOTOR_IN1_PINS[motor], LOW);
      digitalWrite(L298N_MOTOR_IN2_PINS[motor], LOW);
    } else {
      // Set motor direction
      if (direction == 1) {
          digitalWrite(L298N_MOTOR_IN1_PINS[motor], !IS_INVERSED_L298N_MOTOR[motor] ? LOW : HIGH);
          digitalWrite(L298N_MOTOR_IN2_PINS[motor], !IS_INVERSED_L298N_MOTOR[motor] ? HIGH : LOW);
      } else {
          digitalWrite(L298N_MOTOR_IN1_PINS[motor], !IS_INVERSED_L298N_MOTOR[motor] ? HIGH : LOW);
          digitalWrite(L298N_MOTOR_IN2_PINS[motor], !IS_INVERSED_L298N_MOTOR[motor] ? LOW : HIGH);
      }
    }
}
