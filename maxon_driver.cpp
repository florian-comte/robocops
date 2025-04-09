#include "maxon_driver.h"

/**
 * @brief Enable pin mapping for each motor.
 * These pins enable or disable motor output.
 */
const int MOTOR_ENABLE_PINS[MOTOR_COUNT] = {
    //40, // FRONT_LEFT
    //41, // FRONT_RIGHT
    //41, // REAR_LEFT
    41  // REAR_RIGHT
};

/**
 * @brief PWM pin mapping for each motor.
 * These pins control motor speed using pulse-width modulation.
 */
const int MOTOR_PWM_PINS[MOTOR_COUNT] = {
    //40, // FRONT_LEFT
    //41, // FRONT_RIGHT
    //41, // REAR_LEFT
    2  // REAR_RIGHT
};

/**
 * @brief Direction pin mapping for each motor (1 for forward and -1 for backward).
 * These pins control the motor direction (forward/reverse).
 */
const int MOTOR_DIRECTION_PINS[MOTOR_COUNT] = {
    //40, // FRONT_LEFT
    //41, // FRONT_RIGHT
    //41, // REAR_LEFT
    40  // REAR_RIGHT
};

/**
 * @brief Initializes all motor driver pins and sets motors to a disabled state.
 *
 * Configures motor enable, PWM, direction, and encoder pins.
 * All motors are initialized with their enable pins LOW to keep them off.
 */
void init_motor_drivers() {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        pinMode(MOTOR_ENABLE_PINS[i], OUTPUT);
        pinMode(MOTOR_PWM_PINS[i], OUTPUT);
        pinMode(MOTOR_DIRECTION_PINS[i], OUTPUT);
        
        // Disable motor at startup
        digitalWrite(MOTOR_ENABLE_PINS[i], LOW);
        digitalWrite(MOTOR_PWM_PINS[i], MIN_PWM);
        digitalWrite(MOTOR_DIRECTION_PINS[i], LOW);
    }
}

/**
 * @brief Sets the state of a given motor (enabled/disabled, direction, and speed).
 *
 * @param motor The motor position to control.
 * @param enabled If 1, motor is enabled; if 0, motor is disabled.
 * @param direction If 1, sets direction HIGH->LOW; if 0, LOW->HIGH.
 * @param pwm PWM value to control speed (clamped between MIN_PWM and MAX_PWM).
 */
void set_motor_state(motor_position motor, int enabled, int direction, int pwm){
    // Clamp PWM value to allowed range
    if(pwm < MIN_PWM)
        pwm = MIN_PWM;
    if(pwm > MAX_PWM)
        pwm = MAX_PWM;

    analogWrite(MOTOR_PWM_PINS[motor], pwm);

    // Set motor direction
    if (direction == 1) {
        digitalWrite(MOTOR_DIRECTION_PINS[motor], HIGH);
        digitalWrite(MOTOR_DIRECTION_PINS[motor], LOW);
    } else {
        digitalWrite(MOTOR_DIRECTION_PINS[motor], LOW);
        digitalWrite(MOTOR_DIRECTION_PINS[motor], HIGH);
    }

    // Enable or disable motor
    if(enabled == 0){
        digitalWrite(MOTOR_ENABLE_PINS[motor], LOW);
        digitalWrite(MOTOR_ENABLE_PINS[motor], LOW);
    } else {
        digitalWrite(MOTOR_ENABLE_PINS[motor], HIGH);
        digitalWrite(MOTOR_ENABLE_PINS[motor], HIGH);
    }
}
