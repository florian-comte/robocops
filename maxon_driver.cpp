#include "maxon_driver.h"

/**
 * @brief Enable pin mapping for each motor.
 * These pins enable or disable motor output.
 */
const int MAXON_MOTOR_ENABLE_PINS[MAXON_MOTOR_COUNT] = {
    36,  // REAR_RIGHT
    37 // REAR_LEFT
    //40, // FRONT_LEFT
    //41, // FRONT_RIGHT
};

/**
 * @brief Tell if the motor is inversed
 * True if the motor is inversed, false if not.
 */
const int IS_INVERSED_MAXON_MOTOR[MAXON_MOTOR_COUNT] = {
  0, // REAR_RIGHT
  1 // REAR_LEFT
};

/**
 * @brief PWM pin mapping for each motor.
 * These pins control motor speed using pulse-width modulation.
 */
const int MAXON_MOTOR_PWM_PINS[MAXON_MOTOR_COUNT] = {
    6,  // REAR_RIGHT
    7 // REAR_LEFT
    //40, // FRONT_LEFT
    //41, // FRONT_RIGHT
};

double maxon_target_speeds[MAXON_MOTOR_COUNT];

/**
 * @brief Direction pin mapping for each motor (1 for forward and -1 for backward).
 * These pins control the motor direction (forward/reverse).
 */
const int MAXON_MOTOR_DIRECTION_PINS[MAXON_MOTOR_COUNT] = {
    34,  // REAR_RIGHT
    35 // REAR_LEFT

    //40, // FRONT_LEFT
    //41, // FRONT_RIGHT
};

/**
 * @brief Initializes all motor driver pins and sets motors to a disabled state.
 *
 * Configures motor enable, PWM, direction, and encoder pins.
 * All motors are initialized with their enable pins LOW to keep them off.
 */
void init_maxon_motor_drivers() {
    for (int i = 0; i < MAXON_MOTOR_COUNT; i++) {
        pinMode(MAXON_MOTOR_ENABLE_PINS[i], OUTPUT);
        pinMode(MAXON_MOTOR_PWM_PINS[i], OUTPUT);
        pinMode(MAXON_MOTOR_DIRECTION_PINS[i], OUTPUT);
        
        // Disable motor at startup
        digitalWrite(MAXON_MOTOR_ENABLE_PINS[i], LOW);
        digitalWrite(MAXON_MOTOR_PWM_PINS[i], MAXON_MIN_PWM);
        digitalWrite(MAXON_MOTOR_DIRECTION_PINS[i], LOW);

        maxon_target_speeds[i] = MAXON_MIN_MOTOR_SPEED;
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
void set_maxon_motor_state(maxon_motor_position motor, int enabled, int direction, int pwm){
    // Clamp PWM value to allowed range
    if(pwm < MAXON_MIN_PWM)
        pwm = MAXON_MIN_PWM;
    if(pwm > MAXON_MAX_PWM)
        pwm = MAXON_MAX_PWM;

    analogWrite(MAXON_MOTOR_PWM_PINS[motor], pwm);

    // Set motor direction
    if (direction == 1) {
        digitalWrite(MAXON_MOTOR_DIRECTION_PINS[motor], !IS_INVERSED_MAXON_MOTOR[motor] ? LOW : HIGH);
    } else {
        digitalWrite(MAXON_MOTOR_DIRECTION_PINS[motor], !IS_INVERSED_MAXON_MOTOR[motor] ? HIGH : LOW);
    }

    // Enable or disable motor
    if(enabled == 0){
        digitalWrite(MAXON_MOTOR_ENABLE_PINS[motor], LOW);
    } else {
        digitalWrite(MAXON_MOTOR_ENABLE_PINS[motor], HIGH);
    }
}
