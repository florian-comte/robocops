#include "pid.h"


/// Proportional gain constant for the PID controller.
const int k_p = 20;

/// Derivative gain constant for the PID controller.
const int k_d = 12;

/// Integral gain constant for the PID controller.
const int k_i = 0;

/// Output scaling factor to map PID output to PWM range.
const int k_o = 50;

/// Target speeds for each motor in RPM.
float target_speeds[MOTOR_COUNT] = {0};

/// Current estimated speeds for each motor in RPM.
float speeds[MOTOR_COUNT] = {0};

/// Previous measured speeds, used in derivative calculation.
float previous_speeds[MOTOR_COUNT] = {0};

/// Next computed speed command based on PID output.
float next_speeds[MOTOR_COUNT] = {0};

/// Direction for next speed update (1 for forward, -1 for reverse).
float next_directions[MOTOR_COUNT] = {1};

/**
 * @brief Previous input speed used in derivative term to avoid derivative kick.
 *
 * See: http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
 */
int previous_inputs[MOTOR_COUNT] = {0};

/**
 * @brief Integrated term accumulator for the I component of PID.
 *
 * ITerm is used instead of a raw error sum to allow better tuning adjustment.
 * 
 * See: http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
 */
int integrated_terms[MOTOR_COUNT] = {0};

/// Binary flag for whether a motor is actively commanded to move.
int is_moving[MOTOR_COUNT] = {0};

/**
 * @brief Updates the PID loop for the specified motor.
 *
 * If the motor is not currently moving, the PID controller is reset to avoid spikes.
 * Otherwise, PID values are updated and motor speeds are adjusted accordingly.
 *
 * @param motor The motor position enum value to update.
 */
void update_pid(motor_position motor) {
    speeds[motor] = read_encoder(motor);

    if (!is_moving[motor]) {
        /*
         * Reset PID to prevent startup spikes.
         * See: http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
         */
        reset_pid(motor);

        set_motor_state(
            motor,
            false,                              // enabled
            next_directions[motor],            // direction
            (int)next_speeds[motor]            // PWM
        );
        return;
    }

    // Run PID update for this motor
    do_pid(motor, target_speeds[motor]);

    // Apply motor state based on computed output
    set_motor_state(
        motor,
        true,                              // enabled
        next_directions[motor],            // direction
        (int)next_speeds[motor]            // PWM
    );
}

/**
 * @brief Performs a single PID computation for the specified motor.
 *
 * Uses error between target speed and current estimated speed.
 * Incorporates derivative and integral terms, and scales output.
 *
 * @param motor The motor position.
 * @param speed_target The desired RPM speed.
 */
void do_pid(motor_position motor, float speed_target) {
    int input = speeds[motor] - previous_speeds[motor];    
    long p_error = speed_target - input;                    

    // PID formula with output scaling
    float output = (k_p * p_error - k_d * (input - previous_inputs[motor]) + integrated_terms[motor]) / k_o;

    previous_speeds[motor] = speeds[motor];
    previous_inputs[motor] = input;

    // Add PID output to previous command
    next_speeds[motor] += output;

    // Determine direction
    next_directions[motor] = (next_speeds[motor] >= 0) ? 1 : -1;

    // Normalize to positive range for PWM control
    next_speeds[motor] = abs(next_speeds[motor]);

    // Map output to valid PWM range
    next_speeds[motor] = map_float(next_speeds[motor], 0, MAX_MOTOR_SPEED, MIN_PWM, MAX_PWM);

    // Clamp PWM output within range
    if (next_speeds[motor] > MAX_PWM) {
        next_speeds[motor] = MAX_PWM;
    } else if (next_speeds[motor] < MIN_PWM) {
        next_speeds[motor] = MIN_PWM;
    } else {
        // Accumulate integral term (if within bounds)
        integrated_terms[motor] += k_i * p_error;
    }
}

/**
 * @brief Resets PID controller state for a given motor.
 *
 * Clears integral term, previous speed, and sets the direction forward.
 *
 * @param motor The motor to reset.
 */
void reset_pid(motor_position motor) {
    target_speeds[motor] = 0.0;
    speeds[motor] = read_encoder(motor);
    previous_speeds[motor] = speeds[motor];
    next_speeds[motor] = 0;
    previous_inputs[motor] = 0;
    integrated_terms[motor] = 0;
    next_directions[motor] = 1;
}
