#ifndef LIFT_H
#define LIFT_H

#include <Arduino.h>
#include <AccelStepper.h>

// Stepper motors configuration
#define LIFT_STEPS_PER_REVOLUTION 200  // 1.8 degrees per step (360deg / 1.8 = 200)
#define LIFT_MOTOR_INTERFACE_TYPE 1  // to get the proper interface in the AccelStepper class
#define LIFT_MOTOR_MAX_SPEED 1000  // stepper motor's max speed datasheet (in steps per second)
#define LIFT_MOTOR_SPEED 800 // stepper motor's desired speed (in steps per second)
//#define LIFT_MOTOR_ACCELERATION 1  // stepper motor's desired acceleration CAN'T USE BECAUSE OF BLOCKING FUNCTION...

// Right stepper motor
#define LIFT_RIGHT_DIR_PIN 48
#define LIFT_RIGHT_STEP_PIN 46

// Left stepper motor
#define LIFT_LEFT_DIR_PIN 42
#define LIFT_LEFT_STEP_PIN 44

extern double stepper_target_position;

void init_lift();
void update_lift();
bool is_lift_done();

#endif // LIFT_H
