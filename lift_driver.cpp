#include "lift_driver.h"

// create the two stepper motors class instances
AccelStepper right_stepper(LIFT_MOTOR_INTERFACE_TYPE, LIFT_RIGHT_STEP_PIN, LIFT_RIGHT_DIR_PIN);
AccelStepper left_stepper(LIFT_MOTOR_INTERFACE_TYPE, LIFT_LEFT_STEP_PIN, LIFT_LEFT_DIR_PIN);

double stepper_target_position;

void init_lift(){
  // setup pins
  pinMode(LIFT_RIGHT_DIR_PIN, OUTPUT);
  pinMode(LIFT_RIGHT_STEP_PIN, OUTPUT);
  pinMode(LIFT_LEFT_DIR_PIN, OUTPUT);
  pinMode(LIFT_LEFT_STEP_PIN, OUTPUT);

  digitalWrite(LIFT_RIGHT_DIR_PIN, HIGH);  // initialise at CW for safety
  digitalWrite(LIFT_LEFT_DIR_PIN, HIGH); // initialise at CW for safety

  // setting the required configuration for the stepper motors
  right_stepper.setMaxSpeed(LIFT_MOTOR_MAX_SPEED);
  right_stepper.setCurrentPosition(0);
  //right_stepper.setAcceleration(MOTOR_ACCELERATION);

  left_stepper.setMaxSpeed(LIFT_MOTOR_MAX_SPEED);
  left_stepper.setCurrentPosition(0);
  //left_stepper.setAcceleration(MOTOR_ACCELERATION);
}

void update_lift(){
  if(right_stepper.targetPosition() != stepper_target_position){
    right_stepper.moveTo(stepper_target_position);
    right_stepper.setSpeed(LIFT_MOTOR_SPEED);
  }

  if(left_stepper.targetPosition() != stepper_target_position){
    left_stepper.moveTo(stepper_target_position);
    left_stepper.setSpeed(LIFT_MOTOR_SPEED);
  }

  right_stepper.runSpeedToPosition();
  left_stepper.runSpeedToPosition();
}

bool is_lift_done() {
  return (right_stepper.distanceToGo() == 0) && (left_stepper.distanceToGo() == 0);
}
