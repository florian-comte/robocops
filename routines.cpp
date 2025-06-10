#include "routines.h"
#include "dri_driver.h"
#include "maxon_driver.h"
#include "servo_driver.h"
#include "lift_driver.h"
#include "l298n_driver.h"
#include "ir_sensor.h"

void init_routines(){
  unload_state = UNLOAD_IDLE; 
  lift_state = LIFT_IDLE;
  button_state = BUTTON_IDLE;
  slope_up_state = SLOPE_UP_IDLE;
  slope_down_state = SLOPE_DOWN_IDLE;
  capture_state = CAPTURE_IDLE;
}

void handle_routines(){
  // If ready to lift
  if(lift_state == LIFT_IDLE && capture_state == CAPTURE_CAPTURED){
    capture_state = CAPTURE_IDLE;
    lift_state = LIFT_UP;
  }

  // For jerk (prioritized compare to unload)
  if(lift_state != LIFT_REVERSE_CONVOYER && lift_state != LIFT_DOWN){
    handle_unload_routine();
  }
  
  handle_lift_routine();
  handle_button_routine();
  handle_slope_down_routine();
  handle_slope_up_routine();
  handle_capture_routine();
}

// --- Unload routine ---

UnloadState unload_state = UNLOAD_IDLE;
unsigned long unload_timer = 0;

void handle_unload_routine() {
  switch (unload_state) {      
    case UNLOAD_IDLE:
      break;

    /*case UNLOAD_OPEN_LATCH:
      servo_target_angles[UNLOAD_LATCH_SERVO_NAME] = UNLOAD_LATCH_SERVO_OPEN_POS;
      unload_timer = millis();
      unload_state = UNLOAD_OPEN_DOOR;
      break;*/

    case UNLOAD_OPEN_DOOR:
      //if (millis() - unload_timer > UNLOAD_TIME_TO_OPEN_LATCH) {
      servo_target_angles[UNLOAD_DOOR_SERVO_NAME] = UNLOAD_DOOR_SERVO_OPEN_POS;
      unload_timer = millis();
      unload_state = UNLOAD_CONVOYER;
      //}
      break;

    case UNLOAD_CONVOYER:
      if (millis() - unload_timer > UNLOAD_TIME_TO_OPEN_DOOR) {
        dri_target_speeds[UNLOAD_CONVOYER_NAME] = UNLOAD_CONVOYER_SPEED;
        unload_timer = millis();
        unload_state = UNLOAD_REVERSE_CONVOYER;
      }
      break;

    case UNLOAD_REVERSE_CONVOYER:
      if (millis() - unload_timer > UNLOAD_TIME_TO_CONVOY) {
        dri_target_speeds[UNLOAD_CONVOYER_NAME] = -UNLOAD_CONVOYER_SPEED;
        unload_timer = millis();
        unload_state = UNLOAD_CLOSE_DOOR;
      }
      break;

    case UNLOAD_CLOSE_DOOR:
      if (millis() - unload_timer > UNLOAD_TIME_TO_CONVOY) {
        dri_target_speeds[UNLOAD_CONVOYER_NAME] = DRI_MIN_PWM;
        servo_target_angles[UNLOAD_DOOR_SERVO_NAME] = UNLOAD_DOOR_SERVO_CLOSED_POS;
        unload_timer = millis();
        unload_state = UNLOAD_CLOSING;
      }
      break;

    case UNLOAD_CLOSING:
      if (millis() - unload_timer > UNLOAD_TIME_TO_OPEN_DOOR) {
        unload_state = UNLOAD_IDLE;
      }
      break;

    /*case UNLOAD_CLOSE_LATCH:
      if (millis() - unload_timer > UNLOAD_TIME_TO_OPEN_DOOR) {
        servo_target_angles[UNLOAD_LATCH_SERVO_NAME] = UNLOAD_LATCH_SERVO_CLOSED_POS;
        unload_state = UNLOAD_IDLE;
      }
      break;*/
  }
}

// --- Lift routine

LiftState lift_state = LIFT_IDLE;
unsigned long lift_timer = 0;
bool lift_jerk_reversed_direction = 0;
unsigned long lift_jerk_timer = 0;

void handle_lift_routine() {
  switch (lift_state) {      
    case LIFT_IDLE:
      break;
      
    case LIFT_UP:
      stepper_target_position = LIFT_UP_POSITION;
      lift_timer = millis();
      lift_state = LIFT_CONVOYER;
      break;
      
    case LIFT_CONVOYER:
      if ((millis() - lift_timer > LIFT_TIME_TO_UP) || is_lift_done()) {
        dri_target_speeds[LIFT_CONVOYER_NAME] = LIFT_CONVOYER_SPEED;
        lift_timer = millis();
        lift_state = LIFT_REVERSE_CONVOYER;
        lift_jerk_timer = millis();
      }
      break;

    case LIFT_REVERSE_CONVOYER:
      // jerk in parrallel
      if(millis() - lift_jerk_timer > LIFT_CONVOYER_JERK_DURATION){
        if(lift_jerk_reversed_direction){
          dri_target_speeds[UNLOAD_CONVOYER_NAME] = UNLOAD_CONVOYER_SPEED;
          lift_jerk_reversed_direction = false;
          lift_jerk_timer = millis();
        } else {
          dri_target_speeds[UNLOAD_CONVOYER_NAME] = -UNLOAD_CONVOYER_SPEED;
          lift_jerk_reversed_direction = true;
          lift_jerk_timer = millis();
        }
      }
      
      if (millis() - lift_timer > LIFT_TIME_TO_CONVOY) {
        dri_target_speeds[LIFT_CONVOYER_NAME] = -LIFT_CONVOYER_SPEED;
        dri_target_speeds[UNLOAD_CONVOYER_NAME] = 0;
        lift_timer = millis();
        lift_state = LIFT_DOWN;
      }
      break;

    case LIFT_DOWN:
      if (millis() - lift_timer > LIFT_TIME_TO_CONVOY) {
        dri_target_speeds[LIFT_CONVOYER_NAME] = DRI_MIN_PWM;
        stepper_target_position = LIFT_DOWN_POSITION;
        lift_timer = millis();
        lift_state = LIFT_DOWNING;
      }
      break;

    case LIFT_DOWNING:
      if (millis() - lift_timer > LIFT_TIME_TO_UP || is_lift_done()) {
        lift_state = LIFT_IDLE;
      }
      break;
  }
}

// --- Button routine

ButtonState button_state = BUTTON_IDLE;
unsigned long button_timer = 0;

void handle_button_routine() {
  switch (button_state) {      
    case BUTTON_IDLE:
      break;
      
    case BUTTON_BACKWARD:
      button_timer = millis();
      button_state = BUTTON_FORWARD;
      maxon_target_speeds[MAXON_REAR_LEFT] = -3000;
      maxon_target_speeds[MAXON_REAR_RIGHT] = -3000;
      break;
      
    case BUTTON_FORWARD:
      if (millis() - button_timer > BUTTON_TIME_TO_DRIVE) {
        button_timer = millis();
        button_state = BUTTON_GOING_FORWARD;
        maxon_target_speeds[MAXON_REAR_LEFT] = 3000;
        maxon_target_speeds[MAXON_REAR_RIGHT] = 3000;
      }
      break;

    case BUTTON_GOING_FORWARD:
      if (millis() - button_timer > BUTTON_TIME_TO_DRIVE) {
        maxon_target_speeds[MAXON_REAR_LEFT] = 0;
        maxon_target_speeds[MAXON_REAR_RIGHT] = 0;
        button_state = BUTTON_IDLE; 
      }
      break;
  }
}

// --- Slope up routine

SlopeUpState slope_up_state = SLOPE_UP_IDLE;
unsigned long slope_up_timer = 0;

void handle_slope_up_routine() {
  switch (slope_up_state) { 
    case SLOPE_UP_IDLE:
      break;

    case SLOPE_UP_ENGAGE:
      slope_up_timer = millis();
      slope_up_state = SLOPE_UP_REACHED;
      maxon_target_speeds[MAXON_REAR_LEFT] = 5000;
      maxon_target_speeds[MAXON_REAR_RIGHT] = 5000;
      break;

    case SLOPE_UP_REACHED:
      if (millis() - slope_up_timer > SLOPE_TIME_UP) {
        maxon_target_speeds[MAXON_REAR_LEFT] = 0;
        maxon_target_speeds[MAXON_REAR_RIGHT] = 0;
        slope_up_state = SLOPE_UP_IDLE;
      }
      break;
  }
}


// --- Slope up routine

SlopeDownState slope_down_state = SLOPE_DOWN_IDLE;
unsigned long slope_down_timer = 0;

void handle_slope_down_routine() {
  switch (slope_down_state) {      
    case SLOPE_DOWN_IDLE:
      break;

    case SLOPE_DOWN_ENGAGE:
      slope_down_timer = millis();
      slope_down_state = SLOPE_DOWN_REACHED;
      maxon_target_speeds[MAXON_REAR_LEFT] = 5000;
      maxon_target_speeds[MAXON_REAR_RIGHT] = 5000;
      break;

    case SLOPE_DOWN_REACHED:
      if (millis() - slope_down_timer > SLOPE_TIME_DOWN) {
        maxon_target_speeds[MAXON_REAR_LEFT] = 0;
        maxon_target_speeds[MAXON_REAR_RIGHT] = 0;
        slope_down_state = SLOPE_DOWN_IDLE;
      }
      break;
  }
}

// --- Capture routine

CaptureState capture_state = CAPTURE_IDLE;
unsigned long capture_timer = 0;
unsigned long next_small_convoyer = -1;
unsigned long current_small_convoyer = -1;
bool front_detected_something = false;

void handle_capture_routine() {
  switch (capture_state) {      
    case CAPTURE_IDLE:
      // By security reset the detection
      front_detected_something = false;
      l298n_target_speeds[BRUSH_LEFT] = 0;
      l298n_target_speeds[BRUSH_RIGHT] = 0;
      break;
      
    case CAPTURE_BRUSHING:
      l298n_target_speeds[BRUSH_LEFT] = CAPTURE_BRUSH_SPEED;
      l298n_target_speeds[BRUSH_RIGHT] = CAPTURE_BRUSH_SPEED;

      if(front_detected_something){
        if(millis() - capture_timer > CAPTURE_BRUSHING_STOP_AFTER){
          l298n_target_speeds[BRUSH_LEFT] = 0;
          l298n_target_speeds[BRUSH_RIGHT] = 0;
     
          capture_state = CAPTURE_SMALL_BACKWARD;
          capture_timer = millis();
          front_detected_something = false;
        }
      } else {
        if(duplo_detected_in_front_lift()){
          capture_timer = millis();
          front_detected_something = true;
        }
      }

      if(duplo_detected_in_back_lift()){
        l298n_target_speeds[BRUSH_LEFT] = 0;
        l298n_target_speeds[BRUSH_RIGHT] = 0;
   
        capture_state = CAPTURE_SMALL_BACKWARD;
        front_detected_something = false;
      }
      
      break;
      
    case CAPTURE_SMALL_BACKWARD:
      // Small backward logic
      if(current_small_convoyer == -1){
        if(millis() > next_small_convoyer || next_small_convoyer == -1){
          current_small_convoyer = millis();
          dri_target_speeds[LIFT_CONVOYER_NAME] = CAPTURE_SMALL_CONVOYER_SPEED;
        }
      } else {
        if(millis() - current_small_convoyer > CAPTURE_SMALL_CONVOYER_DURATION){
          dri_target_speeds[LIFT_CONVOYER_NAME] = 0;
          next_small_convoyer = millis() + CAPTURE_SMALL_CONVOYER_INTERVAL;
          current_small_convoyer = -1;
        }
      }
     
      if(millis() - capture_timer > CAPTURE_SMALL_BACKWARD_TIMEOUT){
        if(!duplo_detected_in_front_lift()){
          current_small_convoyer = -1;
          next_small_convoyer = -1;
          dri_target_speeds[LIFT_CONVOYER_NAME] = 0;
          capture_state = CAPTURE_CAPTURED; 
        } else {
          // todo: here maybe eject duplos ? 
        }
      }

      // If duplo detected
      if(duplo_detected_in_back_lift() && !duplo_detected_in_front_lift()){
        current_small_convoyer = -1;
        next_small_convoyer = -1;
        dri_target_speeds[LIFT_CONVOYER_NAME] = 0;
        capture_state = CAPTURE_CAPTURED;
      }
      
      break;
      
    case CAPTURE_CAPTURED:
      break;
  }
}
