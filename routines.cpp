#include "routines.h"
#include "dri_driver.h"
#include "maxon_driver.h"
#include "servo_driver.h"
#include "lift_driver.h"

void handle_routines(){
  handle_unload_routine();
  handle_lift_routine();
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
      }
      break;

    case LIFT_REVERSE_CONVOYER:
      if (millis() - lift_timer > LIFT_TIME_TO_CONVOY) {
        dri_target_speeds[LIFT_CONVOYER_NAME] = -LIFT_CONVOYER_SPEED;
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
