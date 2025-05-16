#include "routines.h"
#include "dri_driver.h"
#include "maxon_driver.h"
#include "servo_driver.h"

void handle_routines(){
  handle_unload_routine();
}

UnloadState unload_state = UNLOAD_IDLE;
unsigned long unload_timer = 0;

void handle_unload_routine() {
  switch (unload_state) {      
    case UNLOAD_IDLE:
      break;

    case UNLOAD_OPEN_LATCH:
      set_servo_motor_angle(UNLOAD_LATCH_SERVO_NAME, UNLOAD_LATCH_SERVO_OPEN_POS);
      unload_timer = millis();
      unload_state = UNLOAD_OPEN_DOOR;
      break;

    case UNLOAD_OPEN_DOOR:
      if (millis() - unload_timer > UNLOAD_TIME_TO_OPEN_LATCH) {
        set_servo_motor_angle(UNLOAD_DOOR_SERVO_NAME, UNLOAD_DOOR_SERVO_OPEN_POS);
        unload_timer = millis();
        unload_state = UNLOAD_CONVOYER;
      }
      break;

    case UNLOAD_CONVOYER:
      if (millis() - unload_timer > UNLOAD_TIME_TO_OPEN_DOOR) {
        set_dri_motor_state(UNLOAD_CONVOYER_NAME, UNLOAD_CONVOYER_SPEED);
        unload_timer = millis();
        unload_state = UNLOAD_CLOSE_DOOR;
      }
      break;

    case UNLOAD_CLOSE_DOOR:
      if (millis() - unload_timer > UNLOAD_TIME_TO_CONVOY) {
        set_dri_motor_state(UNLOAD_CONVOYER_NAME, DRI_MIN_PWM);
        set_servo_motor_angle(UNLOAD_DOOR_SERVO_NAME, UNLOAD_DOOR_SERVO_CLOSED_POS);
        unload_timer = millis();
        unload_state = UNLOAD_CLOSE_LATCH;
      }
      break;

    case UNLOAD_CLOSE_LATCH:
      if (millis() - unload_timer > UNLOAD_TIME_TO_OPEN_DOOR) {
        set_servo_motor_angle(UNLOAD_LATCH_SERVO_NAME, UNLOAD_LATCH_SERVO_CLOSED_POS);
        unload_state = UNLOAD_IDLE;
      }
      break;
  }
}
