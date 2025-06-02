#ifndef ROUTINES_H
#define ROUTINES_H

#include <Arduino.h>

// --- UNLOAD ROUTINE ---

//#define UNLOAD_LATCH_SERVO_CLOSED_POS 0
#define UNLOAD_DOOR_SERVO_CLOSED_POS 135
//#define UNLOAD_LATCH_SERVO_OPEN_POS 90
#define UNLOAD_DOOR_SERVO_OPEN_POS 20

#define UNLOAD_DOOR_SERVO_NAME UNLOAD_DOOR_MOTOR
//#define UNLOAD_LATCH_SERVO_NAME UNLOAD_LATCH_MOTOR
#define UNLOAD_CONVOYER_NAME CONVOYER_BACK

#define UNLOAD_CONVOYER_SPEED 200

//#define UNLOAD_TIME_TO_OPEN_LATCH 1000
#define UNLOAD_TIME_TO_OPEN_DOOR 1000
#define UNLOAD_TIME_TO_CONVOY 1000

enum UnloadState {
  UNLOAD_IDLE,
  //UNLOAD_OPEN_LATCH,
  UNLOAD_OPEN_DOOR,
  UNLOAD_CONVOYER,
  UNLOAD_REVERSE_CONVOYER,
  UNLOAD_CLOSE_DOOR,
  UNLOAD_CLOSING
  //UNLOAD_CLOSE_LATCH
};

extern UnloadState unload_state;
extern unsigned long unload_timer;

// --- LIFT ROUTINE ---

#define LIFT_TIME_TO_UP 5000
#define LIFT_TIME_TO_CONVOY 1000
#define LIFT_CONVOYER_NAME CONVOYER_LIFT
#define LIFT_CONVOYER_SPEED 200
#define LIFT_UP_POSITION 1270
#define LIFT_DOWN_POSITION 0

enum LiftState {
  LIFT_IDLE,
  LIFT_UP,
  LIFT_CONVOYER,
  LIFT_REVERSE_CONVOYER,
  LIFT_DOWN,
  LIFT_DOWNING
};

extern LiftState lift_state;
extern unsigned long lift_timer;

extern void handle_routines();
void handle_unload_routine();
void handle_lift_routine();

#endif // ROUTINES_H
