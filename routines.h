#ifndef ROUTINES_H
#define ROUTINES_H

#include <Arduino.h>

// --- UNLOAD ROUTINE ---

#define UNLOAD_LATCH_SERVO_CLOSED_POS 45
#define UNLOAD_DOOR_SERVO_CLOSED_POS 90
#define UNLOAD_LATCH_SERVO_OPEN_POS 45
#define UNLOAD_DOOR_SERVO_OPEN_POS 90

#define UNLOAD_DOOR_SERVO_NAME UNLOAD_DOOR_MOTOR
#define UNLOAD_LATCH_SERVO_NAME UNLOAD_LATCH_MOTOR
#define UNLOAD_CONVOYER_NAME CONVOYER_BACK

#define UNLOAD_CONVOYER_SPEED 200

#define UNLOAD_TIME_TO_OPEN_LATCH 500
#define UNLOAD_TIME_TO_OPEN_DOOR 500
#define UNLOAD_TIME_TO_CONVOY 2000

enum UnloadState {
  UNLOAD_IDLE,
  UNLOAD_OPEN_LATCH,
  UNLOAD_OPEN_DOOR,
  UNLOAD_CONVOYER,
  UNLOAD_CLOSE_DOOR,
  UNLOAD_CLOSE_LATCH
};

extern UnloadState unload_state;
extern unsigned long unload_timer;


extern void handle_routines();
void handle_unload_routine();

#endif // ROUTINES_H
