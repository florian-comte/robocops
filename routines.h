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
#define LIFT_CONVOYER_JERK_DURATION 75

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


// --- BUTTON ROUTINE ---

#define BUTTON_TIME_TO_DRIVE 1750

enum ButtonState {
  BUTTON_IDLE,
  BUTTON_BACKWARD,
  BUTTON_FORWARD,
  BUTTON_GOING_FORWARD,
  BUTTON_FINISHED
};

extern ButtonState button_state;
extern unsigned long button_timer;

// --- SLOPE UP ROUTINE ---

enum SlopeUpState {
  SLOPE_UP_IDLE,
};

extern SlopeUpState slope_up_state;
extern unsigned long slope_up_timer;

// --- SLOPE DOWN ROUTINE ---

enum SlopeDownState {
  SLOPE_DOWN_IDLE,
};

extern SlopeDownState slope_down_state;
extern unsigned long slope_down_timer;

// --- CAPTURE ROUTINE ---

#define CAPTURE_BRUSH_SPEED 240
#define CAPTURE_BRUSHING_STOP_AFTER 1000
#define CAPTURE_SMALL_BACKWARD_TIMEOUT 5000
#define CAPTURE_SMALL_CONVOYER_SPEED 80
#define CAPTURE_SMALL_CONVOYER_INTERVAL 200 
#define CAPTURE_SMALL_CONVOYER_DURATION 200 

enum CaptureState {
  CAPTURE_IDLE,
  CAPTURE_BRUSHING,
  CAPTURE_SMALL_BACKWARD,
  CAPTURE_CAPTURED
};

extern CaptureState capture_state;
extern unsigned long capture_timer;

extern void init_routines();
extern void handle_routines();
void handle_unload_routine();
void handle_lift_routine();
void handle_button_routine();
void handle_slope_up_routine();
void handle_slope_down_routine();
void handle_capture_routine();

#endif // ROUTINES_H
