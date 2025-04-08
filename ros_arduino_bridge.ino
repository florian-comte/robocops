#include "maxon_driver.h"
#include "maxon_encoder.h"
#include "pid.h"
#include "commands.h"

#define BAUDRATE 57600

unsigned long last_loop_time = 0;
const unsigned long loop_interval = 50; // ms 

void setup(){
  Serial.begin(BAUDRATE);
  
  init_motor_drivers();
  init_motor_encoders();

  for (int i = 0; i < MOTOR_COUNT; i++) {
    reset_pid(i);
  }
}

void loop(){
  unsigned long now = millis();

  if (now - last_loop_time >= loop_interval) {
    last_loop_time = now;

    // For each motor, update PID
    for (int i = 0; i < MOTOR_COUNT; i++) {
      // todo: update pid
      
    }
  }

  //todo: handling serial commands
}
