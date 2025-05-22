#include "lift_ultrasound_sensor.h"

#define LIFT_ULTRASOUND_SENSOR_TRIGGER_PIN 12
#define LIFT_ULTRASOUND_SENSOR_ECHO_PIN 11

#define LIFT_ULTRASOUND_TIMEOUT 5000
#define LIFT_ULTRASOUND_PING_INTERVAL 50 //ms

unsigned long next_ping_time;

NewPing lift_ultrasound_sensor(LIFT_ULTRASOUND_SENSOR_TRIGGER_PIN, LIFT_ULTRASOUND_SENSOR_ECHO_PIN, LIFT_ULTRASOUND_MAX_DISTANCE);

int lift_ultrasound_averaged_distance = LIFT_ULTRASOUND_MAX_DISTANCE;

void ultrasound_lift_echo_callback(){
  if (lift_ultrasound_sensor.check_timer()){
    lift_ultrasound_averaged_distance = ((lift_ultrasound_sensor.ping_result / US_ROUNDTRIP_CM) + ((LIFT_ULTRASOUND_WINDOW - 1) * lift_ultrasound_averaged_distance)) / LIFT_ULTRASOUND_WINDOW;
  }
}

void init_lift_ultrasound_sensor(){
  next_ping_time = millis();
}

void update_lift_ultrasound_sensor() {
  if(millis() > next_ping_time){
    next_ping_time += LIFT_ULTRASOUND_PING_INTERVAL;
    lift_ultrasound_sensor.ping_timer(ultrasound_lift_echo_callback); 
  }
}
