#include "lift_ultrasound_sensor.h"

#define LIFT_ULTRASOUND_SENSOR_TRIGGER_PIN 47
#define LIFT_ULTRASOUND_SENSOR_ECHO_PIN 45

#define LIFT_ULTRASOUND_TIMEOUT 5000
#define LIFT_ULTRASOUND_PING_INTERVAL 100 //ms

unsigned long next_ping_time;

NewPing lift_ultrasound_sensor(LIFT_ULTRASOUND_SENSOR_TRIGGER_PIN, LIFT_ULTRASOUND_SENSOR_ECHO_PIN, LIFT_ULTRASOUND_MAX_DISTANCE);

double lift_ultrasound_averaged_distance = LIFT_ULTRASOUND_MAX_DISTANCE;

bool available_to_ping = true;

void ultrasound_lift_echo_callback(){
  if (lift_ultrasound_sensor.check_timer()){
    lift_ultrasound_averaged_distance = ((lift_ultrasound_sensor.ping_result / US_ROUNDTRIP_CM) + ((LIFT_ULTRASOUND_WINDOW - 1.0) * lift_ultrasound_averaged_distance)) / LIFT_ULTRASOUND_WINDOW;

    available_to_ping = true;
  }
}

void init_lift_ultrasound_sensor(){
  next_ping_time = millis();
}

void update_lift_ultrasound_sensor() {
  if(available_to_ping){
    available_to_ping = false;
    next_ping_time = millis() + LIFT_ULTRASOUND_PING_INTERVAL;
  }
  
  if(millis() > next_ping_time){
    lift_ultrasound_sensor.ping_timer(ultrasound_lift_echo_callback); 
  }
}
