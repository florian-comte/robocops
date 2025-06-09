#include "back_ultrasound_sensor.h"

#define BACK_ULTRASOUND_SENSOR_TRIGGER_PIN 47
#define BACK_ULTRASOUND_SENSOR_ECHO_PIN 45

#define BACK_ULTRASOUND_TIMEOUT 5000
#define BACK_ULTRASOUND_PING_INTERVAL 100 //ms

unsigned long next_ping_time;

NewPing back_ultrasound_sensor(BACK_ULTRASOUND_SENSOR_TRIGGER_PIN, BACK_ULTRASOUND_SENSOR_ECHO_PIN, BACK_ULTRASOUND_MAX_DISTANCE);

double back_ultrasound_averaged_distance = BACK_ULTRASOUND_MAX_DISTANCE;

bool available_to_ping = true;

void ultrasound_back_echo_callback(){
  if(back_ultrasound_sensor.check_timer()){
      back_ultrasound_averaged_distance = (back_ultrasound_sensor.ping_result / US_ROUNDTRIP_CM + ((BACK_ULTRASOUND_WINDOW - 1.0) * back_ultrasound_averaged_distance)) / BACK_ULTRASOUND_WINDOW;
  }
  
  available_to_ping = true;
}

void init_back_ultrasound_sensor(){
  next_ping_time = millis();
}

void update_back_ultrasound_sensorr() {
  if(available_to_ping){
    available_to_ping = false;
    next_ping_time = millis() + BACK_ULTRASOUND_PING_INTERVAL;
  }
  
  if(millis() > next_ping_time){
    back_ultrasound_sensor.ping_timer(ultrasound_back_echo_callback); 
  }
}

void update_back_ultrasound_sensor(){
  if(millis() > next_ping_time){
    digitalWrite(BACK_ULTRASOUND_SENSOR_TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(BACK_ULTRASOUND_SENSOR_TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(BACK_ULTRASOUND_SENSOR_TRIGGER_PIN, LOW);
  
    double duration = pulseIn(BACK_ULTRASOUND_SENSOR_ECHO_PIN, HIGH);
    back_ultrasound_averaged_distance = (duration*.0343)/2;

    next_ping_time = millis() + BACK_ULTRASOUND_PING_INTERVAL;
  }
}
