#include "lift_ultrasound_sensor.h"

#define LIFT_ULTRASOUND_SENSOR_TRIGGER_PIN 5
#define LIFT_ULTRASOUND_SENSOR_ECHO_PIN 5

#define LIFT_ULTRASOUND_TIMEOUT 30000

NewPing lift_ultrasound_sensor(LIFT_ULTRASOUND_SENSOR_TRIGGER_PIN, LIFT_ULTRASOUND_SENSOR_ECHO_PIN, LIFT_ULTRASOUND_MAX_DISTANCE);
bool ready_to_ping = 1;
unsigned long last_ping = 0;

int lift_ultrasound_averaged_distance = LIFT_ULTRASOUND_MAX_DISTANCE;

void ultrasound_lift_echo_callback(){
  if (lift_ultrasound_sensor.check_timer()){
    lift_ultrasound_averaged_distance = ((lift_ultrasound_sensor.ping_result / US_ROUNDTRIP_CM) + ((LIFT_ULTRASOUND_WINDOW - 1) * lift_ultrasound_averaged_distance)) / LIFT_ULTRASOUND_WINDOW;
    ready_to_ping = 1;
  }
}

void update_lift_ultrasound_sensor() {
  if(ready_to_ping == 1 || (millis() - last_ping >= LIFT_ULTRASOUND_TIMEOUT)){
    lift_ultrasound_sensor.timer_stop(); 

    ready_to_ping = 0;
    last_ping = millis();
    lift_ultrasound_sensor.ping_timer(ultrasound_lift_echo_callback);
  }
}
