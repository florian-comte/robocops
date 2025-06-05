#include "ir_sensor.h"

const int IR_SENSORS_PINS[IR_SENSOR_COUNT] = {
  41, // BOTTOM
  47, // TOP
  9 /// FRONT_IR
};

// Activated if > 0.5
double ir_activated_averaged[IR_SENSOR_COUNT];
float next_front_ir_capture;
float next_back_ir_capture;

void init_ir_sensors(){
  for(int i = 0; i < IR_SENSOR_COUNT; i++){
    pinMode(i, INPUT);
  }

  next_front_ir_capture = millis();
  next_back_ir_capture = millis();
}

void update_ir_sensors(){
  if(millis() >= next_back_ir_capture){
    ir_activated_averaged[TOP_IR] = (digitalRead(IR_SENSORS_PINS[TOP_IR]) + (IR_SENSOR_BACK_WINDOW - 1) * IR_SENSORS_PINS[TOP_IR]) / IR_SENSOR_BACK_WINDOW;
    ir_activated_averaged[BOTTOM_IR] = (digitalRead(IR_SENSORS_PINS[BOTTOM_IR]) + (IR_SENSOR_BACK_WINDOW - 1) * IR_SENSORS_PINS[BOTTOM_IR]) / IR_SENSOR_BACK_WINDOW;

    next_back_ir_capture += IR_SENSOR_BACK_INTERVAL;
  }

  if(millis() >= next_front_ir_capture){
    ir_activated_averaged[FRONT_IR] = (digitalRead(IR_SENSORS_PINS[FRONT_IR]) + (IR_SENSOR_FRONT_WINDOW - 1) * IR_SENSORS_PINS[FRONT_IR]) / IR_SENSOR_FRONT_WINDOW;
    next_front_ir_capture += IR_SENSOR_FRONT_INTERVAL;
  }
}

bool duplo_detected_in_back_lift(){
  return ir_activated_averaged[BOTTOM_IR] > 0.5 || ir_activated_averaged[TOP_IR] > 0.5;
}

bool duplo_detected_in_front_lift(){
  return ir_activated_averaged[FRONT_IR] > 0.5;
}
