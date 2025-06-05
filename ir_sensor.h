#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include <Arduino.h>

#define IR_SENSOR_COUNT 3
#define IR_SENSOR_BACK_WINDOW 10
#define IR_SENSOR_BACK_INTERVAL 50
#define IR_SENSOR_FRONT_WINDOW 10
#define IR_SENSOR_FRONT_INTERVAL 10

enum ir_sensor_position {
    BOTTOM_IR = 0,
    TOP_IR = 1,
    FRONT_IR = 2
};

extern double ir_activated_averaged[IR_SENSOR_COUNT];

void init_ir_sensors();
void update_ir_sensors();
bool duplo_detected_in_back_lift();
bool duplo_detected_in_front_lift();

#endif // IR_SENSOR_H
