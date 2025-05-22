#ifndef LIFT_ULTRASOUND_SENSOR_H
#define LIFT_ULTRASOUND_SENSOR_H

#include <Arduino.h>
#include <NewPing.h>

#define LIFT_ULTRASOUND_WINDOW 20
#define LIFT_ULTRASOUND_MAX_DISTANCE 50

extern int lift_ultrasound_averaged_distance;

void update_lift_ultrasound_sensor();
void init_lift_ultrasound_sensor();

#endif // LIFT_ULTRASOUND_SENSOR_H
