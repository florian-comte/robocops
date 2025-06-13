#ifndef BACK_ULTRASOUND_SENSOR_H
#define BACK_ULTRASOUND_SENSOR_H

#include <Arduino.h>
#include <NewPing.h>

#define BACK_ULTRASOUND_WINDOW 2
#define BACK_ULTRASOUND_MAX_DISTANCE 500

extern double back_ultrasound_averaged_distance;

void update_back_ultrasound_sensor();
void init_back_ultrasound_sensor();

#endif // BACK_ULTRASOUND_SENSOR_H
