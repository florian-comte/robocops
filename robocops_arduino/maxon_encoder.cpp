#include "maxon_encoder.h"
#include "maxon_driver.h"

/**
 * @brief Analog input pin mapping for motor encoder speed signals.
 * These pins are used to read the encoder voltage corresponding to motor RPM.
 */
const int MAXON_MOTOR_ENCODER_SPEED_PINS[MAXON_MOTOR_COUNT] = {
   A4,  // REAR_RIGHT, A5
   A6 // REAR_LEFT A7
   //    40, // FRONT_LEFT
   // 41, // FRONT_RIGHT
    
   
};

double maxon_encoder_speeds[MAXON_MOTOR_COUNT];


/**
 * @brief Initializes motor encoder pins.
 *
 * Sets each encoder pin to input mode with an internal pull-up resistor.
 */
void init_maxon_motor_encoders() {
  for (int i = 0; i < MAXON_MOTOR_COUNT; i++) {
      pinMode(MAXON_MOTOR_ENCODER_SPEED_PINS[i], INPUT);
  }
}

/**
 * @brief Reads the encoder voltage for a given motor and converts it to speed in RPM.
 *
 * Reads the analog voltage and maps it to the encoder's RPM range [MIN_RPM to MAX_RPM, see in maxon_encoder.h].
 *
 * @param motor The motor position.
 * @return The motor speed in RPM.
 */
float read_maxon_encoder(maxon_motor_position motor){
  int raw_analog = analogRead(MAXON_MOTOR_ENCODER_SPEED_PINS[motor]);

  // 0 et 818.4

  // 0 -> -10 000 ou 10 000

  // - 818.4/2 et 818.4/2
  raw_analog = raw_analog - MAXON_MAX_ANALOG_VALUE / 2;

  raw_analog = IS_INVERSED_MAXON_MOTOR[motor] ? raw_analog : -raw_analog;

  // Map voltage range [0–MAX_ANALOG_VALUE] to speed range [-10000 RPM to 10000 RPM]
  return map(raw_analog, -MAXON_MAX_ANALOG_VALUE/2, MAXON_MAX_ANALOG_VALUE/2, MAXON_MIN_ENCODER_SPEED, MAXON_MAX_ENCODER_SPEED);  
}
