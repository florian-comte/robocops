#include "maxon_encoder.h"

/**
 * @brief Analog input pin mapping for motor encoder speed signals.
 * These pins are used to read the encoder voltage corresponding to motor RPM.
 */
const int MAXON_MOTOR_ENCODER_SPEED_PINS[MAXON_MOTOR_COUNT] = {
   A0,  // REAR_RIGHT,
   A1 // REAR_LEFT
   //    40, // FRONT_LEFT
   // 41, // FRONT_RIGHT
    
   
};

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

  // Map voltage range [0–MAX_ANALOG_VALUE] to speed range [-10000 RPM to 10000 RPM]
  return map(raw_analog, 0, MAXON_MAX_ANALOG_VALUE, MAXON_MIN_ENCODER_SPEED, MAXON_MAX_ENCODER_SPEED);  
}
