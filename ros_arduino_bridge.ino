#include "maxon_driver.h"
#include "l298n_driver.h"
#include "servo_driver.h"
#include "dri_driver.h"
#include "maxon_encoder.h"
#include "commands.h"
#include "routines.h"

// Communication baudrate
#define BAUDRATE 57600

// PWM from 0 to 255
#define BRUSH_SPEED 200
#define CONVOYER_SPEED 200

// The target_speeds wanted for the motors
double maxon_target_speeds[MAXON_MOTOR_COUNT] = {MAXON_MIN_PWM};
double l298n_target_speeds[L298N_MOTOR_COUNT] = {L298N_MIN_PWM};
double servo_target_angles[SERVO_MOTOR_COUNT] = {0};
double dri_target_speeds[DRI_MOTOR_COUNT] = {DRI_MIN_PWM};

/**
   @brief Arduino setup function. Initializes serial communication,
   motor drivers, encoders, and resets PID states.
*/
void setup() {
  Serial.begin(BAUDRATE);

  init_maxon_motor_drivers();
  init_maxon_motor_encoders();
  init_servo_motors_drivers();
  init_l298n_motor_drivers();
  init_dri_motor_drivers();

  // Init unload routine
  unload_state = UNLOAD_OPEN_LATCH; 

  // Init lift routine
  // todo
}

/**
   @brief Main Arduino loop. Handles serial commands and runs the PID update
   at a fixed interval for each motor.
*/
void loop() {
  handle_serial_command();
  handle_routines();

  // Maxon motors
  for (int i = 0; i < MAXON_MOTOR_COUNT; i++) {
    int enable = maxon_target_speeds[i] != 0;
    int direction = (maxon_target_speeds[i] >= 0);
    int pwm = map(abs(maxon_target_speeds[i]), MAXON_MIN_MOTOR_SPEED, MAXON_MAX_MOTOR_SPEED, MAXON_MIN_PWM, MAXON_MAX_PWM);
    set_maxon_motor_state(i, enable, direction, pwm);
  }

  // L298N
  for (int i = 0; i < L298N_MOTOR_COUNT; i++) {
    int enable = l298n_target_speeds[i] != 0;
    int direction = (l298n_target_speeds[i] >= 0);
    int pwm = abs(l298n_target_speeds[i]);
    set_l298n_motor_state(i, direction, pwm);
  }

  // DRI
  for (int i = 0; i < DRI_MOTOR_COUNT; i++) {
    set_dri_motor_state(i, dri_target_speeds[i]); 
  }
  
  // Servo
  for (int i = 0; i < SERVO_MOTOR_COUNT; i++) {
    set_servo_motor_angle(i, servo_target_angles[i]); 
  }
}

void handle_serial_command() {
  if (Serial.available() >= 5) {
    byte buf[5];
    Serial.readBytes(buf, 5);

    // Maxon left (15 bits)
    int16_t maxon_left = ((buf[0] << 7) | (buf[1] >> 1));
    maxon_left -= 10000;

    // Maxon right (15 bits)
    int16_t maxon_right = (((buf[1] & 0x01) << 14) | (buf[2] << 6) | (buf[3] >> 2));
    maxon_right -= 10000;

    // 1 = Activate brush, 0 = Deactivate brush
    bool brush_signal = (buf[3] >> 1) & 0x01;

    if(brush_signal = 1){
      servo_target_angles[BRUSH_LEFT] = BRUSH_SPEED;
      servo_target_angles[BRUSH_RIGHT] = BRUSH_SPEED;
    } else {
      servo_target_angles[BRUSH_LEFT] = 0;
      servo_target_angles[BRUSH_RIGHT] = 0;
    }

    // 1 = Activate unload routine, 0 = Close back
    bool activate_unload_routine = (buf[3] >> 0) & 0x01;

    if(unload_state == UNLOAD_IDLE){
      unload_state = UNLOAD_OPEN_LATCH; 
    }

    // 1 = Activate lift routine, 0 or 1 where already activate = nothing 
    bool activate_lift_routine = (buf[4] >> 7) & 0x01;

    // todo, waiting for michel's code

    // Apply speeds
    maxon_target_speeds[MAXON_REAR_LEFT] = maxon_left;
    maxon_target_speeds[MAXON_REAR_RIGHT] = maxon_right;

    // Return the  is_brush_activate, is_unload_routine_activate, is_lift_activate
    int16_t maxon_encoder_left = read_maxon_encoder(MAXON_REAR_LEFT);
    int16_t maxon_encoder_right = read_maxon_encoder(MAXON_REAR_RIGHT);

    byte response[5];

    response[0] = highByte(maxon_encoder_left);
    response[1] = lowByte(maxon_encoder_left);
    response[2] = highByte(maxon_encoder_right);
    response[3] = lowByte(maxon_encoder_right);

    // Bit-packed flags:
    // bit 0 = brush active
    // bit 1 = unload routine active (non-IDLE)
    // bit 2 = todo: lift active
    response[4] = 0;
    response[4] |= (brush_signal << 0);
    response[4] |= ((unload_state != UNLOAD_IDLE) << 1);
    
    //todo lift

    Serial.write(response, 5);
  }
}
