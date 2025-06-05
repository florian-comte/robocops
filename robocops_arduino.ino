#include "maxon_driver.h"
#include "l298n_driver.h"
#include "servo_driver.h"
#include "dri_driver.h"
#include "maxon_encoder.h"
#include "routines.h"
#include "lift_driver.h"
#include "ir_sensor.h"

// Communication baudrate
#define BAUDRATE 57600

bool emergency_stop = 0;

/**
   @brief Arduino setup function.
*/
void setup() {
  Serial.begin(BAUDRATE);

  init_maxon_motor_drivers();
  init_maxon_motor_encoders();
  init_servo_motors_drivers();
  init_l298n_motor_drivers();
  init_dri_motor_drivers();
  init_ir_sensors();
  init_lift();

  unload_state = UNLOAD_IDLE; 
  lift_state = LIFT_IDLE;
  button_state = BUTTON_IDLE;
  slope_up_state = SLOPE_UP_IDLE;
  slope_down_state = SLOPE_DOWN_IDLE;
  capture_state = CAPTURE_IDLE;
}

/**
   @brief Main Arduino loop.
*/
void loop() {
  handle_serial_command();

  // Check emergency
  if(emergency_stop){
      for (int i = 0; i < MAXON_MOTOR_COUNT; i++) {
        set_maxon_motor_state(i, 0, 0, 0);
      }

      for (int i = 0; i < L298N_MOTOR_COUNT; i++) {
        set_l298n_motor_state(i, 0, 0);
      }
    
      for (int i = 0; i < DRI_MOTOR_COUNT; i++) {
        set_dri_motor_state(i, 0, 0); 
      }
    return;
  }

  // --- Update routines
  handle_routines();

  // --- Update states
  if(lift_state == LIFT_IDLE){
    update_ir_sensors();
  } else {
    for(int i = 0; i < IR_SENSOR_COUNT; i++){
      ir_activated_averaged[i] = 0;
    }
  }
  
  // --- Update commands

  // Maxon motors
  for (int i = 0; i < MAXON_MOTOR_COUNT; i++) {
    int enable = maxon_target_speeds[i] != 0;
    int direction = (maxon_target_speeds[i] >= 0);
    int pwm = map(abs(maxon_target_speeds[i]), MAXON_MIN_MOTOR_SPEED, MAXON_MAX_MOTOR_SPEED, MAXON_MIN_PWM, MAXON_MAX_PWM);
    set_maxon_motor_state(i, enable, direction, pwm);
    maxon_encoder_speeds[i] = read_maxon_encoder(i);
  }

  // Servo
  for (int i = 0; i < SERVO_MOTOR_COUNT; i++) {
    set_servo_motor_angle(i, servo_target_angles[i]); 
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
    set_dri_motor_state(i, (dri_target_speeds[i] >= 0), abs(dri_target_speeds[i])); 
  }

  // Lift
  update_lift();
}

// From raspberry
// 16 bits maxon_left (buf[0] & buf[1])
// 16 bits maxon_right (buf[2] and buf[3])
// 1 bit activate brush (buf[4] >> 0)
// 1 bit activate unload (buf[4] >> 1)
// 1 bit authorized_lift (buf[4] >> 2)

// To raspberry
// 16 bits maxon_left_encoder (buf[0] & buf[1])
// 16 bits maxon_right_encoder (buf[2] & buf[3])
// 1 bit active brush (buf[4] >> 0)
// 1 bit active unload (buf[4] >> 1)
// 1 bit authorized_lift (buf[4] >> 2)
// 1 bit active_lift (buf[4] >> 3)

int handle_serial_command() {
  if (Serial.available() >= 5) {    
    byte buf[5];
    Serial.readBytes(buf, 5);

    // Maxon left (16 bits)
    int16_t maxon_left = ((buf[0] << 8) | buf[1]);
    maxon_left -= 10000;

    // Maxon right (16 bits)
    int16_t maxon_right = ((buf[2] << 8) | buf[3]);
    maxon_right -= 10000;    

    // 1 = Activate brush, 0 = Deactivate brush
    bool brush_activated = buf[4] & 0x01;

    // 1 = Activate unload routine, 0 = Close back
    bool activate_unload_routine = (buf[4] >> 1) & 0x01;

    bool activate_button_routine = (buf[4] >> 3) & 0x01;

    emergency_stop = (buf[4] >> 4) & 0x01;

    bool activate_slope_up_routine = (buf[4] >> 5) & 0x01;
    bool activate_slope_down_routine = (buf[4] >> 6) & 0x01;

    if(!emergency_stop){
      // Activate / deactivate capture routine
      if(brush_activated == 1){
        if(capture_state == CAPTURE_IDLE && lift_state == LIFT_IDLE){
            capture_state = CAPTURE_BRUSHING;
        }
      } else {
        capture_state = CAPTURE_IDLE;
      }
  
      if((activate_unload_routine == 1) && lift_state != LIFT_REVERSE_CONVOYER && lift_state != LIFT_DOWN && (unload_state == UNLOAD_IDLE) && ((button_state == BUTTON_IDLE) || (button_state == BUTTON_FINISHED))){
        unload_state = UNLOAD_OPEN_DOOR; 
      }
  
      if((activate_button_routine == 1) && (button_state == BUTTON_IDLE) && (unload_state == UNLOAD_IDLE)){
        button_state = BUTTON_BACKWARD; 
      }

      if((activate_slope_up_routine == 1) && slope_up_state == SLOPE_UP_IDLE){
        // todo
        slope_up_state = SLOPE_UP_IDLE; 
      }

      if((activate_slope_down_routine == 1) && slope_down_state == SLOPE_DOWN_IDLE){
        // todo
        slope_down_state = SLOPE_DOWN_IDLE; 
      }
      
      // Apply speeds
      if((button_state == BUTTON_FINISHED || button_state == BUTTON_IDLE) && slope_up_state == SLOPE_UP_IDLE && slope_down_state == SLOPE_DOWN_IDLE) {
        maxon_target_speeds[MAXON_REAR_LEFT] = maxon_left;
        maxon_target_speeds[MAXON_REAR_RIGHT] = maxon_right;
      }

    }
    // Return infos to rasp
    int16_t maxon_encoder_left = maxon_encoder_speeds[MAXON_REAR_LEFT] + 10000;
    int16_t maxon_encoder_right = maxon_encoder_speeds[MAXON_REAR_RIGHT] + 10000;

    byte response[5];

    response[0] = highByte(maxon_encoder_left);
    response[1] = lowByte(maxon_encoder_left);
    response[2] = highByte(maxon_encoder_right);
    response[3] = lowByte(maxon_encoder_right);

    response[4] = 0;
    response[4] |= brush_activated & 0x01;
    response[4] |= (((unload_state != UNLOAD_IDLE) & 0x01) << 1);
    response[4] |= (0 & 0x01) << 2;
    response[4] |= ((lift_state != LIFT_IDLE) & 0x01) << 3;
    response[4] |= ((button_state != BUTTON_IDLE) & 0x01) << 4;
    response[4] |= (emergency_stop & 0x01) << 5;
    response[4] |= ((slope_up_state != SLOPE_UP_IDLE) & 0x01) << 6;
    response[4] |= ((slope_down_state != SLOPE_DOWN_IDLE) & 0x01) << 7;

    Serial.write(response, 5);
  }
}
