#include "maxon_driver.h"
#include "l298n_driver.h"
#include "servo_driver.h"
#include "dri_driver.h"
#include "maxon_encoder.h"
#include "routines.h"
#include "lift_ultrasound_sensor.h"

// Communication baudrate
#define BAUDRATE 57600

// PWM from 0 to 255
#define BRUSH_SPEED 255

// Distance of ultrasound to activate the lift routine (cm)
#define ULTRASOUND_SENSOR_LIFT_DETECTION_VALUE 10

// todo: need to be replaced by michel's logic
bool active_lift_routine = 0;
bool authorized_lift_routine = 1;

double previous_loop = 0;
double loop_start = 0;

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
  init_lift_ultrasound_sensor();

  // Init unload routine
  unload_state = UNLOAD_OPEN_LATCH; 

  // Init lift routine
  // todo

}

/**
   @brief Main Arduino loop.
*/
void loop() {
  handle_serial_command();
  handle_routines();

  // Update lift ultrasound sensor
  if(authorized_lift_routine == 1 && active_lift_routine == 0){
      update_lift_ultrasound_sensor();

      // Check if routine should be started
      if(lift_ultrasound_averaged_distance <= ULTRASOUND_SENSOR_LIFT_DETECTION_VALUE){
        // todo michel: start lift routine
        active_lift_routine = 1;
      }
  } else {
    // By security, reset the current lift ultrasound value to MAX_DISTANCE when not updating it
    lift_ultrasound_averaged_distance = LIFT_ULTRASOUND_MAX_DISTANCE;
  }
  
  // Maxon motors
  for (int i = 0; i < MAXON_MOTOR_COUNT; i++) {
    int enable = maxon_target_speeds[i] != 0;
    int direction = (maxon_target_speeds[i] >= 0);
    int pwm = map(abs(maxon_target_speeds[i]), MAXON_MIN_MOTOR_SPEED, MAXON_MAX_MOTOR_SPEED, MAXON_MIN_PWM, MAXON_MAX_PWM);
    set_maxon_motor_state(i, enable, direction, pwm);
    maxon_encoder_speeds[i] = read_maxon_encoder(i);
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
    bool brush_signal = buf[4] & 0x01;

    // 1 = Activate unload routine, 0 = Close back
    bool activate_unload_routine = (buf[4] >> 1) & 0x01;

    // 1 = Authorized lift routine, 0 = Not authorized lift routine
    authorized_lift_routine = (buf[4] >> 2) & 0x01;

    // Activate / Deactivate brushes
    if(brush_signal == 1){
      l298n_target_speeds[BRUSH_LEFT] = BRUSH_SPEED;
      l298n_target_speeds[BRUSH_RIGHT] = BRUSH_SPEED;
    } else {
      l298n_target_speeds[BRUSH_LEFT] = L298N_MIN_PWM;
      l298n_target_speeds[BRUSH_RIGHT] = L298N_MIN_PWM;
    }

    if((activate_unload_routine == 1) && (unload_state == UNLOAD_IDLE)){
      unload_state = UNLOAD_OPEN_LATCH; 
    }
    
    // Apply speeds
    maxon_target_speeds[MAXON_REAR_LEFT] = maxon_left;
    maxon_target_speeds[MAXON_REAR_RIGHT] = maxon_right;

    // Return infos to rasp
    int16_t maxon_encoder_left = maxon_encoder_speeds[MAXON_REAR_LEFT];
    int16_t maxon_encoder_right = maxon_encoder_speeds[MAXON_REAR_RIGHT];

    byte response[5];

    response[0] = highByte(maxon_encoder_left);
    response[1] = lowByte(maxon_encoder_left);
    response[2] = highByte(maxon_encoder_right);
    response[3] = lowByte(maxon_encoder_right);

    response[4] = 0;
    response[4] |= brush_signal & 0x01;
    response[4] |= (((unload_state != UNLOAD_IDLE) & 0x01) << 1);
    response[4] |= (authorized_lift_routine & 0x01) << 2;
    response[4] |= (active_lift_routine & 0x01) << 3;
    
    Serial.write(response, 5);
  }
}
