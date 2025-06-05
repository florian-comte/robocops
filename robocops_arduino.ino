#include "maxon_driver.h"
#include "l298n_driver.h"
#include "servo_driver.h"
#include "dri_driver.h"
#include "maxon_encoder.h"
#include "routines.h"
#include "lift_driver.h"
#include "ir_sensor.h"

#define BAUDRATE 57600
#define STATES_BUFFER_SIZE 11
#define COMMANDS_BUFFER_SIZE 5

byte commands_buffer[COMMANDS_BUFFER_SIZE];
byte states_buffer[STATES_BUFFER_SIZE];

bool command_emergency = 0;

bool command_capture = 0;
bool command_unload = 0;
bool command_button = 0;
bool command_slope_up = 0;
bool command_slope_down = 0;

int command_maxon_left = 0;
int command_maxon_right = 0;

int16_t state_maxon_left = 0;
int16_t state_maxon_right = 0;

int16_t state_other_one = 0;
int16_t state_other_two = 0;

int16_t state_nb_captured_duplos = 0;

void update_commands(){
  if(command_emergency){
    for (int i = 0; i < MAXON_MOTOR_COUNT; i++) {
      set_maxon_motor_state(i, 0, 0, 0);
    }

    for (int i = 0; i < L298N_MOTOR_COUNT; i++) {
      set_l298n_motor_state(i, 0, 0);
    }
  
    for (int i = 0; i < DRI_MOTOR_COUNT; i++) {
      set_dri_motor_state(i, 0, 0); 
    }
  } else {
    // Activate / deactivate capture routine
    if(command_capture){
      if(capture_state == CAPTURE_IDLE && lift_state == LIFT_IDLE){
          capture_state = CAPTURE_BRUSHING;
          state_nb_captured_duplos++;
      }
    } else {
      capture_state = CAPTURE_IDLE;
    }
    
    if((command_unload == 1) && lift_state != LIFT_REVERSE_CONVOYER && lift_state != LIFT_DOWN && (unload_state == UNLOAD_IDLE) && ((button_state == BUTTON_IDLE) || (button_state == BUTTON_FINISHED))){
      unload_state = UNLOAD_OPEN_DOOR; 
    }
    
    if((command_button == 1) && (button_state == BUTTON_IDLE) && (unload_state == UNLOAD_IDLE)){
      button_state = BUTTON_BACKWARD; 
    }
    
    if((command_slope_up == 1) && slope_up_state == SLOPE_UP_IDLE){
      // todo
      slope_up_state = SLOPE_UP_IDLE; 
    }
    
    if((command_slope_down == 1) && slope_down_state == SLOPE_DOWN_IDLE){
      // todo
      slope_down_state = SLOPE_DOWN_IDLE; 
    }
    
    // Apply speeds
    if((button_state == BUTTON_FINISHED || button_state == BUTTON_IDLE) && slope_up_state == SLOPE_UP_IDLE && slope_down_state == SLOPE_DOWN_IDLE) {
      maxon_target_speeds[MAXON_REAR_LEFT] = command_maxon_left;
      maxon_target_speeds[MAXON_REAR_RIGHT] = command_maxon_right;
    }
    
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
}

void update_states(){
  // only update ir sensor if lift not working
  if(lift_state == LIFT_IDLE){
    update_ir_sensors();
  } else {
    for(int i = 0; i < IR_SENSOR_COUNT; i++){
      ir_activated_averaged[i] = 0;
    }
  }

  // Update maxon encoder states
  state_maxon_left = maxon_encoder_speeds[MAXON_REAR_LEFT] + 10000;
  state_maxon_right = maxon_encoder_speeds[MAXON_REAR_RIGHT] + 10000;

  // Update additional state values
}

void setup() {
  Serial.begin(BAUDRATE);

  init_maxon_motor_drivers();
  init_maxon_motor_encoders();
  init_servo_motors_drivers();
  init_l298n_motor_drivers();
  init_dri_motor_drivers();
  init_ir_sensors();
  init_lift();
  init_routines();
}

void loop() {
  handle_serial_command();
  handle_routines();

  update_states();
  update_commands();
}

// From Raspberry Pi (5 bytes):
//  - 16 bits: wanted_maxon_left    (commands_buffer[0] << 8 | commands_buffer[1]) - 10000
//  - 16 bits: wanted_maxon_right   (commands_buffer[2] << 8 | commands_buffer[3]) - 10000
//  - 1 bit : command_capture       (commands_buffer[4] >> 0) & 0x01
//  - 1 bit : command_unload        (commands_buffer[4] >> 1) & 0x01
//  - 1 bit : command_button        (commands_buffer[4] >> 2) & 0x01
//  - 1 bit : command_slope_up      (commands_buffer[4] >> 3) & 0x01
//  - 1 bit : command_slope_down    (commands_buffer[4] >> 4) & 0x01
//  - 1 bit : command_emergency     (commands_buffer[4] >> 5) & 0x01

// To Raspberry Pi (11 bytes):
//  - 16 bits: state_maxon_left     (states_buffer[0] << 8 | states_buffer[1])
//  - 16 bits: state_maxon_right    (states_buffer[2] << 8 | states_buffer[3])
//  - 1 bit : capture_active        (states_buffer[4] >> 0) & 0x01
//  - 1 bit : unload_active         (states_buffer[4] >> 1) & 0x01
//  - 1 bit : button_active         (states_buffer[4] >> 2) & 0x01
//  - 1 bit : slope_up_active       (states_buffer[4] >> 3) & 0x01
//  - 1 bit : slope_down_active     (states_buffer[4] >> 4) & 0x01
//  - 1 bit : emergency_active      (states_buffer[4] >> 5) & 0x01
//  - 16 bits: nb_duplos_captured   (states_buffer[5] << 8 | states_buffer[6])
//  - 16 bits: state_other_one      (states_buffer[7] << 8 | states_buffer[8])
//  - 16 bits: state_other_two      (states_buffer[9] << 8 | states_buffer[10])

int handle_serial_command() {
  if (Serial.available() >= 5) {    
    Serial.readBytes(commands_buffer, 5);

    // Maxon left (16 bits)
    command_maxon_left = ((commands_buffer[0] << 8) | commands_buffer[1]);
    command_maxon_left -= 10000;

    // Maxon right (16 bits)
    command_maxon_right = ((commands_buffer[2] << 8) | commands_buffer[3]);
    command_maxon_right -= 10000;    

    //  Send (handle) commands to arduino
    command_capture = commands_buffer[4] & 0x01;
    command_unload = (commands_buffer[4] >> 1) & 0x01;
    command_button = (commands_buffer[4] >> 2) & 0x01;
    command_slope_up = (commands_buffer[4] >> 3) & 0x01;
    command_slope_down = (commands_buffer[4] >> 4) & 0x01;
    command_emergency = (commands_buffer[4] >> 5) & 0x01;

    // Send states to raspberry
    states_buffer[0] = highByte(state_maxon_left);
    states_buffer[1] = lowByte(state_maxon_left);
    states_buffer[2] = highByte(state_maxon_right);
    states_buffer[3] = lowByte(state_maxon_right);

    states_buffer[4] = 0;
    states_buffer[4] |= ((capture_state != CAPTURE_IDLE) & 0x01);
    states_buffer[4] |= ((unload_state != UNLOAD_IDLE) & 0x01) << 1;
    states_buffer[4] |= ((button_state != BUTTON_IDLE) & 0x01) << 2;
    states_buffer[4] |= ((slope_up_state != SLOPE_UP_IDLE) & 0x01) << 3;
    states_buffer[4] |= ((slope_down_state != SLOPE_DOWN_IDLE) & 0x01) << 4;
    states_buffer[4] |= (command_emergency & 0x01) << 5;

    states_buffer[5] = highByte(state_nb_captured_duplos);
    states_buffer[6] = lowByte(state_nb_captured_duplos);

    states_buffer[7] = highByte(state_other_one);
    states_buffer[8] = lowByte(state_other_one);
    states_buffer[9] = highByte(state_other_two);
    states_buffer[10] = lowByte(state_other_two);
    
    Serial.write(states_buffer, 11);
  }
}
