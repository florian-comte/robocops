#include "maxon_driver.h"
#include "l298n_driver.h"
#include "servo_driver.h"
#include "maxon_encoder.h"
#include "commands.h"

// Communication baudrate
#define BAUDRATE 57600

// Max args for serial commands
#define MAX_ARGS 4

// Interval between PID updates in milliseconds
//#define LOOP_INTERVAL 50

// a String to hold incoming data
String input_string = "";

// whether the string is complete
bool is_string_completed = false;

// The target_speeds wanted for the motors
double maxon_target_speeds[MAXON_MOTOR_COUNT] = {0};
double l298n_target_speeds[L298N_MOTOR_COUNT] = {0};
double servo_target_angles[SERVO_MOTOR_COUNT] = {0};

/**
   @brief Handle incoming serial commands. See commands.h for more infos
*/
void handle_serial_command() {
  input_string.trim();
  if (input_string.length() == 0) return;

  char cmd = input_string.charAt(0);
  input_string = input_string.substring(1);
  input_string.trim();

  long args[MAX_ARGS];
  int arg_count = 0;

  // Tokenize using strtok
  char *token = strtok((char *)input_string.c_str(), " ");
  while (token != NULL && arg_count < MAX_ARGS) {
    char *endptr;
    long val = strtol(token, &endptr, 10);

    // Check for parsing errors
    if (*endptr != '\0') {
      Serial.println("Error: Invalid numeric value");
      return;
    }

    args[arg_count++] = val;
    token = strtok(NULL, " ");
  }

  switch (cmd) {
    case MOTOR_SPEEDS: // 'm' command
      if (arg_count != MAXON_MOTOR_COUNT) {
        Serial.println("Error: Expected motor speeds for each motor");
        return;
      }

      for (int i = 0; i < MAXON_MOTOR_COUNT; i++) {
        maxon_target_speeds[i] = (int)args[i];
      }

      Serial.println("Motor speeds received.");

      break;
    case L298N_SPEEDS: // 'l' command
      if (arg_count != L298N_MOTOR_COUNT){
        Serial.println("Error: Expected motor speeds for each motor");
        return;
      }

      for (int i = 0; i < L298N_MOTOR_COUNT; i++) {
        l298n_target_speeds[i] = (int)args[i];
      }

      Serial.println("Motor speeds received.");

      break;

     case SERVO_POSITIONS: // 's' command
      if (arg_count != SERVO_MOTOR_COUNT){
        Serial.println("Error: Expected motor speeds for each motor");
        return;
      }
  
      for (int i = 0; i < SERVO_MOTOR_COUNT; i++) {
        servo_target_angles[i] = (int)args[i];
      }
  
      Serial.println("Motor speeds received.");
  
      break;
      
    case ENCODERS_FEEDBACK: // 'e' command
    
      String encoder_output = "";
      for (int i = 0; i < MAXON_MOTOR_COUNT; i++) {
        encoder_output += read_maxon_encoder(i);
        encoder_output += " ";
      }
      
      Serial.println(encoder_output);
      break;
    default:
      Serial.println("Error: Unknown command");
      break;
  }
}


/**
   @brief Arduino setup function. Initializes serial communication,
   motor drivers, encoders, and resets PID states.
*/
void setup() {
  Serial.begin(BAUDRATE);

  input_string.reserve(200);

  init_maxon_motor_drivers();
  init_maxon_motor_encoders();
  init_servo_motors();
  init_l298n_motor_drivers();

  for (int i = 0; i < MAXON_MOTOR_COUNT; i++) {
    maxon_target_speeds[i] = 0;
  }

  for (int i = 0; i < L298N_MOTOR_COUNT; i++) {
    l298n_target_speeds[i] = 0;
  }

  for (int i = 0; i < SERVO_MOTOR_COUNT; i++) {
    set_servo_motor_angle(i, 0); 
  }
}

/**
   @brief Main Arduino loop. Handles serial commands and runs the PID update
   at a fixed interval for each motor.
*/
void loop() {
  if (is_string_completed) {
    handle_serial_command();
    input_string = "";
    is_string_completed = false;
  }

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
  
  // Servo
  for (int i = 0; i < SERVO_MOTOR_COUNT; i++) {
    set_servo_motor_angle(i, servo_target_angles[i]); 
  }
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  if (Serial.available()) {
    char inChar = (char)Serial.read();
    input_string += inChar;
    if (inChar == '\n') {
      is_string_completed = true;
    }
  }
}
