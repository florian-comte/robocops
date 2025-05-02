#include "maxon_driver.h"
#include "maxon_encoder.h"
#include "commands.h"

// Communication baudrate
#define BAUDRATE 57600

// Max args for serial commands
#define MAX_ARGS 4

// Interval between PID updates in milliseconds
//#define LOOP_INTERVAL 50

// Time tracking for periodic PID updates
//unsigned long last_loop_time = 0;

// a String to hold incoming data
String input_string = "";

// whether the string is complete
bool is_string_completed = false;

// The target_speeds wanted for the motors
double target_speeds[MOTOR_COUNT] = {0};

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
    //case PID_VALUES: // 'p' command, expects 3 values: k_p, k_i, k_d
      //if (arg_count != 3) {
        //Serial.println("Error: Expected 3 PID values");
        //return;
      //}

      //Serial.println("Updated PID values.");
      //break;
    case MOTOR_SPEEDS: // 'm' command
      if (arg_count != MOTOR_COUNT) {
        Serial.println("Error: Expected motor speeds for each motor");
        return;
      }

      for (int i = 0; i < MOTOR_COUNT; i++) {
        target_speeds[i] = (int)args[i];
      }

      break;
    case ENCODERS_FEEDBACK: // 'e' command
    
      String encoder_output = "";
      for (int i = 0; i < MOTOR_COUNT; i++) {
        encoder_output += read_encoder(i);
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

  init_motor_drivers();
  init_motor_encoders();

  for (int i = 0; i < MOTOR_COUNT; i++) {
    target_speeds[i] = 0;
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

  for (int i = 0; i < MOTOR_COUNT; i++) {
    int enable = target_speeds[i] != 0;
    int direction = (target_speeds[i] >= 0) ? !IS_INVERSED_MOTOR[i] : IS_INVERSED_MOTOR[i];
    int pwm = map(abs(target_speeds[i]), MIN_MOTOR_SPEED, MAX_MOTOR_SPEED, MIN_PWM, MAX_PWM);
    set_motor_state(i, enable, direction, pwm);
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
