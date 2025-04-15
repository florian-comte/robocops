#include "maxon_driver.h"
#include "maxon_encoder.h"
#include "pid.h"
#include "commands.h"

// Communication baudrate
#define BAUDRATE 57600

// Max args for serial commands
#define MAX_ARGS 4

// Interval between PID updates in milliseconds
#define LOOP_INTERVAL 50

// Time tracking for periodic PID updates
unsigned long last_loop_time = 0;

// a String to hold incoming data
String input_string = "";   

// whether the string is complete
bool is_string_completed = false;  

/**
 * @brief Handle incoming serial commands to control motor behavior.
 *
 * Expected format:
 *    s <speed1> <speed2> <speed3> <speed4>
 *
 * Where 's' indicates a speed command, followed by 4 integers representing
 * the target speeds (RPM) for each motor.
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
    case MOTOR_SPEEDS: // 's' command
      if (arg_count != MOTOR_COUNT) {
        Serial.println("Error: Expected motor speeds for each motor");
        return;
      }

      for (int i = 0; i < MOTOR_COUNT; i++) {
        int speed = (int)args[i];
        target_speeds[i] = speed;
        is_moving[i] = (speed != 0) ? 1 : 0;
      }

      Serial.println("Updated motor speeds.");
      break;

    case ENCODERS_FEEDBACK: // 'e' command
      {
        String encoder_output = "";
        for (int i = 0; i < MOTOR_COUNT; i++) {
          encoder_output += " ";
          encoder_output += String(read_encoder(i));
        }
        Serial.println(encoder_output);
      }
      break;

    case PID_VALUES: // 'p' command, expects 4 values: k_p, k_d, k_i, k_o
      if (arg_count != 4) {
        Serial.println("Error: Expected 4 PID values");
        return;
      }

      k_p = (int)args[0];
      k_d = (int)args[1];
      k_i = (int)args[2];
      k_o = (int)args[3];
      Serial.println("Updated PID values.");
      break;

    default:
      Serial.println("Error: Unknown command");
      break;
  }
}


/**
 * @brief Arduino setup function. Initializes serial communication,
 * motor drivers, encoders, and resets PID states.
 */
void setup() {
  Serial.begin(BAUDRATE);

  input_string.reserve(200);
  
  init_motor_drivers();
  init_motor_encoders();

  // Reset PID controller for each motor
  for (int i = 0; i < MOTOR_COUNT; i++) {
    reset_pid(i);
  }
}

/**
 * @brief Main Arduino loop. Handles serial commands and runs the PID update
 * at a fixed interval for each motor.
 */
void loop() {
  if (is_string_completed) {
    handle_serial_command();
    input_string = "";
    is_string_completed = false;
  }

  unsigned long now = millis();

  // Run PID update every loop_interval milliseconds
  if (now - last_loop_time >= LOOP_INTERVAL) {
    last_loop_time = now;

    for (int i = 0; i < MOTOR_COUNT; i++) {
      update_pid(i);
    }
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
