#include "maxon_driver.h"
#include "maxon_encoder.h"
#include "pid.h"
#include "commands.h"

#define BAUDRATE 57600

// Time tracking for periodic PID updates
unsigned long last_loop_time = 0;
// Interval between PID updates in milliseconds
const unsigned long loop_interval = 50; // ms

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
  if (Serial.available()) {
    // Read the incoming command line up to newline
    String line = Serial.readStringUntil('\n'); 
    line.trim(); // Remove leading/trailing whitespace

    if (line.length() == 0) return; // Ignore empty lines

    // Extract command character (e.g. 's')
    char cmd = line.charAt(0);
    // Remove the command character from the line
    line = line.substring(1);
    line.trim(); // Trim again to clean up spaces

    int values[MOTOR_COUNT]; // Array to store parsed motor values
    int i = 0;

    // Tokenize the rest of the line by space
    char *token = strtok((char*)line.c_str(), " ");
    
    // Parse each value and store it
    while (token != NULL && i < MOTOR_COUNT) {
      values[i++] = atoi(token); // Convert string to int
      token = strtok(NULL, " ");
    }

    // Check if correct number of motor values were received
    if (i != MOTOR_COUNT) {
      Serial.println("Error: Invalid number of values");
      return;
    }

    // Handle the parsed command
    switch (cmd) {
      case MOTOR_SPEEDS: // 's' command for setting motor speeds
        for (int j = 0; j < MOTOR_COUNT; j++) {
          if (values[j] == 0) {
            is_moving[j] = 0; // Mark motor as idle
          } else {
            is_moving[j] = 1; // Mark motor as active
          }
          target_speeds[j] = values[j]; // Set new target speed
        }
        Serial.println("Updated motor speeds.");
        break;

      default:
        Serial.println("Error: Unknown command");
        break;
    }
  }
}

/**
 * @brief Arduino setup function. Initializes serial communication,
 * motor drivers, encoders, and resets PID states.
 */
void setup() {
  Serial.begin(BAUDRATE);
  
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
  handle_serial_command(); // Handle any new commands from serial

  unsigned long now = millis();

  // Run PID update every loop_interval milliseconds
  if (now - last_loop_time >= loop_interval) {
    last_loop_time = now;

    for (int i = 0; i < MOTOR_COUNT; i++) {
      update_pid(i);
    }
  }  
}
