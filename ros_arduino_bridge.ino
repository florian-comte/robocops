#include "maxon_driver.h"
#include "maxon_encoder.h"
#include "pid.h"
#include "commands.h"

// Communication baudrate
#define BAUDRATE 57600

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
  input_string.trim(); // Remove leading/trailing whitespace

  if (input_string.length() == 0) return;

  // Extract command character (e.g. 's')
  char cmd = input_string.charAt(0);
  // Remove the command character from the line
  input_string = input_string.substring(1);
  input_string.trim();

  int values[MOTOR_COUNT];
  int i = 0;

  // Tokenize the rest of the line by space
  char *token = strtok((char*)input_string.c_str(), " ");
  
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
      
     case ENCODERS_FEEDBACK: // 'e' command
        // Create a string to hold encoder values
        String encoder_output = "";
        
        // Read and append each encoder's position
        for (int j = 0; j < MOTOR_COUNT; j++) {
          long position = read_encoder(j);
          encoder_output += " ";
          encoder_output += String(position);
        }
        
        // Send the encoder data back over serial
        Serial.println(encoder_output);
     
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
