# robocops_arduino – Team RoboCops (EPFL Robot Competition 2025)

Arduino firmware acting as the hardware interface between the Raspberry Pi and the robot's actuators/sensors.

Author: Florian Comte (florian.comte@epfl.ch)

## Serial Communication Protocol

This firmware uses a **binary serial protocol** (not plain text). It exchanges fixed-length messages between the Raspberry Pi and the Arduino using a **custom 5-byte command** and **11-byte state** format, with **start (0xAA)** and **end (0x55)** markers.

### Default baud rate: `115200`

### Commands from Raspberry Pi (5 bytes):

| Bytes | Description                                                                                                                                                                           |
| ----- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [0–1] | 16-bit signed int: `command_maxon_left` speed (offset by +10000)                                                                                                                      |
| [2–3] | 16-bit signed int: `command_maxon_right` speed (offset by +10000)                                                                                                                     |
| [4]   | Bitfield:<br>bit 0: `command_capture`<br>bit 1: `command_unload`<br>bit 2: `command_button`<br>bit 3: `command_slope_up`<br>bit 4: `command_slope_down`<br>bit 5: `command_emergency` |

### States to Raspberry Pi (11 bytes):

| Bytes  | Description                                                                                                                                                                                 |
| ------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [0–1]  | 16-bit signed int: `state_maxon_left` (offset by +10000 when read from encoder)                                                                                                             |
| [2–3]  | 16-bit signed int: `state_maxon_right`                                                                                                                                                      |
| [4]    | Bitfield:<br>bit 0: capture active<br>bit 1: unload active<br>bit 2: button routine active<br>bit 3: slope up routine active<br>bit 4: slope down routine active<br>bit 5: emergency active |
| [5–6]  | 16-bit unsigned int: number of Duplos captured                                                                                                                                              |
| [7–8]  | 16-bit int: back ultrasound sensor distance                                                                                                                                                 |
| [9–10] | 16-bit int: `state_other` (reserved for future use/debug)                                                                                                                                   |

---

## Configuration

Feel free to go through the files to update the different values in the #define (mainly the pinouts, the timing for the routines, ...).

## File Structure

| File                       | Description                                                   |
| -------------------------- | ------------------------------------------------------------- |
| `robocops_arduino.ino`     | Main control loop and serial interface                        |
| `maxon_driver.*`           | Control for Maxon motors (PWM, direction)                     |
| `maxon_encoder.*`          | Encoder read and speed estimation                             |
| `servo_driver.*`           | Servo angle control                                           |
| `l298n_driver.*`           | Control of brushed motors using L298N                         |
| `dri_driver.*`             | Control of brushed motors using DRI driver                    |
| `lift_driver.*`            | State machine for lift and unloading                          |
| `ir_sensor.*`              | IR line-following sensor averaging and reading                |
| `back_ultrasound_sensor.*` | Distance measurement for button routine                       |
| `routines.*`               | Logic for composite robot routines (capture, slopes, buttons) |

You can also find in the tester folder some to tools I used to test my code without using the ros2_control package. Easier to test.

---

## Some notes

- **Routine-based behaviors**: capture, unload, slope up/down, and button press
- **Modular and extensible motor/sensor interface**
