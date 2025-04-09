# ros_arduino_bridge of team RoboCops (EPFL Robot competition 2025)

Code of the arduino that is used as controller for our motors. 
For now it only have maxon motors but we can imagine using it also for the other motors + for the sensors.

Inspired by https://github.com/joshnewans/ros_arduino_bridge/tree/main which is a fork of https://github.com/hbrobotics/ros_arduino_bridge.

## Serial command interface
The system accepts simple text commands over serial (default baud rate: 57600).

### Set motor speeds

**Command format:**

```
s <speed1> <speed2> <speed3> <speed4>
```

**Example:** (set all motors to 1000 RPM):

```
s 1000 1000 1000 1000
```

**Parameters:**
- Integer values representing target speeds (RPM) for each motor
- Motor order: Front Left, Front Right, Rear Left, Rear Right (to be updated when more)
- `0` RPM stops the motor
- Valid RPM range: `-3500` to `3500` (as defined in firmware, see configuration section)
- Negative values indicate reverse rotation

## Configuration
Key parameters can be adjusted in these header files:

| File | Configurable Parameters |
|------|-------------------------|
| `maxon_driver.h` | PWM limits (`MIN_PWM`, `MAX_PWM`) |
| `maxon_encoder.h` | Speed ranges (`MIN_ENCODER_SPEED`, `MAX_ENCODER_SPEED`) |
| `pid.h` | PID constants (`KP`, `KI`, `KD`) |
| `ros_arduino_bridge.ino` | PID updates interval (`LOOP_INTERVAL`) |

## File Structure
| File | Description |
|------|-------------|
| `maxon_driver` | Motor driver interface and pin mappings |
| `maxon_encoder` | Encoder reading functions |
| `pid` | PID control implementation |
| `commands` | Serial command definitions |
| `utils` | Utility functions |
| `ros_arduino_bridge` | Main program |

## Usage Notes
- PID controller runs at fixed 20Hz interval (50ms), can be change (see Configuration section)
- Encoder feedback provides closed-loop speed control, motor speeds are maintained automatically once set
- System designed for expansion to additional motor types and sensors