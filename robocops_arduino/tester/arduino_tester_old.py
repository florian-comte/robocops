# OLD tester: you should use the arduino_tester.py
import serial
import time

COM_PORT = 'COM4'
BAUDRATE = 57600
MOTOR_COUNT = 1

def is_valid_speed_list(values):
    if len(values) != MOTOR_COUNT:
        print(f"Error: Expected {MOTOR_COUNT} values.")
        return False
    try:
        [int(v) for v in values]
        return True
    except ValueError:
        print("Error: All values must be integers.")
        return False

def main():
    with serial.Serial(COM_PORT, BAUDRATE, timeout=1) as ser:
        print("Waiting for Arduino to boot...")
        time.sleep(2)  # Give Arduino time to reset and be ready

        print("Connected to Arduino.")
        print(f"Type {MOTOR_COUNT} motor speeds (e.g. -1000 0 1000 0), or 'q' to quit.")

        while True:
            user_input = input(">> ").strip()
            
            if user_input.lower() in ('q', 'quit', 'exit'):
                print("Exiting.")
                break

            values = user_input.split()
            # if not is_valid_speed_list(values):
            #     continue

            command = f"s {' '.join(values)}\n"
            ser.write(command.encode())

            try:
                response = ser.readline().decode().strip()
                if response:
                    print(f"Arduino says: {response}")
            except:
                print("No response or decode error.")

if __name__ == "__main__":
    main()