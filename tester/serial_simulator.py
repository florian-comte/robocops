import serial
import struct
import time

SERIAL_PORT = 'COM3'
BAUDRATE = 57600

def encode_command(maxon_left, maxon_right, brush=False, unload=False, lift=False):
    maxon_left += 10000
    maxon_right += 10000

    buf = bytearray(5)
    buf[0] = (maxon_left >> 8) & 0xFF
    buf[1] = maxon_left & 0xFF
    buf[2] = (maxon_right >> 8) & 0xFF
    buf[3] = maxon_right & 0xFF

    control_flags = 0
    control_flags |= int(brush) << 0
    control_flags |= int(unload) << 1
    control_flags |= int(lift) << 2

    buf[4] = control_flags

    return buf

def send_command(ser, maxon_left, maxon_right, brush=False, unload=False, lift=False):
    command = encode_command(maxon_left, maxon_right, brush, unload, lift)
    ser.write(command)
    print("Sent:", list(command))

    response = ser.read(5)
    if len(response) == 5:
        maxon_encoder_left = (response[0] << 8) | response[1]
        maxon_encoder_right = (response[2] << 8) | response[3]
        status = response[4]
        print(f"Received:")
        print(f"  Maxon Encoder Left: {maxon_encoder_left}")
        print(f"  Maxon Encoder Right: {maxon_encoder_right}")
        print(f"  Brush: {bool(status & 0x01)}")
        print(f"  Unload Active: {bool((status >> 1) & 0x01)}")
        print(f"  Lift Authorized: {bool((status >> 2) & 0x01)}")
        print(f"  Lift Active: {bool((status >> 3) & 0x01)}")

    else:
        print("No response or incomplete response from Arduino.")

def parse_input(user_input):
    tokens = user_input.strip().split()
    if len(tokens) < 2:
        raise ValueError("You must enter at least two numbers for left and right motor speeds.")

    maxon_left = int(tokens[0])
    maxon_right = int(tokens[1])
    brush = 'brush' in tokens
    unload = 'unload' in tokens
    lift = 'lift' in tokens

    return maxon_left, maxon_right, brush, unload, lift

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        time.sleep(2)  # Wait for Arduino to reset
        print("Connected to Arduino. Type 'exit' to quit.")
        print("Enter command: <left> <right> [brush] [unload] [lift]")

        while True:
            user_input = input(">> ")
            if user_input.strip().lower() == 'exit':
                break

            try:
                maxon_left, maxon_right, brush, unload, lift = parse_input(user_input)
                send_command(ser, maxon_left, maxon_right, brush, unload, lift)
            except Exception as e:
                print(f"Invalid input: {e}")

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    main()
