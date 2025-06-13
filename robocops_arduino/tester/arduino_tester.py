import serial
import struct
import time

SERIAL_PORT = 'COM3'
BAUDRATE = 115200

START_BYTE = 0xAA
END_BYTE = 0x55

def encode_command(maxon_left, maxon_right, capture=False, unload=False, button=False, slope_up=False, slope_down=False, emergency=False):
    # Offset by +10000 to ensure all values are positive
    maxon_left += 10000
    maxon_right += 10000

    buf = bytearray(7)
    buf[0] = START_BYTE
    buf[1] = (maxon_left >> 8) & 0xFF
    buf[2] = maxon_left & 0xFF
    buf[3] = (maxon_right >> 8) & 0xFF
    buf[4] = maxon_right & 0xFF

    flags = 0
    flags |= int(capture)     << 0
    flags |= int(unload)      << 1
    flags |= int(button)      << 2
    flags |= int(slope_up)    << 3
    flags |= int(slope_down)  << 4
    flags |= int(emergency)   << 5

    buf[5] = flags
    buf[6] = END_BYTE
    return buf

def send_command(ser, maxon_left, maxon_right, capture=False, unload=False, button=False, slope_up=False, slope_down=False, emergency=False):
    command = encode_command(maxon_left, maxon_right, capture, unload, button, slope_up, slope_down, emergency)
    ser.write(command)
    print("Sent:", list(command))

    # Wait and read 11-byte response
    response = ser.read(11)
    if len(response) == 11:
        left_speed = ((response[0] << 8) | response[1]) - 10000
        right_speed = ((response[2] << 8) | response[3]) - 10000
        status_flags = response[4]
        duplos = (response[5] << 8) | response[6]
        ultrasound = (response[7] << 8) | response[8]
        other_state = (response[9] << 8) | response[10]

        print("Received:")
        print(f"  Left Speed: {left_speed}")
        print(f"  Right Speed: {right_speed}")
        print(f"  Capture Active:    {bool(status_flags & (1 << 0))}")
        print(f"  Unload Active:     {bool(status_flags & (1 << 1))}")
        print(f"  Button Routine:    {bool(status_flags & (1 << 2))}")
        print(f"  Slope Up Active:   {bool(status_flags & (1 << 3))}")
        print(f"  Slope Down Active: {bool(status_flags & (1 << 4))}")
        print(f"  Emergency Active:  {bool(status_flags & (1 << 5))}")
        print(f"  Duplos Count: {duplos}")
        print(f"  Back Ultrasound: {ultrasound} mm")
        print(f"  Other State (debug): {other_state}")
    else:
        print("No or incomplete response received from Arduino.")

def parse_input(user_input):
    tokens = user_input.strip().split()
    if len(tokens) < 2:
        raise ValueError("Enter at least two numbers for left and right speeds.")

    maxon_left = int(tokens[0])
    maxon_right = int(tokens[1])
    capture = 'capture' in tokens
    unload = 'unload' in tokens
    button = 'button' in tokens
    slope_up = 'slope_up' in tokens
    slope_down = 'slope_down' in tokens
    emergency = 'emergency' in tokens

    return maxon_left, maxon_right, capture, unload, button, slope_up, slope_down, emergency

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        time.sleep(2)  # Give Arduino time to reboot
        print("Connected to Arduino. Type 'exit' to quit.")
        print("Enter command: <left> <right> [capture] [unload] [button] [slope_up] [slope_down] [emergency]")

        while True:
            user_input = input(">> ")
            if user_input.strip().lower() == 'exit':
                break

            try:
                args = parse_input(user_input)
                send_command(ser, *args)
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