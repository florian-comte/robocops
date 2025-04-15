import serial
import time

COM_PORT = 'COM3'
BAUDRATE = 57600

speeds = [-2000]  # RPM

# Format: s <speed1> <...> <speedn>
command = f"s {' '.join(map(str, speeds))}\n"

# Open serial connection
with serial.Serial(COM_PORT, BAUDRATE, timeout=1) as ser:
    time.sleep(2)

    print(f"Sending command: {command.strip()}")
    ser.write(command.encode())

    response = ser.readline().decode().strip()
    print(f"Arduino says: {response}")
