import serial
PORT = "COM4"        # Windows: viz Správce zařízení; Linux: /dev/ttyACM0 nebo /dev/ttyUSB0
BAUD = 115200
with serial.Serial(PORT, BAUD, timeout=1) as ser:
    while True:
        line = ser.readline().decode("utf-8", errors="replace").strip()
        if line:
            print(line)
