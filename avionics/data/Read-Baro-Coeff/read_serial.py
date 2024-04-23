import serial

port = "/dev/ttyUSB0"
baudrate = 115200

try:
    ser = serial.Serial(port, baudrate)
    print("Serial port connected")
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit()

filename = "coeff.txt"

try:
    with open(filename, "ab") as f:
        print(f"Writing data to {filename}")
        while True:
            data = ser.read(1)
            if data:
                f.write(data)
            else:
                print("No data received")
                break

    print("Finished writing data")
except FileNotFoundError:
    print(f"Error opening file: {filename}")

ser.close()
print("Serial port closed")
