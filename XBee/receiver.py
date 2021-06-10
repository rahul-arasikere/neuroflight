import serial

ser = serial.Serial('/dev/ttyUSB5', 115200)  # open first serial port

print("connected to: " + ser.portstr)
count=1

while True:
    ser.flush()
    line = ser.readline()
    print(line)
ser.close()
