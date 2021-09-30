import serial
import sys

ser = serial.Serial('/dev/ttyUSB4', 115200)  # open first serial port

print("connected to: " + ser.portstr)
count=1

while True:
    ser.flush()
    line = ser.read()
    print(ord(line))
    #sys.stdout.write(line)
    #sys.stdout.flush()
    #print(line, end='')
ser.close()
