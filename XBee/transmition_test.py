import time
import serial
import struct
ser = serial.Serial('/dev/ttyUSB4', 115200)  # open first serial port

while True:
  ser.write(166);
  ser.write(166);
  ser.write(166);
  ser.write(166);
  for i in range(10):
    f_data = struct.pack("f",i/20)
    ser.write(f_data[0]);
    ser.write(f_data[1]);
    ser.write(f_data[2]);
    ser.write(f_data[3]);
  ser.write(166);
  ser.write(166);
  ser.write(166);
  ser.write(166);

  ser.flush()
  time.sleep(0.001);


time.sleep(0.1)

ser.close()             # close port
