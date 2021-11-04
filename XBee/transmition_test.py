import time
import serial
import struct
import time
ser = serial.Serial('/dev/ttyUSB5', 115200)  # open first serial port
#ser = serial.Serial('/dev/ttyUSB4', 115200)  # open first serial port
expected_num = 10
delay_ms=0.1

dt = 1
while True:
  begin = time.time()
  ser.write(chr(166))
  # ser.write(chr(166));
  # ser.write(chr(166));
  # ser.write(chr(166));
  for i in range(expected_num):
    f_data = struct.pack("f",float(i)/10)
    ser.write(f_data[0])
    ser.write(f_data[1])
    ser.write(f_data[2])
    ser.write(f_data[3])
  ser.write(chr(167))
  # ser.write(chr(167));
  # ser.write(chr(167));
  # ser.write(chr(167));
  
  #ser.flush()
  #print(dt)
  dt = time.time()-begin
  time.sleep( max(delay_ms-dt,0))
  #time.sleep( delay_ms)
  

ser.close()             # close port
