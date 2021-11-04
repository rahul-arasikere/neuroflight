import serial
import struct
import sys
import numpy

first_sync_size = 1
second_sync_size = 1
expected_num = 2
ser = serial.Serial('/dev/ttyUSB5', 115200)  # open first serial port
#==ser = serial.Serial('/dev/ttyUSB5', 230400)  # open first serial port

gyro_roll  = []
gyro_pitch = []



print("connected to: " + ser.portstr)
sync_counter=0

keep_checking=True
check=True

while True:
    if check:
        while True:
            sync_byte = ser.read(1)
            if ord(sync_byte)==166:
                check = keep_checking
                break
    else:
        ser.read()
    
    #sync_counter = 0
        

    gyro_yaw   = []

    for i in range(expected_num):
        ser.flush()
        data = ser.read(4)    
        # print(ord(data[0]),ord(data[1]),ord(data[2]),ord(data[3]))
        # if (ord(data[0])==225 and ord(data[1])==225 and ord(data[2])==225 and ord(data[3])==225 ):
        f_data = struct.unpack("f",data)
        #print(f_data[0])
        gyro_yaw.append(f_data[0])

    second_sync = ser.read(second_sync_size)
    # if(ord(second_sync[0])==167 and ord(second_sync[1])==167 and ord(second_sync[2])==167 and ord(second_sync[3])==167):
    #     print(gyro_yaw)
    # else:
    #     gyro_yaw=[]

    print(gyro_yaw)
    
    #sys.stdout.write(line)
    #sys.stdout.flush()
    #print(line, end='')
    
ser.close()
