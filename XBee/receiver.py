import serial
import struct
import sys
import numpy

expected_num = 10
ser = serial.Serial('/dev/ttyUSB4', 115200)  # open first serial port
gyro_roll  = []
gyro_pitch = []



print("connected to: " + ser.portstr)
sync_counter=0



while True:
    while sync_counter != 4:
        #print("sync counter : ", sync_counter)
        sync_byte = ser.read(1)
        if ord(sync_byte)==166:
            sync_counter += 1
        else:
            sync_counter = 0
    sync_counter = 0
    gyro_yaw   = []

    for i in range(expected_num):
        ser.flush()
        data = ser.read(4)    
        # print(ord(data[0]),ord(data[1]),ord(data[2]),ord(data[3]))
        # if (ord(data[0])==225 and ord(data[1])==225 and ord(data[2])==225 and ord(data[3])==225 ):
        f_data = struct.unpack("f",data)
        #print(f_data[0])
        gyro_yaw.append(f_data[0])

    second_sync = ser.read(4)
    if(ord(second_sync[0])==167 and ord(second_sync[1])==167 and ord(second_sync[2])==167 and ord(second_sync[3])==167):
        print(gyro_yaw)
    else:
        gyro_yaw=[]

    #sys.stdout.write(line)
    #sys.stdout.flush()
    #print(line, end='')
    
ser.close()
