import serial
import struct
import sys
import numpy
import time

first_sync_size = 1
second_sync_size = 1
expected_num = 100
ser = serial.Serial('/dev/ttyUSB4', 115200)  # open first serial port
#ser = serial.Serial('/dev/ttyUSB5', 115200)  # open first serial port

gyro_roll  = []
gyro_pitch = []



print("connected to: " + ser.portstr)
sync_counter=0

keep_checking=True
check=True
avg_loop_time = 1
loop_num = 0
while True:
    loop_num += 1
    begin = time.time()
    if check:
        while True:
            sync_byte = ser.read(1)
            if ord(sync_byte)==165:
                check = keep_checking
                break
    else:
        ser.read()
    
    #sync_counter = 0
        

    gyro_yaw   = []

    for i in range(expected_num):
        #ser.flush()
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
    avg_loop_time = avg_loop_time*0.9 + 0.1*(time.time()-begin)
    #if(loop_num == 1000):
    print(avg_loop_time)
    #    break
    print(gyro_yaw)
    
    #sys.stdout.write(line)
    #sys.stdout.flush()
    #print(line, end='')
    
ser.close()
