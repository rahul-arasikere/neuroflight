import serial
import struct
import sys
import numpy
import time
import ctypes


first_sync_size = 1
second_sync_size = 1
expected_num = 1
ser = serial.Serial('/dev/ttyUSB4', 115200)  # open first serial port
#ser = serial.Serial('/dev/ttyUSB5', 115200)  # open first serial port

class rpy_t (ctypes.Structure):
    _fields_ = [
        ("roll", ctypes.c_float),     #4B
        ("pitch", ctypes.c_float),    #4B
        ("yaw", ctypes.c_float)       #4B
    ]
    
class prev_action_t (ctypes.Structure):
    _fields_ = [
        ("top_left", ctypes.c_float),     #4B
        ("top_right", ctypes.c_float),    #4B
        ("bottom_left", ctypes.c_float),  #4B
        ("bottom_right", ctypes.c_float)  #4B
    ]

class observation_t (ctypes.Structure):
    _fields_ = [
        ("error",   rpy_t),            #12B
        ("ang_vel", rpy_t),            #12B 
        ("ang_acc", rpy_t),            #12B 
        ("ang_acc", prev_action_t)     #16B     
]   


observation =   observation_t()

print("connected to: " + ser.portstr)


keep_checking = False
check         = True
avg_loop_time = 1
loop_num      = 0

ser.read(4000)

while True:
    loop_num += 1
    begin = time.time()
    if check:
        while True:
            sync_byte = ser.read(1)
            if ord(sync_byte)==228:
                check = keep_checking
                break
    else:
        ser.read()

    for i in range(expected_num):
        data = ser.read(ctypes.sizeof(observation))    
        #f_data = struct.unpack("f",data)
        #observation = struct.unpack(observation.format ,data)  #Double check   
        ctypes.memmove(ctypes.pointer(observation), data, ctypes.sizeof(observation))

    second_sync = ser.read(second_sync_size)
    if(ord(second_sync) == 229):
        print(avg_loop_time)
        print(observation.ang_vel.yaw)
        #print(observation.rpy_t.pitch)
        #print(observation.rpy_t.yaw)
    
    
    avg_loop_time = avg_loop_time*0.9 + 0.1*(time.time()-begin)
ser.close()
