import serial
import struct
import sys
import numpy
import time
import ctypes
from crccheck.crc import Crc16Mcrf4XX

first_sync_size = 1
second_sync_size = 0
expected_num = 1
ser = serial.Serial('/dev/ttyUSB4', 115200)  # open first serial port
#ser = serial.Serial('/dev/ttyUSB5', 115200)  # open first serial port



class rpy_t (ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("roll", ctypes.c_float),     #4B
        ("pitch", ctypes.c_float),    #4B
        ("yaw", ctypes.c_float)       #4B
    ]
    
class action_t (ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("bottom_right", ctypes.c_float),  #4B
        ("top_right", ctypes.c_float),    #4B
        ("bottom_left", ctypes.c_float),  #4B
        ("top_left", ctypes.c_float)     #4B
    ]

class observation_t (ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("error",   rpy_t),            #12B
        ("ang_vel", rpy_t),            #12B 
        ("ang_acc", rpy_t),            #12B 
        ("prev_action", action_t)     #16B     
]   
    
class checked_observation_t (ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("observation", observation_t),
        ("crc", ctypes.c_uint16)
]   


checked_observation = checked_observation_t()

print("connected to: " + ser.portstr)


keep_checking = False
check         = True
avg_loop_time = 1
loop_num      = 0

ser.read(4000)

def block_crc(block):
    crc = Crc16Mcrf4XX().calc(block)
    return ctypes.c_uint16(crc)

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
        #print(ctypes.sizeof(checked_observation))
        #print(ctypes.sizeof(observation_t))
        #print(ctypes.sizeof(ctypes.c_uint16))
        data = ser.read(ctypes.sizeof(checked_observation)) #array of bytes    
        #f_data = struct.unpack("f",data)
        ctypes.memmove(ctypes.pointer(checked_observation), data, ctypes.sizeof(checked_observation))
        received_crc = ctypes.c_uint16(checked_observation.crc)
        crc = block_crc(bytes(checked_observation.observation))
        packed_crc = bytes(crc)
    #expected_crc = ser.read(second_sync_size)
    expected_crc = checked_observation.crc
    print(type(expected_crc))
    
    print("calculated_crc: ", crc.value, "   recieved_crc:", expected_crc)
    #print("calculated_crc_first_byte: ", packed_crc[0], "calculated_crc_second_byte: ", packed_crc[1] )
    
    if(expected_crc != crc.value):
        print("not match in crc")
        exit()
    else:
        #print(avg_loop_time)
        #print(checked_observation.observation.ang_vel.yaw)
        print(checked_observation.observation.ang_vel.yaw)
        print(checked_observation.observation.ang_acc.roll)
    
    
    avg_loop_time = avg_loop_time*0.9 + 0.1*(time.time()-begin)
ser.close()
