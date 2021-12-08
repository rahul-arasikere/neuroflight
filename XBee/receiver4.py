import serial
import struct
import sys
import numpy
import time
import ctypes


first_sync_size = 1
second_sync_size = 0
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
    
class checked_observation_t (ctypes.Structure):
    _fields_ = [

        ("observation", observation_t),
        ("crc", ctypes.c_uint16)
]   

    
def block_crc(byte_array):
    crcAccum = ctypes.c_uint16(0xffff)
    for byte in map(ctypes.c_char, byte_array):
        tmp = byte ^ ctypes.c_char(bytearray(crcAccum)[0])
        print(type(tmp))
        tmp ^= (tmp << 4)
        crcAccum = (crcAccum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
        print("size of crc: ", ctypes.sizeof(crcAccum))
    return crcAccum

checked_observation =   checked_observation_t()

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
        data = ser.read(ctypes.sizeof(checked_observation)) #array of bytes    
        #f_data = struct.unpack("f",data)
        ctypes.memmove(ctypes.pointer(checked_observation), data, ctypes.sizeof(checked_observation))
        crc = block_crc(data)
        packed_crc = crc.to_bytes(4, 'big')
    #expected_crc = ser.read(second_sync_size)
    expected_crc = checked_observation.crc
    
    print("calculated_crc: ", crc, "   recieved_crc:", expected_crc)
    print("calculated_crc_first_byte: ", packed_crc[0], "calculated_crc_second_byte: ", packed_crc[1] )
    
    if(expected_crc == crc):
        print(avg_loop_time)
        print(checked_observation.observation.ang_vel.yaw)
        #print(observation.rpy_t.pitch)
        #print(observation.rpy_t.yaw)
    
    
    avg_loop_time = avg_loop_time*0.9 + 0.1*(time.time()-begin)
ser.close()
