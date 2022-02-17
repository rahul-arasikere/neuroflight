import serial
import struct
import sys
import numpy
import time
import ctypes
from crccheck.crc import Crc16Mcrf4XX
import xbee


class MyStructure(ctypes.Structure):

    def __repr__(self) -> str:
        values = ", ".join(f"{name}={value}"
            for name, value in self._asdict().items())
        return f"<{self.__class__.__name__}: {values}>"

    def _asdict(self) -> dict:
       return {field[0]: getattr(self, field[0])
            for field in self._fields_
}    



class rpy_t (MyStructure):
    _pack_ = 1
    _fields_ = [
        ("roll", ctypes.c_float),     #4B
        ("pitch", ctypes.c_float),    #4B
        ("yaw", ctypes.c_float)       #4B
    ]
    
class action_t (MyStructure):
    _pack_ = 1
    _fields_ = [
        ("bottom_right", ctypes.c_float),  #4B
        ("top_right", ctypes.c_float),    #4B
        ("bottom_left", ctypes.c_float),  #4B
        ("top_left", ctypes.c_float)     #4B
    ]

class observation_t (MyStructure):
    _pack_ = 1
    _fields_ = [
        ("error",   rpy_t),                   #12B
        ("ang_vel", rpy_t),                   #12B 
        ("ang_acc", rpy_t),                   #12B 
        ("prev_action", action_t),            #16B     
        # ("iter", ctypes.c_uint16),            #16B 
        # ("delta_micros", ctypes.c_uint16)     #16B 
]   
    
class checked_observation_t (MyStructure):
    _pack_ = 1
    _fields_ = [
        ("observation", observation_t),
        ("crc", ctypes.c_uint16)
]   



def block_crc(block):
    crc = Crc16Mcrf4XX().calc(block)
    return ctypes.c_uint16(crc)




def keep_receiving(
        ser,
        keep_checking=False,
        check=True,
        avg_loop_time=1,
        loop_num=0,
        first_sync_size = 1,
        second_sync_size=0,
        expected_num=1,
        debug=False
    ):

    checked_observation = checked_observation_t()
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
            ctypes.memmove(ctypes.pointer(checked_observation),
                data, ctypes.sizeof(checked_observation))
            received_crc = ctypes.c_uint16(checked_observation.crc)
            crc = block_crc(bytes(checked_observation.observation))
            packed_crc = bytes(crc)
        #expected_crc = ser.read(second_sync_size)
        expected_crc = checked_observation.crc
        #print(type(expected_crc))
        if debug:    
            print("calculated_crc: ", crc.value, "   recieved_crc:", expected_crc)
            #print("calculated_crc_first_byte: ", packed_crc[0], "calculated_crc_second_byte: ", packed_crc[1] )
            
            if(expected_crc != crc.value):
                print("not match in crc")
                #exit()
            else:
                #print(avg_loop_time)
                #print(checked_observation.observation.ang_vel.yaw)
                # print(checked_observation.observation.ang_vel.yaw)
                # print(checked_observation.observation.ang_acc.roll)
                print("recieved struct:")
                print(checked_observation.observation,"\n\n")
                
        avg_loop_time = avg_loop_time*0.9 + 0.1*(time.time()-begin)


def send_and_recieve_byte(
        ser,
        byte=b"A"
    ):
    print("sending: ", byte)
    ser.write(byte)
    ans = ser.read(1)
    print("recieved: ", ans)


        
if __name__ == '__main__':
    ser = xbee.init()
    
    print("Reading the dummy byte from drone:")
    dummy  = ser.read(4)
    print("Recieved byte is: ",dummy)

    #ser.read(4000)
    #send_and_recieve_byte(ser)
    keep_receiving(ser, debug=True)
    ser.close()