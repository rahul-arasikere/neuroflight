import transmitter
import receiver
import time
import serial
from threading import Thread
import xbee
import datetime
 
transmitting = False
NN_to_send = transmitter.model_tflite_short1

# def transmit_thread(ser):
#     transmitter.transmit(ser, transmitter.model_tflite_short1)
#     time.sleep(5)
#     transmitter.transmit(ser, transmitter.model_tflite_short2)

transmission_count = 0;

def live_training():
    global transmitting, NN_to_send, transmission_count
    while True:
        time.sleep(7)
        if not transmitting:
            transmission_count+=1
            print("Beginning transmission #", transmission_count, "[", datetime.datetime.now(),"]")
            NN_to_send = transmitter.model_tflite_should_work
            transmitting = True

def tranceive_thread(ser):
    global transmitting, NN_to_send
    check = True

    while True:
        while not transmitting:
            check = receiver.receive_obs(ser, check)
        xbee.empty_read_buffer(ser)
        transmitter.transmit(ser, NN_to_send)
        xbee.empty_read_buffer(ser)
        print("DEBUG: bytes on input buffer: ", ser.in_waiting)
        print("DEBUG: sending b")
        ser.write(b'b') # Telling drone to send metadata of the NN
        print("DEBUG: Sent the 'b' command; waiting for metadata")
    
        transmitter.receive_metadata(ser)
        time.sleep(.1)
        xbee.empty_read_buffer(ser) #EXPECT BUFFER TO BE EMPTY
        
        print("DEBUG: bytes on input buffer: ", ser.in_waiting)
        print("DEBUG: sending c")
        ser.write(b'c') # Telling drone to re-start sending obs data by convention, drone waits for 'C'
        print("DEBUG: Sent the 'C' command; waiting for OBSdata")

        transmitting = False
        check = True

if __name__ == '__main__':
    ser = xbee.init()
    print("Number of pending bytes on imput buffer:", ser.in_waiting)
    print("Reading the dummy byte from drone: ")
    print(ser.read(4))

    # time.sleep(3)

    Thread(target=live_training).start()
    Thread(target=lambda: tranceive_thread(ser)).start()
    #Thread(target=lambda: receiver.keep_receiving(ser)).start()