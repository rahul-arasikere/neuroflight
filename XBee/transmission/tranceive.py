import transmitter
import receiver
import time
import serial
from threading import Thread
import xbee
 
transmitting = False
NN_to_send = transmitter.model_tflite_short1

# def transmit_thread(ser):
#     transmitter.transmit(ser, transmitter.model_tflite_short1)
#     time.sleep(5)
#     transmitter.transmit(ser, transmitter.model_tflite_short2)


if __name__ == '__main__':
    ser = xbee.init()
    print("Reading the dummy byte from drone:")
    dummy  = ser.read(4)
    print("Recieved byte is: ",dummy)
    time.sleep(3)

    while True:
        while not transmitting:
            receiver.receive_obs()

        transmitter.transmit(ser, NN_to_send)
        receive_metadata()
        transmitting = False

    # ser.read(4000)
    # Thread(target=lambda: transmit_thread(ser)).start()
    # Thread(target=lambda: receiver.keep_receiving(ser, keep_checking=True, debug=True)).start()
