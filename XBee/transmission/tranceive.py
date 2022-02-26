import transmitter
import receiver
import time
import serial
from threading import Thread
import xbee
import datetime
import spinup
import copy
from gym import spaces
import numpy as np
import os
from multiprocessing import Process, Queue
import tensorflow as tf

def clear_queue(q):
    while not q.empty():
        q.get()

def existing_actor_critic(*args, **kwargs):
    actor = tf.keras.models.load_model("actor"
    )
    critic = tf.keras.models.load_model("critic")
    return actor, critic 


def live_training(nn_queue, obs_queue):
    action_space = spaces.Box(-np.ones(4), np.ones(4), dtype=np.float32)
    observation_space = spaces.Box(-np.inf, np.inf, shape=(13,), dtype=np.float32)
    def on_save(actor, critic, epoch):
        save_path = os.path.join(".", f"ckpt_{epoch}")
        critic.save(os.path.join(save_path, "critic"))
        actor.save(os.path.join(save_path, "actor"))
        print("Sending new actor!")
        converted = tf.lite.TFLiteConverter.from_keras_model(actor).convert()
        with open("sent.tflite", "wb") as f:
            f.write(converted)

        nn_queue.put(converted)
    spinup.live_ddpg(
        obs_queue,
        observation_space,
        action_space,
        actor_critic=existing_actor_critic,
        on_save=on_save,
        safety_q=tf.keras.models.load_model("critic")
    )


def tranceive(ser, nn_queue, obs_queue, circular_buffer_size=2000):
    check = True
    obs_dropped_count = 0
    prev_obs = None
    while True:
        while nn_queue.empty():
            check, obs = receiver.receive_obs(ser, check, debug=False)
            if prev_obs:
                if prev_obs.iter == obs.iter + 1: # trajectories are reversed
                    obs_queue.put((copy.deepcopy(prev_obs), obs.prev_action, obs))
                    if obs_queue.qsize() > circular_buffer_size:
                        obs_queue.get()
                else:
                    obs_dropped_count += 1
                    print("obs dropped:", obs_dropped_count)
            prev_obs = obs
        NN_to_send = nn_queue.get_nowait() # nn_queue should always be non-empty
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
        clear_queue(nn_queue)
        check = True

if __name__ == '__main__':
    ser = xbee.init()
    print("Number of pending bytes on imput buffer:", ser.in_waiting)
    print("Reading the dummy byte from drone: ")
    # print(ser.read(4))
    nn_queue = Queue()
    obs_queue = Queue()
    # time.sleep(3)

    Process(target=live_training, args=(nn_queue, obs_queue)).start()
    tranceive(ser, nn_queue, obs_queue)
    #Thread(target=lambda: receiver.keep_receiving(ser)).start()