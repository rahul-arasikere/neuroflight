#include "trajectory_buffer.h"
#include "io/uart4Serial.h"


#define TRAJ_SIZE 100

observation_t trajectory[TRAJ_SIZE];
uint16_t traj_size = 0;

#define US_PER_BYTE (4000/14)

const uint32_t US_PER_TRANS = (sizeof(observation_t) + 2) * US_PER_BYTE;

TRAJ_BUFFER_STATE_t traj_buffer_state = PRODUCING;

void add_to_traj(observation_t obs) {
    trajectory[traj_size] = obs;
    traj_size++;
    if(traj_size >= TRAJ_SIZE) {
        traj_buffer_state = TRANSMITTING;
    }
}

observation_t consume_from_traj() {
    traj_size--;
    if(traj_size <= 0) {
        traj_buffer_state = PRODUCING;
    }
    return trajectory[traj_size];
}

void write_float(float x) {
    unsigned char bytes_array[sizeof(float)];
    *((float *)bytes_array) = x;
    for(unsigned int i = 0; i < sizeof(bytes_array); i++)
        serialWrite(uart4Serial, bytes_array[i]);
}

void write_observation(observation_t obs) {
    serialWrite(uart4Serial, 228);
    const unsigned char *buffer = (unsigned char*)&obs;
    for (uint16_t i = 0; i < sizeof(observation_t); i++) {
        serialWrite(uart4Serial, buffer[i]);
    }   
    serialWrite(uart4Serial, 229);
}


void traj_transmission_handler(observation_t curr_state) {
    switch(traj_buffer_state) {
        case PRODUCING:
            add_to_traj(curr_state);
            break;
        case TRANSMITTING:
            {
                static uint32_t last_send_time = 0;
                uint32_t current_time = micros();
                if((current_time - last_send_time) > US_PER_TRANS) {
                    write_observation(consume_from_traj());
                    last_send_time = current_time;
                }

            }
            break;
    };
}