#include "trajectory_buffer.h"
#include <stdint.h>
#include "io/serial.h"
#include "io/uart4Serial.h"
#include "crc.h"

void add_to_traj(observation_t obs);
observation_t consume_from_traj();
void write_float(float x);
void write_checked_observation(checked_observation_t obs);
checked_observation_t with_crc(observation_t obs);

#define TRAJ_SIZE 100
observation_t trajectory[TRAJ_SIZE];
uint16_t traj_size = 0;

#define US_PER_BYTE (4000/14)

#define START_BYTE ((char)228)

#define NUM_TRANS_BYTES (sizeof(START_BYTE) + sizeof(observation_t) + sizeof(crc_t))

const uint32_t US_PER_TRANS = NUM_TRANS_BYTES * US_PER_BYTE;

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
        serialWrite(getUART4(), bytes_array[i]);
}


void write_checked_observation(checked_observation_t obs) {
    serialWrite(getUART4(), START_BYTE);
    const unsigned char *buffer = (unsigned char*)&obs;
    size_t buffer_size = sizeof(obs);
    for (uint16_t i = 0; i < buffer_size; i++) {
        serialWrite(getUART4(), buffer[i]);
    }
}

checked_observation_t with_crc(observation_t obs) {
    crc_t crc = compute_crc((unsigned char*)&obs, sizeof(obs));
    checked_observation_t checked_observation;
    checked_observation.observation = obs;
    checked_observation.crc = crc;
    return checked_observation;
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
                    observation_t obs;
                    write_checked_observation(with_crc(consume_from_traj()));
                    last_send_time = current_time;
                }
            }
            break;
    };
}