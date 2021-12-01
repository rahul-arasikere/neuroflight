#ifndef TRAJECTORY_BUFFER_H
#define TRAJECTORY_BUFFER_H

typedef struct rpy_t {
    float roll;
    float pitch;
    float yaw;
} __attribute__((packed)) rpy_t;

typedef struct action_t {
    float bottom_right;
    float top_right;
    float bottom_left;
    float top_left;
} __attribute__((packed)) action_t;

typedef struct observation_t {
    rpy_t error;
    rpy_t ang_vel;
    rpy_t ang_acc;
    action_t prev_action;
} __attribute__((packed)) observation_t;

typedef enum TRAJ_BUFFER_STATE_t {
    PRODUCING,
    TRANSMITTING
} TRAJ_BUFFER_STATE_t;


void add_to_traj(observation_t obs);
observation_t consume_from_traj();
void write_float(float x);
void write_observation(observation_t s);
void traj_transmission_handler(observation_t curr_state);

#endif