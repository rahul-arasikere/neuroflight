#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <platform.h>
#include "build/build_config.h"
#include "build/debug.h"
#include "common/axis.h"
#include "common/maths.h"
#include "drivers/time.h"
#include "fc/fc_core.h"
#include "fc/fc_rc.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "neuroflight/neuro.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "sensors/gyro.h"
#include "neuroflight/graph_interface.h"
#include "graph_dim.h"
#include "common/filter.h"
#include "io/serial.h"
#include "io/uart4Serial.h"
#include "trajectory_buffer.h"
#include "crc.h"
#include "byte_utils.h"
// #include "common/printf.h"
// #include "tflite/model_data.h"
// #include "tflite/smooth_frozen2.h"
// #include "tflite/model_test_data.h"
#include "tflite/large_test.h"
// #include "tflite/model64x64_2_motor.h"
// #include "tflite/small_test.h"
// #include "tflite/small_test_no_mul_add.h"

/* An array containing inputs for the neural network 
 * where the first element is the oldest
 */
static float graphInput[GRAPH_INPUT_SIZE];
static float graphOutput[GRAPH_OUTPUT_SIZE];
static float controlOutput[GRAPH_OUTPUT_SIZE];
static float previousOutput[GRAPH_OUTPUT_SIZE];

static bool initFlag = true;


void neuroInit(const pidProfile_t *pidProfile)
{
	for (int i = 0; i < GRAPH_OUTPUT_SIZE; i++) {
		previousOutput[i] = -1; 
	}
}

void evaluateGraphWithErrorStateDeltaStateAct(timeUs_t currentTimeUs){
	static timeUs_t previousTime;
	static float previousState[3];
	const float deltaT = ((float)(currentTimeUs - previousTime))/1000000.0f;

	//Prepare the neural network inputs
	// Set the current error and deriviate
	for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {

		float currentSetpoint = getSetpointRate(axis);
		float state = gyro.gyroADCf[axis];
		const float deltaState = state - previousState[axis]; 
		float error = currentSetpoint - state; 
		graphInput[axis] = error;

		if (debugMode == DEBUG_NN_GYDELTA) {
			debug[axis] = (int16_t)(1000*deltaState);
		}
		
		if (debugMode == DEBUG_NN_SP) {
			debug[axis] = (int16_t)(1000*currentSetpoint);
		}

		if (debugMode == DEBUG_NN_GYRATE) {
			debug[axis] = (int16_t)(10*state);
		}

		if (debugMode == DEBUG_NN_ERR_RATE) {
			debug[axis] = (int16_t)(10*error);
		}

		graphInput[axis + 3] = state;
		//TODO We need to include delta time because the loop is not fixed
		graphInput[axis + 6] = deltaState;

		previousState[axis] = state;
	}

	for (int i = 0; i < GRAPH_OUTPUT_SIZE; i++) {
		graphInput[i+9] = previousOutput[i];
	}
	
	// if (debugMode == DEBUG_NN_OUT) {
	//     for (int i = 0; i<GRAPH_INPUT_SIZE; i++){
	//         debug[i] = (int16_t)(graphInput[i] * 1000.0);
	//     }
	// }

	if (debugMode == DEBUG_NN_ACT_IN) {
		for (int i = 0; i<GRAPH_OUTPUT_SIZE; i++){
			debug[i] = (int16_t)(previousOutput[i] * 1000.0);
		}
	}

	rpy_t error;
	error.roll = 0.1;
	error.pitch = 0.2;
	error.yaw = 0.3;

	rpy_t ang_vel;
	ang_vel.roll = gyro.gyroADCf[0];
	ang_vel.pitch = gyro.gyroADCf[1];
	ang_vel.yaw = gyro.gyroADCf[2];

	rpy_t ang_acc;
	ang_acc.roll = 0.4;
	ang_acc.pitch = 0.5;
	ang_acc.yaw = 0.6;
	action_t prev_action;
	prev_action.top_left = 0;
	prev_action.top_right = 0;
	prev_action.bottom_left = 0;
	prev_action.bottom_right = 0;

	observation_t obs = {
		.error = error,
		.ang_vel = ang_vel,
		.ang_acc = ang_acc,
		.prev_action = prev_action,
		// .delta_micros = currentTimeUs - previousTime,
		// .delta_micros = infer_time,
		// .iter = 0
	};

	// traj_transmission_handler(obs);

	//Evaluate the neural network graph and convert to range [-1,1]->[0,1]
	infer(graphInput, GRAPH_INPUT_SIZE, graphOutput, memory_trick(), GRAPH_OUTPUT_SIZE);

	for (int i = 0; i < GRAPH_OUTPUT_SIZE; i++) {
		float new_output = graphOutput[i];
		new_output = constrainf(new_output, -1.0f, 1.0f);
		controlOutput[i] = transformScale(new_output, -1.0f, 1.0f, 0.0f, 1.0f);
		previousOutput[i] = new_output;
	}

	if (debugMode == DEBUG_NN_OUT) {
		for (int i = 0; i<GRAPH_OUTPUT_SIZE; i++){
			debug[i] = (int16_t)(previousOutput[i] * 1000.0);
		}
	}
	previousTime = currentTimeUs;
}

#define MAX_BUFFER_SIZE 30000

#define buffer_size_t uint16_t
#define NUM_SIZE_BYTES (sizeof(buffer_size_t))
#define NUM_META_BYTES (NUM_SIZE_BYTES + NUM_CRC_BYTES)

uint8_t buffer[NUM_META_BYTES + MAX_BUFFER_SIZE];
buffer_size_t buffer_size = 0;

buffer_size_t expected_block_size() {
	if(buffer_size < NUM_SIZE_BYTES)
		return 0;
	buffer_size_t num_bytes = 0;
	for(int i=0; i < NUM_SIZE_BYTES; i++)
		num_bytes += ((buffer_size_t)buffer[i]) << (i*8);
	return num_bytes;
}

crc_t expected_crc() {
	buffer_size_t crc = 0;
	for(int i=0; i < NUM_CRC_BYTES; i++)
		crc += ((buffer_size_t)buffer[NUM_SIZE_BYTES + i]) << (i*8);
	return crc;
}

buffer_size_t block_size() {
	return (buffer_size > NUM_META_BYTES) ? (buffer_size - NUM_META_BYTES) : 0;
}

void add_to_buffer(uint8_t add_me) {
	buffer[buffer_size] = add_me;
	buffer_size++;
}

uint8_t block_at(buffer_size_t i) {
	return buffer[i+NUM_META_BYTES];
}

uint8_t* block_ptr() {
	return buffer + NUM_META_BYTES;
} 

void print_block() {
	for(int i = 0; i < block_size(); i++) {
		serialWrite(getUART4(), block_at(i));
	}
	serialWrite(getUART4(), '\n');
}

void update_nn() {
	for(int i = 0; i < block_size(); i++) {
	    memory_trick()[i] = block_at(i);
	}
}

crc_t block_crc() {
	return compute_crc(block_ptr(), block_size());
}

void neuroController(timeUs_t currentTimeUs, const pidProfile_t *pidProfile){
	if(initFlag) {
		neuroInit(pidProfile);
		initFlag = false;
		serialWrite(getUART4(), 'a');
	} else {
		// serialWrite(getUART4(), 'a');
		uint8_t bytesWaiting;
		while ((bytesWaiting = serialRxBytesWaiting(getUART4()))) {
			uint8_t b = serialRead(getUART4());
			serialWrite(getUART4(), b);
			add_to_buffer(b);
			if((buffer_size >= NUM_META_BYTES) && (block_size() == expected_block_size())) {
				// print_block();
				// serialWrite(getUART4(), 'a');
				// serialWrite(getUART4(), 'a');
				// serialWrite(getUART4(), 'a');
				// serialWrite(getUART4(), 'a');
				// write_little_endian(block_size());
				// write_little_endian(block_crc());
				if(block_crc() == expected_crc())
					update_nn();
				buffer_size = 0;
			}
		};

		evaluateGraphWithErrorStateDeltaStateAct(currentTimeUs);
		mixGraphOutput(currentTimeUs, controlOutput);
	}
}

float transformScale(float value, float oldLow, float oldHigh, float newLow, float newHigh){
	return ((value - oldLow) / (oldHigh - oldLow)) * (newHigh - newLow) + newLow;
}
