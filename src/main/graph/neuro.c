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
#include "graph/neuro.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "sensors/gyro.h"
#include "graph/graph_interface.h"
#include "graph_dim.h"
#include "common/filter.h"


/* An array containing inputs for the neural network 
 * where the first element is the oldest
 */
static float graphInput[GRAPH_INPUT_SIZE];
static float graphOutput[GRAPH_OUTPUT_SIZE];
static float controlOutput[GRAPH_OUTPUT_SIZE];
static float previousOutput[GRAPH_OUTPUT_SIZE];

static bool initFlag = true;



static FAST_RAM filterApplyFnPtr dtermNotchFilterApplyFn;
static FAST_RAM void *dtermFilterNotch[3];
static FAST_RAM filterApplyFnPtr dtermLpfApplyFn;
static FAST_RAM void *dtermFilterLpf[3];

static FAST_RAM float dT;


typedef union dtermFilterLpf_u {
    pt1Filter_t pt1Filter[3];
    biquadFilter_t biquadFilter[3];
    firFilterDenoise_t denoisingFilter[3];
} dtermFilterLpf_t;

void neuroInitFilters(const pidProfile_t *pidProfile)
{
    // BUILD_BUG_ON(FD_YAW != 2); // only setting up Dterm filters on roll and pitch axes, so ensure yaw axis is 2
    float targetLooptime = gyro.targetLooptime;
    dT = (float)gyro.targetLooptime * 0.000001f;
    if (targetPidLooptime == 0) {
        // no looptime set, so set all the filters to null
        dtermNotchFilterApplyFn = nullFilterApply;
        dtermLpfApplyFn = nullFilterApply;
        return;
    }

    const uint32_t pidFrequencyNyquist = (1.0f / dT) / 2; // No rounding needed

    uint16_t dTermNotchHz;
    if (pidProfile->dterm_notch_hz <= pidFrequencyNyquist) {
        dTermNotchHz = pidProfile->dterm_notch_hz;
    } else {
        if (pidProfile->dterm_notch_cutoff < pidFrequencyNyquist) {
            dTermNotchHz = pidFrequencyNyquist;
        } else {
            dTermNotchHz = 0;
        }
    }

    if (dTermNotchHz != 0 && pidProfile->dterm_notch_cutoff != 0) {
        static biquadFilter_t biquadFilterNotch[3];
        dtermNotchFilterApplyFn = (filterApplyFnPtr)biquadFilterApply;
        const float notchQ = filterGetNotchQ(dTermNotchHz, pidProfile->dterm_notch_cutoff);
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
            dtermFilterNotch[axis] = &biquadFilterNotch[axis];
            biquadFilterInit(dtermFilterNotch[axis], dTermNotchHz, targetPidLooptime, notchQ, FILTER_NOTCH);
        }
    } else {
        dtermNotchFilterApplyFn = nullFilterApply;
    }

    static dtermFilterLpf_t dtermFilterLpfUnion;
    if (pidProfile->dterm_lpf_hz == 0 || pidProfile->dterm_lpf_hz > pidFrequencyNyquist) {
        dtermLpfApplyFn = nullFilterApply;
    } else {
        switch (pidProfile->dterm_filter_type) {
        default:
            dtermLpfApplyFn = nullFilterApply;
            break;
        case FILTER_PT1:
            dtermLpfApplyFn = (filterApplyFnPtr)pt1FilterApply;
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                dtermFilterLpf[axis] = &dtermFilterLpfUnion.pt1Filter[axis];
                pt1FilterInit(dtermFilterLpf[axis], pidProfile->dterm_lpf_hz, dT);
            }
            break;
        case FILTER_BIQUAD:
            dtermLpfApplyFn = (filterApplyFnPtr)biquadFilterApply;
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                dtermFilterLpf[axis] = &dtermFilterLpfUnion.biquadFilter[axis];
                biquadFilterInitLPF(dtermFilterLpf[axis], pidProfile->dterm_lpf_hz, targetPidLooptime);
            }
            break;
        case FILTER_FIR:
            dtermLpfApplyFn = (filterApplyFnPtr)firFilterDenoiseUpdate;
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                dtermFilterLpf[axis] = &dtermFilterLpfUnion.denoisingFilter[axis];
                firFilterDenoiseInit(dtermFilterLpf[axis], pidProfile->dterm_lpf_hz, targetPidLooptime);
            }
            break;
        }
    }
}

void neuroInit(const pidProfile_t *pidProfile)
{
    neuroInitFilters(pidProfile);
    for (int i = 0; i < GRAPH_OUTPUT_SIZE; i++) {
        previousOutput[i] = -1; 
    }

}


void evaluateGraphWithErroeDeltaTargetDeltaStateAct(timeUs_t currentTimeUs){
    static timeUs_t previousTime;
    static float previousSetpoint[3];
    static float previousGyroFiltered[3];

    const float deltaT = ((float)(currentTimeUs - previousTime))/1000000.0f;
    if (debugMode == DEBUG_NN_DT) {
        debug[0] = (int16_t)(currentTimeUs - previousTime);
        debug[1] = 0;
        debug[2] = 0;
        debug[3] = 0;
    }
    previousTime = currentTimeUs;

    //Prepare the neural network inputs
    // Set the current error and deriviate
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {

        float currentSetpoint = getSetpointRate(axis);
        float changeInSetpoint = currentSetpoint - previousSetpoint[axis];
        float gyroRate = gyro.gyroADCf[axis];
        float gyroRateFiltered = gyroRate;
        // gyroRateFiltered = dtermLpfApplyFn(dtermFilterLpf[axis], gyroRate);
        const float gyroDelta = gyroRateFiltered - previousGyroFiltered[axis]; 
        float errorRate = currentSetpoint - gyroRate; 
        graphInput[axis] = errorRate;

        if (debugMode == DEBUG_NN_SPDELTA) {
            debug[axis] = (int16_t)(1000*changeInSetpoint);
        }

        if (debugMode == DEBUG_NN_GYDELTA) {
            debug[axis] = (int16_t)(1000*gyroDelta);
        }

        if (debugMode == DEBUG_NN_GYRATE) {
            debug[axis] = (int16_t)(10*gyroRate);
        }

        if (debugMode == DEBUG_NN_ERR_RATE) {
            debug[axis] = (int16_t)(10*errorRate);
        }

        //TODO We need to include delta time because the loop is not fixed
        // graphInput[axis + 3] = changeInSetpoint;
        graphInput[axis + 3] = gyroRate;

        // previousSetpoint[axis] = currentSetpoint;
        // previousGyroFiltered[axis] = gyroRateFiltered;
    }

    for (int i = 0; i < GRAPH_OUTPUT_SIZE; i++) {
        graphInput[i+6] = previousOutput[i];
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

    //Evaluate the neural network graph and convert to range [-1,1]->[0,1]
    run_graph(graphInput, GRAPH_INPUT_SIZE, graphOutput, GRAPH_OUTPUT_SIZE);

    for (int i = 0; i < GRAPH_OUTPUT_SIZE; i++) {
        float filtered = previousOutput[i]*0.9 + graphOutput[i]*0.1;
        controlOutput[i] = transformScale(constrainf(filtered, -1.0f, 1.0f), -1.0f, 1.0f, 0, 1);
        previousOutput[i] = filtered;
    }

    if (debugMode == DEBUG_NN_OUT) {
        for (int i = 0; i<GRAPH_OUTPUT_SIZE; i++){
            debug[i] = (int16_t)(previousOutput[i] * 1000.0);
        }
    }
}


void evaluateGraphWithErrorStateDeltaStateActRelative(timeUs_t currentTimeUs){
    static timeUs_t previousTime;
    static float previousGyroFiltered[3];

    const float deltaT = ((float)(currentTimeUs - previousTime))/1000000.0f;
    previousTime = currentTimeUs;

    //Prepare the neural network inputs
    // Set the current error and deriviate
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {

        float currentSetpoint = getSetpointRate(axis);
        float gyroRate = gyro.gyroADCf[axis];
        float gyroRateFiltered = dtermNotchFilterApplyFn(dtermFilterNotch[axis], gyro.gyroADCf[axis]);
        gyroRateFiltered = dtermLpfApplyFn(dtermFilterLpf[axis], gyroRate);
        const float gyroDelta = gyroRateFiltered - previousGyroFiltered[axis]; 
        float errorRate = currentSetpoint - gyroRate; 
        graphInput[axis] = errorRate;

        if (debugMode == DEBUG_NN_GYDELTA) {
            debug[axis] = (int16_t)(1000*gyroDelta);
        }

        if (debugMode == DEBUG_NN_GYRATE) {
            debug[axis] = (int16_t)(10*gyroRate);
        }

        if (debugMode == DEBUG_NN_ERR_RATE) {
            debug[axis] = (int16_t)(10*errorRate);
        }

        //TODO We need to include delta time because the loop is not fixed
        graphInput[axis + 3] = gyroDelta;

        previousGyroFiltered[axis] = gyroRateFiltered;
    }

    for (int i = 0; i < GRAPH_OUTPUT_SIZE; i++) {
        graphInput[i+6] = previousOutput[i];
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

    //Evaluate the neural network graph and convert to range [-1,1]->[0,1]
    run_graph(graphInput, GRAPH_INPUT_SIZE, graphOutput, GRAPH_OUTPUT_SIZE);

    for (int i = 0; i < GRAPH_OUTPUT_SIZE; i++) {
        float new_output = previousOutput[i] + graphOutput[i]*0.2;
        new_output = constrainf(new_output, -1.0f, 1.0f);
        controlOutput[i] = transformScale(new_output, -1.0f, 1.0f, 0.0f, 1.0f);
        previousOutput[i] = new_output;
    }

    if (debugMode == DEBUG_NN_OUT) {
        for (int i = 0; i<GRAPH_OUTPUT_SIZE; i++){
            debug[i] = (int16_t)(previousOutput[i] * 1000.0);
        }
    }
}

void evaluateGraphWithErrorTargetDeltaStateActRelative(timeUs_t currentTimeUs){
    static timeUs_t previousTime;
    static float previousGyroFiltered[3];

    const float deltaT = ((float)(currentTimeUs - previousTime))/1000000.0f;

    if (debugMode == DEBUG_NN_DT) {
        debug[0] = (int16_t)(currentTimeUs - previousTime);
        debug[1] = 0;
        debug[2] = 0;
        debug[3] = 0;
    }

    previousTime = currentTimeUs;

    //Prepare the neural network inputs
    // Set the current error and deriviate
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {

        float currentSetpoint = getSetpointRate(axis);
        float gyroRate = gyro.gyroADCf[axis];
        float gyroRateFiltered = dtermNotchFilterApplyFn(dtermFilterNotch[axis], gyro.gyroADCf[axis]);
        gyroRateFiltered = dtermLpfApplyFn(dtermFilterLpf[axis], gyroRate);
        const float gyroDelta = gyroRateFiltered - previousGyroFiltered[axis]; 
        float errorRate = currentSetpoint - gyroRate; 
        graphInput[axis] = errorRate;

        if (debugMode == DEBUG_NN_GYDELTA) {
            debug[axis] = (int16_t)(1000*gyroDelta);
        }

        if (debugMode == DEBUG_NN_GYRATE) {
            debug[axis] = (int16_t)(10*gyroRate);
        }

        if (debugMode == DEBUG_NN_ERR_RATE) {
            debug[axis] = (int16_t)(10*errorRate);
        }

        graphInput[axis + 3] = currentSetpoint;
        //TODO We need to include delta time because the loop is not fixed
        graphInput[axis + 6] = gyroDelta;

        previousGyroFiltered[axis] = gyroRateFiltered;
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

    //Evaluate the neural network graph and convert to range [-1,1]->[0,1]
    // run_graph(graphInput, GRAPH_INPUT_SIZE, graphOutput, GRAPH_OUTPUT_SIZE);

    for (int i = 0; i < GRAPH_OUTPUT_SIZE; i++) {
        float new_output = previousOutput[i] + graphOutput[i]*0.2;
        new_output = constrainf(new_output, -1.0f, 1.0f);
        // controlOutput[i] = transformScale(new_output, -1.0f, 1.0f, 0.0f, 1.0f);
        controlOutput[i] = 0;
        previousOutput[i] = new_output;
    }

    if (debugMode == DEBUG_NN_OUT) {
        for (int i = 0; i<GRAPH_OUTPUT_SIZE; i++){
            debug[i] = (int16_t)(previousOutput[i] * 1000.0);
        }
    }
}


void evaluateGraphWithErrorDerivateError(timeUs_t currentTimeUs){
    static timeUs_t previousTime;
    static float previousRateError[3];

    const float deltaT = ((float)(currentTimeUs - previousTime))/1000000.0f;
    if (debugMode == DEBUG_NN_DT) {
        debug[0] = (int16_t)(currentTimeUs - previousTime);
        debug[1] = 0;
        debug[2] = 0;
        debug[3] = 0;
    }
    previousTime = currentTimeUs;

    //Prepare the neural network inputs
    // Set the current error and deriviate
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        float currentSetpoint = getSetpointRate(axis);
        float gyroRate = gyro.gyroADCf[axis];
        // gyroRate = dtermLpfApplyFn(dtermFilterLpf[axis], gyroRate);
		float errorRate = currentSetpoint - gyroRate; 
		graphInput[axis] = errorRate;

        //TODO We need to include delta time because the loop is not fixed
        float delta = (errorRate - previousRateError[axis]);
        graphInput[axis + 3] = delta;

        previousRateError[axis] = errorRate;
    }
    /*  
    if (debugMode == DEBUG_NN_OUT) {
        for (int i = 0; i<GRAPH_INPUT_SIZE; i++){
            debug[i] = (int16_t)(graphInput[i] * 1000.0);
        }
    }
    */

    //Evaluate the neural network graph and convert to range [-1,1]->[0,1]
    run_graph(graphInput, GRAPH_INPUT_SIZE, graphOutput, GRAPH_OUTPUT_SIZE);
    for (int i = 0; i < GRAPH_OUTPUT_SIZE; i++) {
        controlOutput[i] = transformScale(constrainf(graphOutput[i], -1.0f, 1.0f), -1.0f, 1.0f, 0, 1); 
    }

    if (debugMode == DEBUG_NN_OUT) {
        for (int i = 0; i<GRAPH_OUTPUT_SIZE; i++){
            debug[i] = (int16_t)(controlOutput[i] * 1000.0);
        }
    }
}
void neuroController(timeUs_t currentTimeUs, const pidProfile_t *pidProfile){
    if(initFlag) {
        neuroInit(pidProfile);
        initFlag = false;
    }
    evaluateGraphWithErroeDeltaTargetDeltaStateAct(currentTimeUs);
    // evaluateGraphWithErrorTargetDeltaStateActRelative(currentTimeUs);
    // evaluateGraphWithErrorDerivateError(currentTimeUs);
	mixGraphOutput(currentTimeUs, controlOutput);
}
float transformScale(float value, float oldLow, float oldHigh, float newLow, float newHigh){
	return ((value - oldLow) / (oldHigh - oldLow)) * (newHigh - newLow) + newLow;
}
