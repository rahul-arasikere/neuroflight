// #ifndef GRAPH_INTERFACE_H
// #define GRAPH_INTERFACE_H
#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

#include "io/serial.h"

EXTERNC void infer(float *input, int input_size, float *output, const uint8_t* model_data, int output_size, serialPort_t *use_serialPort);


#undef EXTERNC

// #endif