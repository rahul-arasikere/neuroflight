#include <platform.h>
#include "common/maths.h"
#include "graph_interface.h"
//#include "graph.h"
#include "tflite/inference.h"
#include "tflite/model_data.h"

/**
 * A wrapper for the AOT C++ neural network graph to compute 
 * motor outputs for the given state input.
 *
 * 	input: An input array of the current state
 * 	input_size: 
 **/


void run_graph(float *input, int input_size, float *output, int output_size) {
	//fc::NeuroControl controller;
	static tflite::MicroInterpreter* interpreter = init_model(model_tflite);
	static TfLiteTensor* input_ptr = interpreter->input(0);
	// //Copy the input into the buffer
	std::copy(input + 0, input + input_size, input_ptr->data.f);
	interpreter->Invoke();
	static TfLiteTensor* output_ptr = interpreter->output(0);
	std::copy(output_ptr->data.f + 0, output_ptr->data.f + output_size, output);
	// //Run the graph, the output of the neural network is in rage [-1:1] for each motor output
	// //controller.Run();
	// for (int i = 0; i < output_size; i++) {
	// 	*(output + i) = interpreter->output(0)->data.f[i];
	// }
}


