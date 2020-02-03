#include <platform.h>
#include "common/maths.h"
#include "graph_interface.h"
#include "graph.h"

/**
 * A wrapper for the AOT C++ neural network graph to compute 
 * motor outputs for the given state input.
 *
 * 	input: An input array of the current state
 * 	input_size: 
 **/


void run_graph(float *input, int input_size, float *output, int output_size) {
	fc::NeuroControl controller;
	//Copy the input into the buffer
	std::copy(input + 0, input + input_size, controller.arg0_data());

	//Run the graph, the output of the neural network is in rage [-1:1] for each motor output
	controller.Run();
	for (int i = 0; i < output_size; i++) {
		*(output + i) = controller.result0(0,i);
	}
}


