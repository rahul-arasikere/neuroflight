#include <platform.h>
#include "common/maths.h"
#include "graph_interface.h"
#include "tflite/inference.h"
#include "io/uart4Serial.h"



void infer(float *input, int input_size, float *output, const uint8_t* model_data, int output_size) {
	tflite::MicroErrorReporter micro_error_reporter;
	tflite::Model* model = ::tflite::GetModel(model_data);
	if (model->version() != TFLITE_SCHEMA_VERSION) {
		serialPrint(getUART4(), "Model version does not match Schema");
		while(1);
	}
	tflite::MicroMutableOpResolver<11> resolver;
	resolver.AddFullyConnected();
	resolver.AddMinimum();
	resolver.AddArgMin();
	resolver.AddMaximum();
	resolver.AddArgMax();
	resolver.AddSub();
	resolver.AddMul();
	resolver.AddAdd();
	resolver.AddTanh();

	resolver.AddReshape();
	resolver.AddSoftmax();
	static const int tensor_arena_size = 30 * 1024; //limit of 100kb
	static const uint8_t tensor_arena[tensor_arena_size];

	tflite::MicroInterpreter interpreter(
		model, resolver, tensor_arena, tensor_arena_size, &micro_error_reporter
	);
	
	TfLiteStatus allocate_status = interpreter.AllocateTensors();
	if (allocate_status != kTfLiteOk) {
		serialPrint(getUART4(), "AllocateTensors() failed");
		while(1);
	}
	TfLiteTensor* input_ptr = interpreter.input(0);
	// // // //Copy the input into the buffer
	std::copy(input + 0, input + input_size, input_ptr->data.f);
	TfLiteStatus invoke_status = interpreter.Invoke();
	if (invoke_status != kTfLiteOk) {
		serialPrint(getUART4(), "Invoke failed on input");
		while(1);
	}
	// //The output of the neural network is in rage [-1:1] for each motor output
	TfLiteTensor* output_ptr = interpreter.output(0);
	std::copy(output_ptr->data.f + 0, output_ptr->data.f + output_size, output);
}