#include <platform.h>
#include "common/maths.h"
#include "graph_interface.h"
//#include "graph.h"
#include "tflite/inference.h"
// #include "tflite/model_data.h"
// #include "tflite/model2_data.h"



void infer(float *input, int input_size, float *output, const uint8_t* model_data, int output_size, serialPort_t * use_serialPort) {
	tflite::MicroErrorReporter micro_error_reporter;
	micro_error_reporter.SetSerial(use_serialPort);
	tflite::Model* model = ::tflite::GetModel(model_data);
	if (model->version() != TFLITE_SCHEMA_VERSION) {
		serialPrint(use_serialPort, "Model version does not match Schema");
		while(1);
	}
	tflite::MicroMutableOpResolver<10> resolver;
	resolver.AddFullyConnected();
	resolver.AddMinimum();
	resolver.AddArgMin();
	resolver.AddMaximum();
	resolver.AddArgMax();
	resolver.AddSub();
	resolver.AddMul();
	resolver.AddTanh();

	resolver.AddReshape();
	resolver.AddSoftmax();
	static const int tensor_arena_size = 30 * 1024; //limit of 100kb
	static const uint8_t tensor_arena[tensor_arena_size];

	tflite::MicroInterpreter interpreter(
		model, resolver, tensor_arena, tensor_arena_size, &micro_error_reporter);
	
	TfLiteStatus allocate_status = interpreter.AllocateTensors();
	if (allocate_status != kTfLiteOk) {
		serialPrint(use_serialPort, "AllocateTensors() failed");
		while(1);
	}
	// interpreter = init_model(flip ? model_tflite : model2_tflite);
	TfLiteTensor* input_ptr = interpreter.input(0);
	// // //Copy the input into the buffer
	std::copy(input + 0, input + input_size, input_ptr->data.f);
	TfLiteStatus invoke_status = interpreter.Invoke();
	if (invoke_status != kTfLiteOk) {
		serialPrint(use_serialPort, "Invoke failed on input");
	}
	//The output of the neural network is in rage [-1:1] for each motor output
	TfLiteTensor* output_ptr = interpreter.output(0);
	std::copy(output_ptr->data.f + 0, output_ptr->data.f + output_size, output);
}