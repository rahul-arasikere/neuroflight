#include "tflite/inference.h"
#include "tflite/tensorflow/lite/micro/all_ops_resolver.h"
#include "tflite/tensorflow/lite/micro/micro_error_reporter.h"
#include "tflite/tensorflow/lite/micro/micro_utils.h"
#include "tflite/tensorflow/lite/schema/schema_generated.h"

tflite::MicroInterpreter* init_model(const void *buf) {

	static tflite::MicroErrorReporter micro_error_reporter;
	static const int tensor_arena_size = 20 * 1024; //limit of 100kb
	static uint8_t tensor_arena[tensor_arena_size];
	const tflite::Model* model = ::tflite::GetModel(buf);
	static tflite::MicroMutableOpResolver<3> resolver;
	resolver.AddFullyConnected();
	resolver.AddReshape();
	resolver.AddSoftmax();

	tflite::MicroInterpreter interpreter(
		model, resolver, tensor_arena, tensor_arena_size, &micro_error_reporter);
	TfLiteStatus allocate_status = interpreter.AllocateTensors();
	return &interpreter;
}
