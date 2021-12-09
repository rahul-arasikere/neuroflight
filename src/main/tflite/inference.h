#pragma once
#include "tflite/tensorflow/lite/micro/micro_interpreter.h"
#include "tflite/tensorflow/lite/micro/all_ops_resolver.h"
#include "tflite/tensorflow/lite/micro/micro_error_reporter.h"
#include "tflite/tensorflow/lite/micro/micro_utils.h"
#include "tflite/tensorflow/lite/schema/schema_generated.h"

tflite::MicroInterpreter* init_model(const void *buf, bool update);
