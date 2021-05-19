#pragma once
#include "tflite/tensorflow/lite/micro/micro_interpreter.h"

tflite::MicroInterpreter* init_model(const void *buf);
