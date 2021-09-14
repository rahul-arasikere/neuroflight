#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif


EXTERNC void infer(float *input, int input_size, float *output, int output_size);


#undef EXTERNC

