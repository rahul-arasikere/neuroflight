#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif


EXTERNC void run_graph(float *input, int input_size, float *output, int output_size);


#undef EXTERNC

