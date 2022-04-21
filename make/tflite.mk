TF_DIR=tflite_micro/tensorflow/lite

TFLITE_SRCS = \
	$(TF_DIR)/micro/simple_memory_allocator.cc \
	$(TF_DIR)/micro/memory_helpers.cc \
	$(TF_DIR)/micro/test_helpers.cc \
	$(TF_DIR)/micro/micro_context.cc \
	$(TF_DIR)/micro/micro_graph.cc \
	$(TF_DIR)/micro/micro_utils.cc \
	$(TF_DIR)/micro/micro_resource_variable.cc \
	$(TF_DIR)/micro/flatbuffer_utils.cc \
	$(TF_DIR)/micro/recording_micro_allocator.cc \
	$(TF_DIR)/micro/micro_error_reporter.cc \
	$(TF_DIR)/micro/cortex_m_generic/micro_time.cc \
	$(TF_DIR)/micro/recording_simple_memory_allocator.cc \
	$(TF_DIR)/micro/micro_string.cc \
	$(TF_DIR)/micro/micro_profiler.cc \
	$(TF_DIR)/micro/cortex_m_generic/debug_log.cc \
	$(TF_DIR)/micro/system_setup.cc \
	$(TF_DIR)/micro/all_ops_resolver.cc \
	$(TF_DIR)/micro/micro_interpreter.cc \
	$(TF_DIR)/micro/micro_allocator.cc \
	$(TF_DIR)/micro/micro_allocation_info.cc \
	$(TF_DIR)/micro/memory_planner/linear_memory_planner.cc \
	$(TF_DIR)/micro/memory_planner/greedy_memory_planner.cc \
	$(TF_DIR)/c/common.cc \
	$(TF_DIR)/core/api/error_reporter.cc \
	$(TF_DIR)/core/api/flatbuffer_conversions.cc \
	$(TF_DIR)/core/api/op_resolver.cc \
	$(TF_DIR)/core/api/tensor_utils.cc \
	$(TF_DIR)/kernels/internal/quantization_util.cc \
	$(TF_DIR)/kernels/kernel_util.cc \
	$(TF_DIR)/schema/schema_utils.cc \
	$(TF_DIR)/micro/kernels/activations.cc \
	$(TF_DIR)/micro/kernels/add.cc \
	$(TF_DIR)/micro/kernels/add_common.cc \
	$(TF_DIR)/micro/kernels/add_n.cc \
	$(TF_DIR)/micro/kernels/arg_min_max.cc \
	$(TF_DIR)/micro/kernels/batch_to_space_nd.cc \
	$(TF_DIR)/micro/kernels/cast.cc \
	$(TF_DIR)/micro/kernels/ceil.cc \
	$(TF_DIR)/micro/kernels/circular_buffer.cc \
	$(TF_DIR)/micro/kernels/comparisons.cc \
	$(TF_DIR)/micro/kernels/concatenation.cc \
	$(TF_DIR)/micro/kernels/conv.cc \
	$(TF_DIR)/micro/kernels/conv_common.cc \
	$(TF_DIR)/micro/kernels/cumsum.cc \
	$(TF_DIR)/micro/kernels/depth_to_space.cc \
	$(TF_DIR)/micro/kernels/depthwise_conv.cc \
	$(TF_DIR)/micro/kernels/depthwise_conv_common.cc \
	$(TF_DIR)/micro/kernels/dequantize.cc \
	$(TF_DIR)/micro/kernels/detection_postprocess.cc \
	$(TF_DIR)/micro/kernels/elementwise.cc \
	$(TF_DIR)/micro/kernels/elu.cc \
	$(TF_DIR)/micro/kernels/ethosu.cc \
	$(TF_DIR)/micro/kernels/exp.cc \
	$(TF_DIR)/micro/kernels/expand_dims.cc \
	$(TF_DIR)/micro/kernels/fill.cc \
	$(TF_DIR)/micro/kernels/floor.cc \
	$(TF_DIR)/micro/kernels/floor_div.cc \
	$(TF_DIR)/micro/kernels/floor_mod.cc \
	$(TF_DIR)/micro/kernels/fully_connected.cc \
	$(TF_DIR)/micro/kernels/fully_connected_common.cc \
	$(TF_DIR)/micro/kernels/hard_swish.cc \
	$(TF_DIR)/micro/kernels/kernel_runner.cc \
	$(TF_DIR)/micro/kernels/kernel_util.cc \
	$(TF_DIR)/micro/kernels/l2norm.cc \
	$(TF_DIR)/micro/kernels/l2_pool_2d.cc \
	$(TF_DIR)/micro/kernels/leaky_relu.cc \
	$(TF_DIR)/micro/kernels/logical.cc \
	$(TF_DIR)/micro/kernels/logistic.cc \
	$(TF_DIR)/micro/kernels/log_softmax.cc \
	$(TF_DIR)/micro/kernels/maximum_minimum.cc \
	$(TF_DIR)/micro/kernels/mul.cc \
	$(TF_DIR)/micro/kernels/mul_common.cc \
	$(TF_DIR)/micro/kernels/neg.cc \
	$(TF_DIR)/micro/kernels/pack.cc \
	$(TF_DIR)/micro/kernels/pad.cc \
	$(TF_DIR)/micro/kernels/pooling.cc \
	$(TF_DIR)/micro/kernels/prelu.cc \
	$(TF_DIR)/micro/kernels/quantize.cc \
	$(TF_DIR)/micro/kernels/quantize_common.cc \
	$(TF_DIR)/micro/kernels/reduce.cc \
	$(TF_DIR)/micro/kernels/reshape.cc \
	$(TF_DIR)/micro/kernels/resize_bilinear.cc \
	$(TF_DIR)/micro/kernels/resize_nearest_neighbor.cc \
	$(TF_DIR)/micro/kernels/round.cc \
	$(TF_DIR)/micro/kernels/shape.cc \
	$(TF_DIR)/micro/kernels/softmax.cc \
	$(TF_DIR)/micro/kernels/softmax_common.cc \
	$(TF_DIR)/micro/kernels/space_to_batch_nd.cc \
	$(TF_DIR)/micro/kernels/split.cc \
	$(TF_DIR)/micro/kernels/split_v.cc \
	$(TF_DIR)/micro/kernels/squeeze.cc \
	$(TF_DIR)/micro/kernels/strided_slice.cc \
	$(TF_DIR)/micro/kernels/sub.cc \
	$(TF_DIR)/micro/kernels/sub_common.cc \
	$(TF_DIR)/micro/kernels/svdf.cc \
	$(TF_DIR)/micro/kernels/svdf_common.cc \
	$(TF_DIR)/micro/kernels/tanh.cc \
	$(TF_DIR)/micro/kernels/transpose_conv.cc \
	$(TF_DIR)/micro/kernels/unpack.cc \
	$(TF_DIR)/micro/kernels/zeros_like.cc

# NEUROFLIGHT_SRCS = neuroflight/trajectory_buffer.c neuroflight/graph_interface.cc neuroflight/neuro.c
NEUROFLIGHT_SRCS := \
					tflite/large_test.c \
					$(wildcard $(SRC_DIR)/neuroflight/*.c) \
					$(wildcard $(SRC_DIR)/neuroflight/*.cc)

RUY_DIR = ruy/ruy/profiler

RUY_SRC = \
	$(RUY_DIR)/instrumentation.cc \
	$(RUY_DIR)/profiler.cc \
	$(RUY_DIR)/treeview.cc 

SRC += $(TFLITE_SRCS) $(NEUROFLIGHT_SRCS) $(RUY_SRC)

