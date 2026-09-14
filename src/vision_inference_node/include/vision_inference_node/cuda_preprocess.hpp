#ifndef VISION_INFERENCE_NODE__CUDA_PREPROCESS_HPP_
#define VISION_INFERENCE_NODE__CUDA_PREPROCESS_HPP_

#include <cuda_runtime_api.h>

namespace vision_inference_node
{

enum class NetworkInputType
{
  kFloat32,
  kFloat16,
};

cudaError_t launch_yuyv_letterbox(
  const unsigned char * source,
  int source_width,
  int source_height,
  void * destination,
  int destination_width,
  int destination_height,
  NetworkInputType input_type,
  cudaStream_t stream);

}  // namespace vision_inference_node

#endif  // VISION_INFERENCE_NODE__CUDA_PREPROCESS_HPP_
