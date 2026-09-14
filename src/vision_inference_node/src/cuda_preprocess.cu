#include "vision_inference_node/cuda_preprocess.hpp"

#include <cuda_fp16.h>

#include <algorithm>
#include <cmath>

namespace vision_inference_node
{
namespace
{

__device__ unsigned char clamp_u8(const int value)
{
  return static_cast<unsigned char>(max(0, min(255, value)));
}

__device__ float3 read_yuyv_rgb(
  const unsigned char * source, const int width, const int height, int x, int y)
{
  x = max(0, min(width - 1, x));
  y = max(0, min(height - 1, y));
  const int pair_x = x & ~1;
  const int offset = (y * width + pair_x) * 2;
  const int luminance = source[offset + ((x & 1) ? 2 : 0)];
  const int u = source[offset + 1];
  const int v = source[offset + 3];
  const int c = max(0, luminance - 16);
  const int d = u - 128;
  const int e = v - 128;
  return make_float3(
    static_cast<float>(clamp_u8((298 * c + 409 * e + 128) >> 8)),
    static_cast<float>(clamp_u8((298 * c - 100 * d - 208 * e + 128) >> 8)),
    static_cast<float>(clamp_u8((298 * c + 516 * d + 128) >> 8)));
}

__device__ float3 bilinear_yuyv_rgb(
  const unsigned char * source, const int width, const int height, const float x, const float y)
{
  const int x0 = static_cast<int>(floorf(x));
  const int y0 = static_cast<int>(floorf(y));
  const int x1 = x0 + 1;
  const int y1 = y0 + 1;
  const float wx = x - static_cast<float>(x0);
  const float wy = y - static_cast<float>(y0);
  const float3 p00 = read_yuyv_rgb(source, width, height, x0, y0);
  const float3 p10 = read_yuyv_rgb(source, width, height, x1, y0);
  const float3 p01 = read_yuyv_rgb(source, width, height, x0, y1);
  const float3 p11 = read_yuyv_rgb(source, width, height, x1, y1);
  float3 result;
  result.x = (p00.x * (1.0F - wx) + p10.x * wx) * (1.0F - wy) +
    (p01.x * (1.0F - wx) + p11.x * wx) * wy;
  result.y = (p00.y * (1.0F - wx) + p10.y * wx) * (1.0F - wy) +
    (p01.y * (1.0F - wx) + p11.y * wx) * wy;
  result.z = (p00.z * (1.0F - wx) + p10.z * wx) * (1.0F - wy) +
    (p01.z * (1.0F - wx) + p11.z * wx) * wy;
  return result;
}

template<typename T>
__device__ void store_value(T * destination, const int index, const float value);

template<>
__device__ void store_value<float>(float * destination, const int index, const float value)
{
  destination[index] = value;
}

template<>
__device__ void store_value<__half>(__half * destination, const int index, const float value)
{
  destination[index] = __float2half(value);
}

template<typename T>
__global__ void yuyv_letterbox_kernel(
  const unsigned char * source,
  const int source_width,
  const int source_height,
  T * destination,
  const int destination_width,
  const int destination_height,
  const float scale,
  const float padding_x,
  const float padding_y)
{
  const int output_x = blockIdx.x * blockDim.x + threadIdx.x;
  const int output_y = blockIdx.y * blockDim.y + threadIdx.y;
  if (output_x >= destination_width || output_y >= destination_height) {
    return;
  }

  const float source_x =
    (static_cast<float>(output_x) - padding_x + 0.5F) / scale - 0.5F;
  const float source_y =
    (static_cast<float>(output_y) - padding_y + 0.5F) / scale - 0.5F;
  float3 rgb = make_float3(114.0F, 114.0F, 114.0F);
  if (source_x >= -0.5F && source_x < static_cast<float>(source_width) - 0.5F &&
    source_y >= -0.5F && source_y < static_cast<float>(source_height) - 0.5F)
  {
    rgb = bilinear_yuyv_rgb(source, source_width, source_height, source_x, source_y);
  }

  const int plane_size = destination_width * destination_height;
  const int index = output_y * destination_width + output_x;
  store_value(destination, index, rgb.x / 255.0F);
  store_value(destination, plane_size + index, rgb.y / 255.0F);
  store_value(destination, 2 * plane_size + index, rgb.z / 255.0F);
}

}  // namespace

cudaError_t launch_yuyv_letterbox(
  const unsigned char * source,
  const int source_width,
  const int source_height,
  void * destination,
  const int destination_width,
  const int destination_height,
  const NetworkInputType input_type,
  cudaStream_t stream)
{
  const float scale = std::min(
    static_cast<float>(destination_width) / static_cast<float>(source_width),
    static_cast<float>(destination_height) / static_cast<float>(source_height));
  const float resized_width = roundf(static_cast<float>(source_width) * scale);
  const float resized_height = roundf(static_cast<float>(source_height) * scale);
  const float padding_x = (static_cast<float>(destination_width) - resized_width) * 0.5F;
  const float padding_y = (static_cast<float>(destination_height) - resized_height) * 0.5F;
  const dim3 block(16, 16);
  const dim3 grid(
    (destination_width + block.x - 1) / block.x,
    (destination_height + block.y - 1) / block.y);
  if (input_type == NetworkInputType::kFloat16) {
    yuyv_letterbox_kernel<<<grid, block, 0, stream>>>(
      source, source_width, source_height, static_cast<__half *>(destination),
      destination_width, destination_height, scale, padding_x, padding_y);
  } else {
    yuyv_letterbox_kernel<<<grid, block, 0, stream>>>(
      source, source_width, source_height, static_cast<float *>(destination),
      destination_width, destination_height, scale, padding_x, padding_y);
  }
  return cudaGetLastError();
}

}  // namespace vision_inference_node
