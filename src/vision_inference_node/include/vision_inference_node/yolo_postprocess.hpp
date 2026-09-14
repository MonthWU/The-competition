#ifndef VISION_INFERENCE_NODE__YOLO_POSTPROCESS_HPP_
#define VISION_INFERENCE_NODE__YOLO_POSTPROCESS_HPP_

#include <cstddef>
#include <vector>

#include "vision_inference_node/detection.hpp"

namespace vision_inference_node
{

enum class TensorValueType
{
  kFloat32,
  kFloat16
};

class TensorView
{
public:
  TensorView(const void * data, std::size_t elements, TensorValueType type);

  float at(std::size_t index) const;
  std::size_t size() const;

private:
  const void * data_;
  std::size_t elements_;
  TensorValueType type_;
};

Detection restore_letterbox_box(
  float x_min, float y_min, float x_max, float y_max,
  float confidence, int class_id, int source_width, int source_height,
  int network_width, int network_height);

std::vector<Detection> class_aware_nms(
  std::vector<Detection> detections, float threshold, int maximum);

std::vector<Detection> decode_yolo_output(
  const TensorView & output,
  const std::vector<int> & dimensions,
  int source_width,
  int source_height,
  int network_width,
  int network_height,
  float confidence_threshold,
  float iou_threshold,
  int max_detections);

}  // namespace vision_inference_node

#endif  // VISION_INFERENCE_NODE__YOLO_POSTPROCESS_HPP_
