#ifndef VISION_INFERENCE_NODE__DETECTION_HPP_
#define VISION_INFERENCE_NODE__DETECTION_HPP_

namespace vision_inference_node
{

struct Detection
{
  int class_id{0};
  float confidence{0.0F};
  float x_min{0.0F};
  float y_min{0.0F};
  float x_max{0.0F};
  float y_max{0.0F};
};

}  // namespace vision_inference_node

#endif  // VISION_INFERENCE_NODE__DETECTION_HPP_
