#ifndef VISION_INFERENCE_NODE__TENSORRT_DETECTOR_HPP_
#define VISION_INFERENCE_NODE__TENSORRT_DETECTOR_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "vision_inference_node/detection.hpp"
#include "vision_inference_node/inference_metrics.hpp"

namespace vision_inference_node
{

class TensorRtDetector
{
public:
  TensorRtDetector(
    const std::string & engine_path,
    int input_size,
    float confidence_threshold,
    float iou_threshold,
    int max_detections);
  ~TensorRtDetector();

  TensorRtDetector(const TensorRtDetector &) = delete;
  TensorRtDetector & operator=(const TensorRtDetector &) = delete;

  std::vector<Detection> infer(const cv::Mat & yuyv_frame, InferenceMetrics & metrics);
  int input_width() const;
  int input_height() const;
  std::string binding_summary() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace vision_inference_node

#endif  // VISION_INFERENCE_NODE__TENSORRT_DETECTOR_HPP_
