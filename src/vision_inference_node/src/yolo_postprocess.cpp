#include "vision_inference_node/yolo_postprocess.hpp"

#include <cuda_fp16.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace vision_inference_node
{
namespace
{

float intersection_over_union(const Detection & left, const Detection & right)
{
  const float x1 = std::max(left.x_min, right.x_min);
  const float y1 = std::max(left.y_min, right.y_min);
  const float x2 = std::min(left.x_max, right.x_max);
  const float y2 = std::min(left.y_max, right.y_max);
  const float intersection = std::max(0.0F, x2 - x1) * std::max(0.0F, y2 - y1);
  const float left_area = std::max(0.0F, left.x_max - left.x_min) *
    std::max(0.0F, left.y_max - left.y_min);
  const float right_area = std::max(0.0F, right.x_max - right.x_min) *
    std::max(0.0F, right.y_max - right.y_min);
  const float denominator = left_area + right_area - intersection;
  return denominator > std::numeric_limits<float>::epsilon() ? intersection / denominator : 0.0F;
}

}  // namespace

TensorView::TensorView(
  const void * data, const std::size_t elements, const TensorValueType type)
: data_(data), elements_(elements), type_(type)
{
  if (data_ == nullptr && elements_ != 0U) {
    throw std::invalid_argument("BUG_POINT:OUTPUT_DECODE tensor data is null");
  }
}

float TensorView::at(const std::size_t index) const
{
  if (index >= elements_) {
    throw std::out_of_range("BUG_POINT:OUTPUT_DECODE tensor index exceeds output size");
  }
  if (type_ == TensorValueType::kFloat32) {
    return static_cast<const float *>(data_)[index];
  }
  return __half2float(static_cast<const __half *>(data_)[index]);
}

std::size_t TensorView::size() const
{
  return elements_;
}

Detection restore_letterbox_box(
  const float x_min, const float y_min, const float x_max, const float y_max,
  const float confidence, const int class_id, const int source_width,
  const int source_height, const int network_width, const int network_height)
{
  if (source_width <= 0 || source_height <= 0 || network_width <= 0 || network_height <= 0) {
    throw std::invalid_argument("BUG_POINT:OUTPUT_DECODE image dimensions must be positive");
  }
  const float scale = std::min(
    static_cast<float>(network_width) / static_cast<float>(source_width),
    static_cast<float>(network_height) / static_cast<float>(source_height));
  const float resized_width = std::round(static_cast<float>(source_width) * scale);
  const float resized_height = std::round(static_cast<float>(source_height) * scale);
  const float padding_x = (static_cast<float>(network_width) - resized_width) * 0.5F;
  const float padding_y = (static_cast<float>(network_height) - resized_height) * 0.5F;
  Detection detection;
  detection.class_id = class_id;
  detection.confidence = confidence;
  detection.x_min = std::clamp(
    (x_min - padding_x) / scale, 0.0F, static_cast<float>(source_width));
  detection.y_min = std::clamp(
    (y_min - padding_y) / scale, 0.0F, static_cast<float>(source_height));
  detection.x_max = std::clamp(
    (x_max - padding_x) / scale, 0.0F, static_cast<float>(source_width));
  detection.y_max = std::clamp(
    (y_max - padding_y) / scale, 0.0F, static_cast<float>(source_height));
  return detection;
}

std::vector<Detection> class_aware_nms(
  std::vector<Detection> detections, const float threshold, const int maximum)
{
  if (maximum <= 0) {
    return {};
  }
  std::sort(detections.begin(), detections.end(), [](const auto & left, const auto & right) {
    return left.confidence > right.confidence;
  });
  std::vector<Detection> selected;
  selected.reserve(std::min<std::size_t>(detections.size(), static_cast<std::size_t>(maximum)));
  for (const auto & candidate : detections) {
    const bool suppressed = std::any_of(selected.begin(), selected.end(), [&](const auto & kept) {
      return candidate.class_id == kept.class_id &&
             intersection_over_union(candidate, kept) > threshold;
    });
    if (!suppressed) {
      selected.push_back(candidate);
      if (static_cast<int>(selected.size()) >= maximum) {
        break;
      }
    }
  }
  return selected;
}

std::vector<Detection> decode_yolo_output(
  const TensorView & output,
  const std::vector<int> & dimensions,
  const int source_width,
  const int source_height,
  const int network_width,
  const int network_height,
  const float confidence_threshold,
  const float iou_threshold,
  const int max_detections)
{
  if (dimensions.size() < 2U) {
    throw std::runtime_error("BUG_POINT:OUTPUT_DECODE output rank is too small");
  }
  const int last = dimensions.back();
  std::vector<Detection> detections;
  if (last == 6 && output.size() % 6U == 0U) {
    const std::size_t rows = output.size() / 6U;
    detections.reserve(rows);
    for (std::size_t row = 0; row < rows; ++row) {
      const std::size_t base = row * 6U;
      const float confidence = output.at(base + 4U);
      if (!std::isfinite(confidence) || confidence < confidence_threshold) {
        continue;
      }
      const auto detection = restore_letterbox_box(
        output.at(base), output.at(base + 1U), output.at(base + 2U), output.at(base + 3U),
        confidence, static_cast<int>(std::lround(output.at(base + 5U))),
        source_width, source_height, network_width, network_height);
      if (detection.x_max > detection.x_min && detection.y_max > detection.y_min) {
        detections.push_back(detection);
      }
    }
    return class_aware_nms(std::move(detections), iou_threshold, max_detections);
  }

  if (dimensions.size() != 3U) {
    throw std::runtime_error(
            "BUG_POINT:OUTPUT_DECODE expected [1,N,6] or rank-3 raw YOLO output");
  }
  const int first_axis = dimensions[1];
  const int second_axis = dimensions[2];
  const bool channel_first = first_axis >= 5 && first_axis <= 512 && second_axis > first_axis;
  const int channels = channel_first ? first_axis : second_axis;
  const int anchors = channel_first ? second_axis : first_axis;
  if (channels < 5) {
    throw std::runtime_error("BUG_POINT:OUTPUT_DECODE raw YOLO output has fewer than 5 channels");
  }
  const auto expected_elements = static_cast<std::size_t>(channels) *
    static_cast<std::size_t>(anchors);
  if (expected_elements > output.size()) {
    throw std::runtime_error("BUG_POINT:OUTPUT_DECODE tensor shape exceeds output buffer");
  }
  const int class_count = channels - 4;
  detections.reserve(static_cast<std::size_t>(std::min(anchors, max_detections * 4)));
  const auto value = [&](const int anchor, const int channel) {
      const std::size_t index = channel_first ?
        static_cast<std::size_t>(channel * anchors + anchor) :
        static_cast<std::size_t>(anchor * channels + channel);
      return output.at(index);
    };
  for (int anchor = 0; anchor < anchors; ++anchor) {
    int best_class = 0;
    float best_score = value(anchor, 4);
    for (int class_id = 1; class_id < class_count; ++class_id) {
      const float score = value(anchor, 4 + class_id);
      if (score > best_score) {
        best_score = score;
        best_class = class_id;
      }
    }
    if (!std::isfinite(best_score) || best_score < confidence_threshold) {
      continue;
    }
    const float center_x = value(anchor, 0);
    const float center_y = value(anchor, 1);
    const float width = value(anchor, 2);
    const float height = value(anchor, 3);
    const auto detection = restore_letterbox_box(
      center_x - width * 0.5F, center_y - height * 0.5F,
      center_x + width * 0.5F, center_y + height * 0.5F,
      best_score, best_class, source_width, source_height, network_width, network_height);
    if (detection.x_max > detection.x_min && detection.y_max > detection.y_min) {
      detections.push_back(detection);
    }
  }
  return class_aware_nms(std::move(detections), iou_threshold, max_detections);
}

}  // namespace vision_inference_node
