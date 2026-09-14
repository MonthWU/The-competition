#ifndef PUZZLE_PERCEPTION_NODE__HSV_OBJECT_MASK_HPP_
#define PUZZLE_PERCEPTION_NODE__HSV_OBJECT_MASK_HPP_

#include <opencv2/core.hpp>

namespace puzzle_perception_node
{

struct HsvRange
{
  int h_min{0};
  int h_max{179};
  int s_min{0};
  int s_max{255};
  int v_min{0};
  int v_max{255};
};

inline bool valid_hsv_range(const HsvRange & range)
{
  return range.h_min >= 0 && range.h_max <= 179 && range.h_min <= range.h_max &&
         range.s_min >= 0 && range.s_max <= 255 && range.s_min <= range.s_max &&
         range.v_min >= 0 && range.v_max <= 255 && range.v_min <= range.v_max;
}

inline bool hsv_ranges_overlap(const HsvRange & first, const HsvRange & second)
{
  if (!valid_hsv_range(first) || !valid_hsv_range(second)) {
    return false;
  }
  const bool hue_overlap = first.h_min <= second.h_max && second.h_min <= first.h_max;
  const bool saturation_overlap = first.s_min <= second.s_max && second.s_min <= first.s_max;
  const bool value_overlap = first.v_min <= second.v_max && second.v_min <= first.v_max;
  return hue_overlap && saturation_overlap && value_overlap;
}

inline cv::Mat make_hsv_object_mask(const cv::Mat & hsv, const HsvRange & range)
{
  if (hsv.empty() || hsv.type() != CV_8UC3 || !valid_hsv_range(range)) {
    return cv::Mat();
  }
  cv::Mat mask;
  cv::inRange(
    hsv,
    cv::Scalar(range.h_min, range.s_min, range.v_min),
    cv::Scalar(range.h_max, range.s_max, range.v_max), mask);
  return mask;
}

}  // namespace puzzle_perception_node

#endif  // PUZZLE_PERCEPTION_NODE__HSV_OBJECT_MASK_HPP_
