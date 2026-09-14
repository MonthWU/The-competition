#ifndef PUZZLE_PERCEPTION_NODE__LAB_BACKGROUND_MASK_HPP_
#define PUZZLE_PERCEPTION_NODE__LAB_BACKGROUND_MASK_HPP_

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace puzzle_perception_node
{

struct LabBackgroundMaskParams
{
  double distance_threshold{24.0};
  double max_background_mad{10.0};
  int sample_stride{4};
  int minimum_samples{200};
  int morph_kernel{3};
  int morph_iterations{1};
  // Ignore L when illumination gradients are expected but the paper chroma is stable.
  bool include_lightness{true};
};

struct LabBackgroundMaskResult
{
  bool valid{false};
  std::string status{"BACKGROUND_SAMPLE_INVALID"};
  cv::Mat foreground_mask;
  cv::Vec3d background_lab{0.0, 0.0, 0.0};
  double background_mad{0.0};
  std::size_t sample_count{0};
};

inline double median_value(std::vector<double> values)
{
  if (values.empty()) {
    return 0.0;
  }
  const std::size_t middle = values.size() / 2U;
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle), values.end());
  double median = values[middle];
  if (values.size() % 2U == 0U) {
    const auto lower = std::max_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle));
    median = 0.5 * (median + *lower);
  }
  return median;
}

inline LabBackgroundMaskResult make_lab_background_foreground_mask(
  const cv::Mat & bgr, const cv::Mat & sampling_mask,
  const LabBackgroundMaskParams & params)
{
  LabBackgroundMaskResult result;
  if (bgr.empty() || bgr.type() != CV_8UC3 || sampling_mask.empty() ||
    sampling_mask.type() != CV_8UC1 || sampling_mask.size() != bgr.size() ||
    params.distance_threshold <= 0.0 || params.max_background_mad < 0.0 ||
    params.sample_stride <= 0 || params.minimum_samples <= 0 ||
    params.morph_kernel <= 0 || params.morph_iterations < 0)
  {
    return result;
  }

  cv::Mat lab;
  cv::cvtColor(bgr, lab, cv::COLOR_BGR2Lab);
  std::array<std::vector<double>, 3> channels;
  for (int y = 0; y < lab.rows; y += params.sample_stride) {
    for (int x = 0; x < lab.cols; x += params.sample_stride) {
      if (sampling_mask.at<std::uint8_t>(y, x) == 0U) {
        continue;
      }
      const cv::Vec3b value = lab.at<cv::Vec3b>(y, x);
      for (int channel = 0; channel < 3; ++channel) {
        channels[static_cast<std::size_t>(channel)].push_back(value[channel]);
      }
    }
  }
  result.sample_count = channels[0].size();
  if (result.sample_count < static_cast<std::size_t>(params.minimum_samples)) {
    result.status = "BACKGROUND_SAMPLE_TOO_SMALL";
    return result;
  }
  for (int channel = 0; channel < 3; ++channel) {
    result.background_lab[channel] = median_value(channels[static_cast<std::size_t>(channel)]);
  }

  std::vector<double> distances;
  distances.reserve(result.sample_count);
  const int first_distance_channel = params.include_lightness ? 0 : 1;
  for (std::size_t index = 0; index < result.sample_count; ++index) {
    double squared = 0.0;
    for (int channel = first_distance_channel; channel < 3; ++channel) {
      const double delta = channels[static_cast<std::size_t>(channel)][index] -
        result.background_lab[channel];
      squared += delta * delta;
    }
    distances.push_back(std::sqrt(squared));
  }
  result.background_mad = median_value(distances);
  if (result.background_mad > params.max_background_mad) {
    result.status = "BACKGROUND_SAMPLE_UNSTABLE";
    return result;
  }

  result.foreground_mask = cv::Mat::zeros(bgr.size(), CV_8UC1);
  for (int y = 0; y < lab.rows; ++y) {
    for (int x = 0; x < lab.cols; ++x) {
      if (sampling_mask.at<std::uint8_t>(y, x) == 0U) {
        continue;
      }
      const cv::Vec3b value = lab.at<cv::Vec3b>(y, x);
      double squared = 0.0;
      for (int channel = first_distance_channel; channel < 3; ++channel) {
        const double delta = value[channel] - result.background_lab[channel];
        squared += delta * delta;
      }
      if (std::sqrt(squared) >= params.distance_threshold) {
        result.foreground_mask.at<std::uint8_t>(y, x) = 255U;
      }
    }
  }
  if (params.morph_iterations > 0) {
    const int kernel_size = std::max(1, params.morph_kernel | 1);
    const cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
    cv::morphologyEx(
      result.foreground_mask, result.foreground_mask, cv::MORPH_OPEN, kernel,
      cv::Point(-1, -1), params.morph_iterations);
    cv::morphologyEx(
      result.foreground_mask, result.foreground_mask, cv::MORPH_CLOSE, kernel,
      cv::Point(-1, -1), params.morph_iterations);
  }
  result.valid = true;
  result.status = "OK";
  return result;
}

}  // namespace puzzle_perception_node

#endif  // PUZZLE_PERCEPTION_NODE__LAB_BACKGROUND_MASK_HPP_
