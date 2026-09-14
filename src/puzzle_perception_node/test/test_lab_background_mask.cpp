#include "puzzle_perception_node/lab_background_mask.hpp"

#include <cstdint>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

int main()
{
  cv::Mat frame(240, 320, CV_8UC3, cv::Scalar(8, 8, 8));
  const cv::Rect a4_rect(70, 10, 180, 220);
  cv::rectangle(frame, a4_rect, cv::Scalar(105, 185, 105), cv::FILLED);
  cv::rectangle(frame, cv::Rect(105, 45, 70, 75), cv::Scalar(235, 235, 235), cv::FILLED);
  cv::circle(frame, cv::Point(140, 70), 12, cv::Scalar(0, 0, 180), cv::FILLED);
  cv::rectangle(frame, cv::Rect(132, 88, 18, 22), cv::Scalar(10, 10, 10), cv::FILLED);

  cv::Mat sampling_mask(frame.size(), CV_8UC1, cv::Scalar(0));
  cv::rectangle(sampling_mask, a4_rect, cv::Scalar(255), cv::FILLED);
  puzzle_perception_node::LabBackgroundMaskParams params;
  params.distance_threshold = 20.0;
  params.max_background_mad = 5.0;
  params.sample_stride = 2;
  params.minimum_samples = 100;
  params.morph_iterations = 0;
  const auto result = puzzle_perception_node::make_lab_background_foreground_mask(
    frame, sampling_mask, params);
  if (!result.valid) {
    std::cerr << "Lab background sampling failed: " << result.status << '\n';
    return 1;
  }
  if (result.foreground_mask.at<std::uint8_t>(50, 110) == 0U ||
    result.foreground_mask.at<std::uint8_t>(115, 170) == 0U)
  {
    std::cerr << "card outer boundary was not retained\n";
    return 1;
  }
  if (result.foreground_mask.at<std::uint8_t>(20, 80) != 0U) {
    std::cerr << "sampled light-green A4 background became foreground\n";
    return 1;
  }
  if (result.foreground_mask.at<std::uint8_t>(5, 5) != 0U) {
    std::cerr << "area outside the A4 sampling mask became foreground\n";
    return 1;
  }

  cv::Mat reflective_lab(180, 240, CV_8UC3);
  for (int y = 0; y < reflective_lab.rows; ++y) {
    const std::uint8_t lightness = static_cast<std::uint8_t>(110 + 100 * y / reflective_lab.rows);
    reflective_lab.row(y).setTo(cv::Scalar(lightness, 90, 153));
  }
  cv::rectangle(
    reflective_lab, cv::Rect(30, 20, 70, 55), cv::Scalar(70, 128, 128), cv::FILLED);
  cv::rectangle(
    reflective_lab, cv::Rect(130, 35, 75, 60), cv::Scalar(245, 128, 128), cv::FILLED);
  cv::Mat reflective_bgr;
  cv::cvtColor(reflective_lab, reflective_bgr, cv::COLOR_Lab2BGR);
  cv::Mat reflective_sampling(
    reflective_bgr.size(), CV_8UC1, cv::Scalar(255));
  params.distance_threshold = 12.0;
  params.max_background_mad = 5.0;
  params.sample_stride = 2;
  params.morph_iterations = 0;
  params.include_lightness = false;
  const auto reflective_result = puzzle_perception_node::make_lab_background_foreground_mask(
    reflective_bgr, reflective_sampling, params);
  if (!reflective_result.valid ||
    reflective_result.foreground_mask.at<std::uint8_t>(25, 35) == 0U ||
    reflective_result.foreground_mask.at<std::uint8_t>(40, 150) == 0U ||
    reflective_result.foreground_mask.at<std::uint8_t>(150, 115) != 0U)
  {
    std::cerr << "chroma-only background inversion did not retain reflective pieces\n";
    return 1;
  }

  cv::Mat unstable = frame.clone();
  for (int y = 0; y < unstable.rows; ++y) {
    for (int x = 0; x < unstable.cols; ++x) {
      if (sampling_mask.at<std::uint8_t>(y, x) != 0U) {
        unstable.at<cv::Vec3b>(y, x) = ((x + y) % 2 == 0) ?
          cv::Vec3b(0, 0, 0) : cv::Vec3b(180, 180, 180);
      }
    }
  }
  params.max_background_mad = 1.0;
  params.sample_stride = 1;
  params.include_lightness = true;
  const auto unstable_result = puzzle_perception_node::make_lab_background_foreground_mask(
    unstable, sampling_mask, params);
  if (unstable_result.valid || unstable_result.status != "BACKGROUND_SAMPLE_UNSTABLE") {
    std::cerr << "unstable background did not fail closed\n";
    return 1;
  }
  return 0;
}
