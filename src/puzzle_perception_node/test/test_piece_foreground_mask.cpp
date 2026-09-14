#include "puzzle_perception_node/piece_foreground_mask.hpp"

#include <cstdint>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace
{
cv::Mat to_bgr(const cv::Mat & hsv)
{
  cv::Mat bgr;
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
  return bgr;
}

bool selected(const cv::Mat & mask, const int x, const int y)
{
  return mask.at<std::uint8_t>(y, x) != 0U;
}
}  // namespace

int main()
{
  const puzzle_perception_node::HsvRange white{0, 179, 0, 77, 160, 255};
  const puzzle_perception_node::HsvRange magnet{100, 140, 100, 255, 60, 255};

  cv::Mat hsv(80, 100, CV_8UC3, cv::Scalar(75, 150, 180));
  cv::rectangle(hsv, cv::Rect(20, 15, 20, 20), cv::Scalar(0, 20, 230), cv::FILLED);
  cv::rectangle(hsv, cv::Rect(0, 0, 12, 12), cv::Scalar(0, 20, 230), cv::FILLED);
  cv::rectangle(hsv, cv::Rect(50, 35, 12, 12), cv::Scalar(0, 20, 230), cv::FILLED);
  const puzzle_perception_node::PieceForegroundMaskParams actual_params{
    white, true, magnet, 5, 40, 1, 1, 0};
  const cv::Mat actual_mask = puzzle_perception_node::make_piece_foreground_mask(
    to_bgr(hsv), actual_params);
  if (actual_mask.empty()) {
    std::cerr << "actual white-piece mask was not generated\n";
    return 1;
  }
  if (!selected(actual_mask, 25, 20)) {
    std::cerr << "central white piece was not retained\n";
    return 1;
  }
  if (selected(actual_mask, 70, 20)) {
    std::cerr << "green paper was incorrectly selected as a white piece\n";
    return 1;
  }
  if (selected(actual_mask, 1, 1)) {
    std::cerr << "page-margin exclusion was not applied\n";
    return 1;
  }
  if (selected(actual_mask, 55, 40)) {
    std::cerr << "divider exclusion was not applied\n";
    return 1;
  }

  auto no_divider_params = actual_params;
  no_divider_params.page_margin_px = 0;
  no_divider_params.divider_half_height_px = -1;
  const cv::Mat no_divider_mask = puzzle_perception_node::make_piece_foreground_mask(
    to_bgr(hsv), no_divider_params);
  if (!selected(no_divider_mask, 1, 1)) {
    std::cerr << "disabled divider unexpectedly cleared a foreground row\n";
    return 1;
  }

  cv::Mat exclusion_hsv(20, 20, CV_8UC3, cv::Scalar(0, 20, 230));
  exclusion_hsv.at<cv::Vec3b>(10, 10) = cv::Vec3b(120, 180, 200);
  const puzzle_perception_node::PieceForegroundMaskParams exclusion_params{
    puzzle_perception_node::HsvRange{0, 179, 0, 255, 100, 255},
    true, magnet, 0, 100, 0, 1, 0};
  const cv::Mat exclusion_mask = puzzle_perception_node::make_piece_foreground_mask(
    to_bgr(exclusion_hsv), exclusion_params);
  if (!selected(exclusion_mask, 5, 5) || selected(exclusion_mask, 10, 10)) {
    std::cerr << "configured exclusion colour was not removed from the foreground\n";
    return 1;
  }

  auto invalid_params = actual_params;
  invalid_params.morph_kernel = 0;
  if (!puzzle_perception_node::make_piece_foreground_mask(
      to_bgr(hsv), invalid_params).empty())
  {
    std::cerr << "invalid foreground-mask parameters did not fail closed\n";
    return 1;
  }

  cv::Mat canny_scene(100, 120, CV_8UC3, cv::Scalar(40, 150, 70));
  cv::rectangle(canny_scene, cv::Rect(30, 20, 36, 28), cv::Scalar(235, 235, 235), cv::FILLED);
  cv::line(canny_scene, cv::Point(0, 50), cv::Point(119, 50), cv::Scalar(10, 10, 10), 1);
  puzzle_perception_node::CannyPieceForegroundMaskParams canny_params;
  canny_params.page_margin_px = 6;
  canny_params.divider_center_y_px = 50;
  canny_params.divider_half_height_px = 1;
  canny_params.low_threshold = 30.0;
  canny_params.high_threshold = 90.0;
  canny_params.blur_kernel = 3;
  canny_params.aperture_size = 3;
  canny_params.edge_dilate_iterations = 1;
  canny_params.close_kernel = 5;
  canny_params.close_iterations = 2;
  const cv::Mat canny_mask = puzzle_perception_node::make_canny_piece_foreground_mask(
    canny_scene, canny_params);
  if (canny_mask.empty()) {
    std::cerr << "canny foreground mask was not generated\n";
    return 1;
  }
  if (!selected(canny_mask, 45, 32)) {
    std::cerr << "closed Canny piece contour was not filled\n";
    return 1;
  }
  if (selected(canny_mask, 10, 10)) {
    std::cerr << "Canny selected background as a piece\n";
    return 1;
  }
  if (selected(canny_mask, 60, 50)) {
    std::cerr << "Canny divider exclusion was not applied\n";
    return 1;
  }

  cv::Mat close_scene(80, 120, CV_8UC3, cv::Scalar(40, 150, 70));
  cv::rectangle(close_scene, cv::Rect(22, 20, 25, 26), cv::Scalar(235, 235, 235), cv::FILLED);
  cv::rectangle(close_scene, cv::Rect(51, 20, 25, 26), cv::Scalar(235, 235, 235), cv::FILLED);
  auto close_params = canny_params;
  close_params.page_margin_px = 4;
  close_params.divider_half_height_px = -1;
  close_params.edge_dilate_iterations = 0;
  close_params.close_iterations = 0;
  const cv::Mat close_mask = puzzle_perception_node::make_canny_piece_foreground_mask(
    close_scene, close_params);
  if (!selected(close_mask, 34, 33) || !selected(close_mask, 63, 33)) {
    std::cerr << "nearby Canny pieces were not retained\n";
    return 1;
  }
  if (selected(close_mask, 49, 33)) {
    std::cerr << "nearby Canny pieces were bridged across the background gap\n";
    return 1;
  }
  std::vector<std::vector<cv::Point>> close_contours;
  cv::findContours(close_mask.clone(), close_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  int large_close_components = 0;
  for (const auto & contour : close_contours) {
    if (std::abs(cv::contourArea(contour)) > 100.0) {
      ++large_close_components;
    }
  }
  if (large_close_components != 2) {
    std::cerr << "nearby Canny pieces did not remain as two external contours\n";
    return 1;
  }

  auto invalid_canny_params = canny_params;
  invalid_canny_params.high_threshold = invalid_canny_params.low_threshold;
  if (!puzzle_perception_node::make_canny_piece_foreground_mask(
      canny_scene, invalid_canny_params).empty())
  {
    std::cerr << "invalid Canny parameters did not fail closed\n";
    return 1;
  }
  return 0;
}
