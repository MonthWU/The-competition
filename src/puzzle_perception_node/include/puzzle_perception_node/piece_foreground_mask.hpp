#ifndef PUZZLE_PERCEPTION_NODE__PIECE_FOREGROUND_MASK_HPP_
#define PUZZLE_PERCEPTION_NODE__PIECE_FOREGROUND_MASK_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "puzzle_perception_node/hsv_object_mask.hpp"

namespace puzzle_perception_node
{

struct PieceForegroundMaskParams
{
  HsvRange piece_hsv;
  bool remove_exclusion_color{false};
  HsvRange exclusion_hsv;
  int page_margin_px{0};
  int divider_center_y_px{0};
  // Negative disables the divider band; zero clears exactly the centre row.
  int divider_half_height_px{-1};
  int morph_kernel{1};
  int morph_iterations{0};
};

struct CannyPieceForegroundMaskParams
{
  int page_margin_px{0};
  int divider_center_y_px{0};
  // Negative disables the divider band; zero clears exactly the centre row.
  int divider_half_height_px{-1};
  double low_threshold{50.0};
  double high_threshold{150.0};
  int blur_kernel{5};
  int aperture_size{3};
  bool l2_gradient{false};
  int edge_dilate_iterations{0};
  int close_kernel{3};
  int close_iterations{0};
  bool flood_fill_background{true};
};

inline void clear_piece_exclusion_regions(
  cv::Mat & mask, const int page_margin_px, const int divider_center_y_px,
  const int divider_half_height_px)
{
  const int margin = std::max(0, page_margin_px);
  if (margin > 0 && margin * 2 < mask.cols && margin * 2 < mask.rows) {
    mask.rowRange(0, margin).setTo(0);
    mask.rowRange(mask.rows - margin, mask.rows).setTo(0);
    mask.colRange(0, margin).setTo(0);
    mask.colRange(mask.cols - margin, mask.cols).setTo(0);
  }

  if (divider_half_height_px >= 0) {
    const int divider_start = std::clamp(
      divider_center_y_px - divider_half_height_px, 0, mask.rows);
    const int divider_end = std::clamp(
      divider_center_y_px + divider_half_height_px + 1, 0, mask.rows);
    if (divider_start < divider_end) {
      mask.rowRange(divider_start, divider_end).setTo(0);
    }
  }
}

inline bool valid_piece_foreground_mask_params(const PieceForegroundMaskParams & params)
{
  return valid_hsv_range(params.piece_hsv) &&
         (!params.remove_exclusion_color || valid_hsv_range(params.exclusion_hsv)) &&
         params.divider_half_height_px >= -1 && params.morph_kernel > 0 &&
         params.morph_iterations >= 0;
}

inline bool valid_canny_piece_foreground_mask_params(
  const CannyPieceForegroundMaskParams & params)
{
  return params.divider_half_height_px >= -1 && params.low_threshold > 0.0 &&
         params.high_threshold > params.low_threshold && params.blur_kernel > 0 &&
         params.aperture_size >= 3 && params.aperture_size <= 7 &&
         (params.aperture_size % 2) == 1 && params.edge_dilate_iterations >= 0 &&
         params.close_kernel > 0 && params.close_iterations >= 0;
}

inline cv::Mat make_piece_foreground_mask(
  const cv::Mat & bgr, const PieceForegroundMaskParams & params)
{
  if (bgr.empty() || bgr.type() != CV_8UC3 || !valid_piece_foreground_mask_params(params)) {
    return cv::Mat();
  }

  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
  cv::Mat foreground = make_hsv_object_mask(hsv, params.piece_hsv);
  if (foreground.empty()) {
    return cv::Mat();
  }

  if (params.remove_exclusion_color) {
    const cv::Mat exclusion = make_hsv_object_mask(hsv, params.exclusion_hsv);
    if (exclusion.empty()) {
      return cv::Mat();
    }
    foreground.setTo(0, exclusion);
  }

  clear_piece_exclusion_regions(
    foreground, params.page_margin_px, params.divider_center_y_px,
    params.divider_half_height_px);

  if (params.morph_iterations > 0) {
    const int kernel_size = std::max(1, params.morph_kernel | 1);
    const cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
    cv::morphologyEx(
      foreground, foreground, cv::MORPH_OPEN, kernel, cv::Point(-1, -1),
      params.morph_iterations);
    cv::morphologyEx(
      foreground, foreground, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1),
      params.morph_iterations);
  }
  return foreground;
}

inline cv::Mat make_canny_piece_foreground_mask(
  const cv::Mat & bgr, const cv::Mat & valid_mask,
  const CannyPieceForegroundMaskParams & params)
{
  if (bgr.empty() || bgr.type() != CV_8UC3 ||
    !valid_canny_piece_foreground_mask_params(params))
  {
    return cv::Mat();
  }
  if (!valid_mask.empty() &&
    (valid_mask.type() != CV_8UC1 || valid_mask.size() != bgr.size()))
  {
    return cv::Mat();
  }

  cv::Mat active_mask;
  if (valid_mask.empty()) {
    active_mask = cv::Mat(bgr.size(), CV_8UC1, cv::Scalar(255));
  } else {
    active_mask = valid_mask.clone();
  }
  clear_piece_exclusion_regions(
    active_mask, params.page_margin_px, params.divider_center_y_px,
    params.divider_half_height_px);

  cv::Mat gray;
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
  const int blur_kernel = std::max(1, params.blur_kernel | 1);
  if (blur_kernel > 1) {
    cv::GaussianBlur(gray, gray, cv::Size(blur_kernel, blur_kernel), 0.0);
  }

  cv::Mat edges;
  cv::Canny(
    gray, edges, params.low_threshold, params.high_threshold,
    params.aperture_size, params.l2_gradient);
  cv::bitwise_and(edges, active_mask, edges);

  const cv::Mat edge_kernel = cv::getStructuringElement(
    cv::MORPH_RECT, cv::Size(3, 3));
  if (params.edge_dilate_iterations > 0) {
    cv::dilate(
      edges, edges, edge_kernel, cv::Point(-1, -1),
      params.edge_dilate_iterations);
  }
  if (params.close_iterations > 0) {
    const int close_kernel_size = std::max(1, params.close_kernel | 1);
    const cv::Mat close_kernel = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size(close_kernel_size, close_kernel_size));
    cv::morphologyEx(
      edges, edges, cv::MORPH_CLOSE, close_kernel, cv::Point(-1, -1),
      params.close_iterations);
  }
  cv::bitwise_and(edges, active_mask, edges);

  if (params.flood_fill_background) {
    cv::Mat traversable;
    cv::bitwise_not(edges, traversable);
    cv::bitwise_and(traversable, active_mask, traversable);

    cv::Mat eroded_active;
    const cv::Mat boundary_kernel = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size(3, 3));
    cv::erode(active_mask, eroded_active, boundary_kernel);
    cv::Mat seed_mask;
    cv::subtract(active_mask, eroded_active, seed_mask);

    for (int y = 0; y < seed_mask.rows; ++y) {
      const auto * seed_row = seed_mask.ptr<std::uint8_t>(y);
      for (int x = 0; x < seed_mask.cols; ++x) {
        if (seed_row[x] == 0U || traversable.at<std::uint8_t>(y, x) != 255U) {
          continue;
        }
        cv::floodFill(traversable, cv::Point(x, y), cv::Scalar(128));
      }
    }

    cv::Mat background;
    cv::inRange(traversable, cv::Scalar(128), cv::Scalar(128), background);
    cv::Mat foreground = active_mask.clone();
    foreground.setTo(0, background);
    return foreground;
  }

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(edges.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  cv::Mat foreground = cv::Mat::zeros(bgr.size(), CV_8UC1);
  if (!contours.empty()) {
    cv::drawContours(foreground, contours, -1, cv::Scalar(255), cv::FILLED);
  }
  if (!valid_mask.empty()) {
    cv::bitwise_and(foreground, valid_mask, foreground);
  }
  clear_piece_exclusion_regions(
    foreground, params.page_margin_px, params.divider_center_y_px,
    params.divider_half_height_px);
  return foreground;
}

inline cv::Mat make_canny_piece_foreground_mask(
  const cv::Mat & bgr, const CannyPieceForegroundMaskParams & params)
{
  return make_canny_piece_foreground_mask(bgr, cv::Mat(), params);
}

}  // namespace puzzle_perception_node

#endif  // PUZZLE_PERCEPTION_NODE__PIECE_FOREGROUND_MASK_HPP_
