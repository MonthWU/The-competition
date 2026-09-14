#include "puzzle_perception_node/green_a4_detector.hpp"

#include <array>
#include <cmath>
#include <iostream>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace
{
cv::Mat make_hsv_frame(
  const cv::Rect & frame_rect, const cv::Scalar & frame_hsv, const int thickness)
{
  cv::Mat hsv = cv::Mat::zeros(480, 640, CV_8UC3);
  cv::rectangle(hsv, frame_rect, frame_hsv, thickness);
  cv::Mat bgr;
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
  return bgr;
}
}  // namespace

int main()
{
  puzzle_perception_node::GreenA4DetectorParams params;
  if (params.hsv_range.h_min != 60 || params.hsv_range.h_max != 90 ||
    params.hsv_range.s_min != 78 || params.hsv_range.s_max != 201 ||
    params.hsv_range.v_min != 91 || params.hsv_range.v_max != 245)
  {
    std::cerr << "green detector defaults do not match the configured HSV contract\n";
    return 1;
  }
  params.min_area_ratio = 0.08;
  params.min_short_long_ratio = 0.30;
  if (params.line_fit_band_px != 6.0 || params.line_fit_min_points != 12 ||
    params.max_corner_refine_px != 25.0)
  {
    std::cerr << "green detector line-refinement defaults changed unexpectedly\n";
    return 1;
  }
  std::array<cv::Point2f, 4> corners;

  const cv::Mat green_frame = make_hsv_frame(
    cv::Rect(195, 63, 250, 354), cv::Scalar(75, 150, 180), 16);
  if (!puzzle_perception_node::detect_green_a4_quad(green_frame, params, corners)) {
    std::cerr << "normal green outer frame was not detected\n";
    return 1;
  }

  cv::Mat skewed_hsv = cv::Mat::zeros(480, 640, CV_8UC3);
  const std::vector<cv::Point> skewed_quad = {
    cv::Point(120, 80), cv::Point(510, 110),
    cv::Point(480, 420), cv::Point(100, 390)};
  cv::fillConvexPoly(skewed_hsv, skewed_quad, cv::Scalar(75, 150, 180), cv::LINE_8);
  cv::Mat skewed_bgr;
  cv::cvtColor(skewed_hsv, skewed_bgr, cv::COLOR_HSV2BGR);
  if (!puzzle_perception_node::detect_green_a4_quad(skewed_bgr, params, corners)) {
    std::cerr << "skewed filled green page was not detected\n";
    return 1;
  }
  auto expected = puzzle_perception_node::order_green_a4_quad(skewed_quad);
  const double expected_width = 0.5 * (
    cv::norm(expected[1] - expected[0]) + cv::norm(expected[2] - expected[3]));
  const double expected_height = 0.5 * (
    cv::norm(expected[3] - expected[0]) + cv::norm(expected[2] - expected[1]));
  if (expected_width > expected_height) {
    // The detector keeps A4-local width on the short side even when the page is
    // physically landscape in the camera frame.
    expected = {expected[1], expected[2], expected[3], expected[0]};
  }
  for (std::size_t index = 0; index < corners.size(); ++index) {
    if (cv::norm(corners[index] - expected[index]) > 2.0) {
      std::cerr << "refined skewed-page corner exceeded the 2 px test tolerance\n";
      return 1;
    }
  }

  cv::Mat occluded_hsv = cv::Mat::zeros(480, 640, CV_8UC3);
  cv::rectangle(occluded_hsv, cv::Rect(170, 45, 300, 390), cv::Scalar(75, 150, 180), cv::FILLED);
  cv::line(occluded_hsv, cv::Point(315, 435), cv::Point(315, 335), cv::Scalar(0, 0, 0), 18);
  cv::Mat occluded_bgr;
  cv::cvtColor(occluded_hsv, occluded_bgr, cv::COLOR_HSV2BGR);
  if (!puzzle_perception_node::detect_green_a4_quad(occluded_bgr, params, corners)) {
    std::cerr << "green A4 page with bottom-edge occlusion was not detected\n";
    return 1;
  }

  auto insufficient_points = params;
  insufficient_points.line_fit_min_points = 10000;
  if (puzzle_perception_node::detect_green_a4_quad(
      green_frame, insufficient_points, corners))
  {
    std::cerr << "green detector did not fail closed when line evidence was insufficient\n";
    return 1;
  }

  // The user-measured green range is strict; an old pale-green sample must no longer pass.
  const cv::Mat old_pale_green_frame = make_hsv_frame(
    cv::Rect(90, 150, 460, 180), cv::Scalar(50, 35, 230), 16);
  if (puzzle_perception_node::detect_green_a4_quad(old_pale_green_frame, params, corners)) {
    std::cerr << "out-of-range pale green frame was incorrectly accepted\n";
    return 1;
  }

  cv::Mat white_frame = cv::Mat::zeros(480, 640, CV_8UC3);
  cv::rectangle(white_frame, cv::Point(195, 63), cv::Point(445, 417), cv::Scalar(255, 255, 255), 16);
  if (puzzle_perception_node::detect_green_a4_quad(white_frame, params, corners)) {
    std::cerr << "white rectangle was incorrectly accepted\n";
    return 1;
  }

  const cv::Mat yellow_frame = make_hsv_frame(
    cv::Rect(160, 80, 320, 320), cv::Scalar(25, 180, 230), 16);
  if (puzzle_perception_node::detect_green_a4_quad(yellow_frame, params, corners)) {
    std::cerr << "yellow frame was incorrectly accepted\n";
    return 1;
  }

  const cv::Mat over_saturated_green = make_hsv_frame(
    cv::Rect(160, 80, 320, 320), cv::Scalar(75, 230, 200), 16);
  if (puzzle_perception_node::detect_green_a4_quad(over_saturated_green, params, corners)) {
    std::cerr << "green above the configured saturation range was incorrectly accepted\n";
    return 1;
  }

  const cv::Mat green_sliver = make_hsv_frame(
    cv::Rect(70, 200, 500, 70), cv::Scalar(75, 120, 200), 16);
  if (puzzle_perception_node::detect_green_a4_quad(green_sliver, params, corners)) {
    std::cerr << "extremely slender green candidate was incorrectly accepted\n";
    return 1;
  }
  return 0;
}
