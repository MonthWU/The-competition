#ifndef PUZZLE_PERCEPTION_NODE__GREEN_A4_DETECTOR_HPP_
#define PUZZLE_PERCEPTION_NODE__GREEN_A4_DETECTOR_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "puzzle_perception_node/hsv_object_mask.hpp"

namespace puzzle_perception_node
{

struct GreenA4DetectorParams
{
  HsvRange hsv_range{60, 90, 78, 201, 91, 245};
  int morph_kernel{7};
  int morph_iterations{2};
  double min_area_ratio{0.08};
  double max_area_ratio{0.98};
  double min_short_long_ratio{0.30};
  double polygon_epsilon_ratio{0.02};
  double min_border_coverage{0.35};
  double line_fit_band_px{6.0};
  int line_fit_min_points{12};
  double max_corner_refine_px{25.0};
};

inline std::array<cv::Point2f, 4> order_green_a4_quad(
  const std::vector<cv::Point2f> & polygon)
{
  std::vector<cv::Point2f> points = polygon;
  std::sort(points.begin(), points.end(), [](const cv::Point2f & left, const cv::Point2f & right) {
    return left.y == right.y ? left.x < right.x : left.y < right.y;
  });
  std::array<cv::Point2f, 2> top = {points[0], points[1]};
  std::array<cv::Point2f, 2> bottom = {points[2], points[3]};
  if (top[0].x > top[1].x) {
    std::swap(top[0], top[1]);
  }
  if (bottom[0].x > bottom[1].x) {
    std::swap(bottom[0], bottom[1]);
  }
  return {top[0], top[1], bottom[1], bottom[0]};
}

inline std::array<cv::Point2f, 4> order_green_a4_quad(
  const std::vector<cv::Point> & polygon)
{
  std::vector<cv::Point2f> points;
  points.reserve(polygon.size());
  for (const auto & point : polygon) {
    points.emplace_back(static_cast<float>(point.x), static_cast<float>(point.y));
  }
  return order_green_a4_quad(points);
}

inline float green_a4_cross_2d(const cv::Point2f & first, const cv::Point2f & second)
{
  return first.x * second.y - first.y * second.x;
}

inline bool intersect_green_a4_lines(
  const cv::Vec4f & first, const cv::Vec4f & second, cv::Point2f & intersection)
{
  const cv::Point2f first_point(first[2], first[3]);
  const cv::Point2f first_direction(first[0], first[1]);
  const cv::Point2f second_point(second[2], second[3]);
  const cv::Point2f second_direction(second[0], second[1]);
  const float denominator = green_a4_cross_2d(first_direction, second_direction);
  if (std::abs(denominator) < 1.0e-6F) {
    return false;
  }
  const float scale = green_a4_cross_2d(
    second_point - first_point, second_direction) / denominator;
  intersection = first_point + scale * first_direction;
  return std::isfinite(intersection.x) && std::isfinite(intersection.y);
}

inline bool refine_green_a4_quad(
  const std::vector<cv::Point> & contour, const std::vector<cv::Point> & polygon,
  const GreenA4DetectorParams & params, std::array<cv::Point2f, 4> & refined)
{
  if (contour.empty() || polygon.size() != 4U || params.line_fit_band_px <= 0.0 ||
    params.line_fit_min_points < 2 || params.max_corner_refine_px <= 0.0)
  {
    return false;
  }

  std::array<cv::Vec4f, 4> lines;
  for (std::size_t edge_index = 0; edge_index < polygon.size(); ++edge_index) {
    const cv::Point2f start = polygon[edge_index];
    const cv::Point2f finish = polygon[(edge_index + 1U) % polygon.size()];
    const cv::Point2f edge = finish - start;
    const double squared_length = edge.dot(edge);
    const double length = std::sqrt(squared_length);
    if (length <= 1.0) {
      return false;
    }

    std::vector<cv::Point2f> edge_points;
    edge_points.reserve(contour.size() / 4U);
    for (const auto & contour_point : contour) {
      const cv::Point2f point = contour_point;
      const cv::Point2f relative = point - start;
      const double projection = relative.dot(edge) / squared_length;
      const double distance = std::abs(
        static_cast<double>(edge.x) * relative.y -
        static_cast<double>(edge.y) * relative.x) / length;
      if (projection >= 0.05 && projection <= 0.95 &&
        distance <= params.line_fit_band_px)
      {
        edge_points.push_back(point);
      }
    }
    if (edge_points.size() < static_cast<std::size_t>(params.line_fit_min_points)) {
      return false;
    }
    cv::fitLine(edge_points, lines[edge_index], cv::DIST_HUBER, 0.0, 0.01, 0.01);
  }

  std::vector<cv::Point2f> refined_polygon;
  refined_polygon.reserve(4U);
  for (std::size_t corner_index = 0; corner_index < polygon.size(); ++corner_index) {
    cv::Point2f intersection;
    if (!intersect_green_a4_lines(
        lines[(corner_index + polygon.size() - 1U) % polygon.size()],
        lines[corner_index], intersection) ||
      cv::norm(intersection - cv::Point2f(polygon[corner_index])) >
      params.max_corner_refine_px)
    {
      return false;
    }
    refined_polygon.push_back(intersection);
  }
  if (!cv::isContourConvex(refined_polygon)) {
    return false;
  }
  refined = order_green_a4_quad(refined_polygon);
  return true;
}

inline bool accept_green_a4_ordered_quad(
  const cv::Mat & mask, const std::array<cv::Point2f, 4> & ordered,
  const GreenA4DetectorParams & params, const int border_width_px,
  double & border_coverage)
{
  double width = 0.5 * (
    cv::norm(ordered[1] - ordered[0]) + cv::norm(ordered[2] - ordered[3]));
  double height = 0.5 * (
    cv::norm(ordered[3] - ordered[0]) + cv::norm(ordered[2] - ordered[1]));
  if (width <= 1.0 || height <= 1.0) {
    return false;
  }
  const double short_long_ratio = std::min(width, height) / std::max(width, height);
  if (short_long_ratio < std::clamp(params.min_short_long_ratio, 0.0, 1.0)) {
    return false;
  }

  std::vector<cv::Point> ordered_polygon;
  ordered_polygon.reserve(ordered.size());
  for (const auto & point : ordered) {
    ordered_polygon.emplace_back(cvRound(point.x), cvRound(point.y));
  }
  cv::Mat border = cv::Mat::zeros(mask.size(), CV_8UC1);
  cv::polylines(
    border, ordered_polygon, true, cv::Scalar(255),
    std::max(3, border_width_px), cv::LINE_8);
  cv::Mat green_border;
  cv::bitwise_and(mask, border, green_border);
  border_coverage = static_cast<double>(cv::countNonZero(green_border)) /
    std::max(1, cv::countNonZero(border));
  return border_coverage >= params.min_border_coverage;
}

inline bool green_a4_min_area_rect_fallback(
  const std::vector<cv::Point> & contour, const cv::Mat & mask,
  const GreenA4DetectorParams & params, const int border_width_px,
  std::array<cv::Point2f, 4> & ordered, double & border_coverage)
{
  if (contour.size() < 4U) {
    return false;
  }
  cv::RotatedRect rect = cv::minAreaRect(contour);
  if (rect.size.width <= 1.0F || rect.size.height <= 1.0F) {
    return false;
  }
  cv::Point2f vertices[4];
  rect.points(vertices);
  std::vector<cv::Point2f> polygon(vertices, vertices + 4);
  ordered = order_green_a4_quad(polygon);
  if (cv::norm(ordered[1] - ordered[0]) > cv::norm(ordered[3] - ordered[0])) {
    ordered = {ordered[1], ordered[2], ordered[3], ordered[0]};
  }
  return accept_green_a4_ordered_quad(
    mask, ordered, params, border_width_px, border_coverage);
}

inline bool detect_green_a4_quad(
  const cv::Mat & bgr, const GreenA4DetectorParams & params,
  std::array<cv::Point2f, 4> & corners, cv::Mat * output_mask = nullptr)
{
  if (bgr.empty() || bgr.type() != CV_8UC3) {
    return false;
  }

  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
  cv::Mat mask = make_hsv_object_mask(hsv, params.hsv_range);
  if (mask.empty()) {
    return false;
  }
  const int kernel_size = std::max(1, params.morph_kernel | 1);
  if (params.morph_iterations > 0) {
    const cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
    cv::morphologyEx(
      mask, mask, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), params.morph_iterations);
  }
  if (output_mask != nullptr) {
    *output_mask = mask.clone();
  }

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  const double frame_area = static_cast<double>(bgr.cols) * bgr.rows;
  double best_score = -1.0;

  for (const auto & contour : contours) {
    const double area = std::abs(cv::contourArea(contour));
    const double area_ratio = area / frame_area;
    if (area_ratio < params.min_area_ratio || area_ratio > params.max_area_ratio) {
      continue;
    }
    const double perimeter = cv::arcLength(contour, true);
    std::vector<cv::Point> polygon;
    cv::approxPolyDP(contour, polygon, params.polygon_epsilon_ratio * perimeter, true);
    if (polygon.size() != 4U || !cv::isContourConvex(polygon)) {
      std::array<cv::Point2f, 4> fallback_ordered;
      double fallback_border_coverage = 0.0;
      if (!green_a4_min_area_rect_fallback(
          contour, mask, params, std::max(3, kernel_size * 2),
          fallback_ordered, fallback_border_coverage))
      {
        continue;
      }
      const double score = area * (0.4 + 0.4 * fallback_border_coverage);
      if (score > best_score) {
        best_score = score;
        corners = fallback_ordered;
      }
      continue;
    }

    std::array<cv::Point2f, 4> ordered;
    // BUG_POINT:GREEN_A4_CORNER_JITTER - Integer approxPolyDP vertices can move several
    // pixels between static frames. Fit the full contour along each initialized edge and
    // fail closed when the four robust line intersections cannot be trusted.
    if (!refine_green_a4_quad(contour, polygon, params, ordered)) {
      continue;
    }
    double width = 0.5 * (
      cv::norm(ordered[1] - ordered[0]) + cv::norm(ordered[2] - ordered[3]));
    double height = 0.5 * (
      cv::norm(ordered[3] - ordered[0]) + cv::norm(ordered[2] - ordered[1]));
    if (width <= 1.0 || height <= 1.0) {
      continue;
    }
    if (width > height) {
      ordered = {ordered[1], ordered[2], ordered[3], ordered[0]};
    }
    // BUG_POINT:GREEN_FRAME_FALSE_POSITIVE - Do not require the A4 aspect ratio here.
    // Use only a broad anti-sliver gate; colour, area, convexity and border coverage carry
    // the common frame decision for all three tasks.
    double border_coverage = 0.0;
    if (!accept_green_a4_ordered_quad(
        mask, ordered, params, std::max(3, kernel_size * 2), border_coverage))
    {
      continue;
    }

    const double score = area * (0.5 + 0.5 * border_coverage);
    if (score > best_score) {
      best_score = score;
      corners = ordered;
    }
  }
  return best_score >= 0.0;
}

}  // namespace puzzle_perception_node

#endif  // PUZZLE_PERCEPTION_NODE__GREEN_A4_DETECTOR_HPP_
