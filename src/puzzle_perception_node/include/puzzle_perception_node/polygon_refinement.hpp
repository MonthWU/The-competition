#ifndef PUZZLE_PERCEPTION_NODE__POLYGON_REFINEMENT_HPP_
#define PUZZLE_PERCEPTION_NODE__POLYGON_REFINEMENT_HPP_

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace puzzle_perception_node
{

struct PolygonRefinementParams
{
  double epsilon_min_px{2.0};
  double epsilon_max_px{8.0};
  int epsilon_steps{4};
  int min_vertices{3};
  int max_vertices{5};
  int minimum_points_per_edge{8};
  double maximum_rms_px{4.0};
  double maximum_residual_px{10.0};
};

struct PolygonRefinementResult
{
  bool valid{false};
  std::string status{"POLYGON_APPROX_INVALID"};
  std::vector<cv::Point2f> vertices;
  double rms_residual_px{std::numeric_limits<double>::infinity()};
  double max_residual_px{std::numeric_limits<double>::infinity()};
};

inline double distance_to_line(
  const cv::Point2f & point, const cv::Point2f & line_point,
  const cv::Point2f & line_direction)
{
  const double length = cv::norm(line_direction);
  if (length < 1e-9) {
    return std::numeric_limits<double>::infinity();
  }
  const cv::Point2f delta = point - line_point;
  return std::abs(delta.x * line_direction.y - delta.y * line_direction.x) / length;
}

inline bool intersect_lines(
  const cv::Vec4f & first, const cv::Vec4f & second, cv::Point2f & intersection)
{
  const double determinant = static_cast<double>(first[0]) * second[1] -
    static_cast<double>(first[1]) * second[0];
  if (std::abs(determinant) < 1e-6) {
    return false;
  }
  const double dx = second[2] - first[2];
  const double dy = second[3] - first[3];
  const double first_t = (dx * second[1] - dy * second[0]) / determinant;
  intersection = cv::Point2f(
    first[2] + static_cast<float>(first_t) * first[0],
    first[3] + static_cast<float>(first_t) * first[1]);
  return std::isfinite(intersection.x) && std::isfinite(intersection.y);
}

inline double distance_to_segment(
  const cv::Point2f & point, const cv::Point2f & first, const cv::Point2f & second)
{
  const cv::Point2f direction = second - first;
  const double length_squared = direction.dot(direction);
  if (length_squared < 1e-9) {
    return cv::norm(point - first);
  }
  const double ratio = std::clamp(
    static_cast<double>((point - first).dot(direction)) / length_squared, 0.0, 1.0);
  return cv::norm(point - (first + static_cast<float>(ratio) * direction));
}

inline PolygonRefinementResult refine_polygon_from_contour(
  const std::vector<cv::Point> & contour, const PolygonRefinementParams & params)
{
  PolygonRefinementResult best;
  if (contour.size() < static_cast<std::size_t>(params.min_vertices * params.minimum_points_per_edge) ||
    params.epsilon_steps <= 0 || params.epsilon_min_px <= 0.0 ||
    params.epsilon_max_px < params.epsilon_min_px || params.min_vertices < 3 ||
    params.max_vertices < params.min_vertices || params.minimum_points_per_edge < 2)
  {
    best.status = "CONTOUR_TOO_SHORT";
    return best;
  }

  for (int epsilon_index = 0; epsilon_index < params.epsilon_steps; ++epsilon_index) {
    const double ratio = params.epsilon_steps == 1 ? 0.0 :
      static_cast<double>(epsilon_index) / (params.epsilon_steps - 1);
    const double epsilon = params.epsilon_min_px +
      ratio * (params.epsilon_max_px - params.epsilon_min_px);
    std::vector<cv::Point> initial;
    cv::approxPolyDP(contour, initial, epsilon, true);
    if (initial.size() < static_cast<std::size_t>(params.min_vertices) ||
      initial.size() > static_cast<std::size_t>(params.max_vertices))
    {
      continue;
    }

    std::vector<std::vector<cv::Point2f>> edge_points(initial.size());
    for (const auto & contour_point : contour) {
      const cv::Point2f point(contour_point);
      std::size_t best_edge = 0U;
      double best_distance = std::numeric_limits<double>::infinity();
      for (std::size_t edge = 0; edge < initial.size(); ++edge) {
        const cv::Point2f first(initial[edge]);
        const cv::Point2f second(initial[(edge + 1U) % initial.size()]);
        const double distance = distance_to_segment(point, first, second);
        if (distance < best_distance) {
          best_distance = distance;
          best_edge = edge;
        }
      }
      edge_points[best_edge].push_back(point);
    }

    std::vector<cv::Vec4f> lines(initial.size());
    bool enough_points = true;
    for (std::size_t edge = 0; edge < initial.size(); ++edge) {
      if (edge_points[edge].size() < static_cast<std::size_t>(params.minimum_points_per_edge)) {
        enough_points = false;
        break;
      }
      cv::fitLine(edge_points[edge], lines[edge], cv::DIST_HUBER, 0.0, 0.01, 0.01);
    }
    if (!enough_points) {
      continue;
    }

    std::vector<cv::Point2f> refined(initial.size());
    bool intersections_valid = true;
    for (std::size_t vertex = 0; vertex < initial.size(); ++vertex) {
      const std::size_t previous_edge = (vertex + initial.size() - 1U) % initial.size();
      if (!intersect_lines(lines[previous_edge], lines[vertex], refined[vertex])) {
        intersections_valid = false;
        break;
      }
    }
    if (!intersections_valid || std::abs(cv::contourArea(refined)) < 1e-6) {
      continue;
    }

    double squared_sum = 0.0;
    double maximum = 0.0;
    std::size_t residual_count = 0U;
    for (std::size_t edge = 0; edge < edge_points.size(); ++edge) {
      const cv::Point2f direction(lines[edge][0], lines[edge][1]);
      const cv::Point2f line_point(lines[edge][2], lines[edge][3]);
      for (const auto & point : edge_points[edge]) {
        const double residual = distance_to_line(point, line_point, direction);
        squared_sum += residual * residual;
        maximum = std::max(maximum, residual);
        ++residual_count;
      }
    }
    const double rms = std::sqrt(squared_sum / std::max<std::size_t>(1U, residual_count));
    if (rms < best.rms_residual_px) {
      best.vertices = std::move(refined);
      best.rms_residual_px = rms;
      best.max_residual_px = maximum;
    }
  }

  if (best.vertices.empty()) {
    return best;
  }
  if (best.rms_residual_px > params.maximum_rms_px ||
    best.max_residual_px > params.maximum_residual_px)
  {
    best.status = "CONTOUR_LINE_RESIDUAL_HIGH";
    return best;
  }
  best.valid = true;
  best.status = "OK";
  return best;
}

}  // namespace puzzle_perception_node

#endif  // PUZZLE_PERCEPTION_NODE__POLYGON_REFINEMENT_HPP_
