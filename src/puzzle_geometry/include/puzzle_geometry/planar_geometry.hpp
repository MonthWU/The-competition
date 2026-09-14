#ifndef PUZZLE_GEOMETRY__PLANAR_GEOMETRY_HPP_
#define PUZZLE_GEOMETRY__PLANAR_GEOMETRY_HPP_

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace puzzle_geometry
{

struct Point2d
{
  double x{0.0};
  double y{0.0};
};

inline bool is_finite(const Point2d & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

inline Point2d apply_homography(
  const std::array<double, 9> & matrix, const Point2d & point)
{
  const double denominator = matrix[6] * point.x + matrix[7] * point.y + matrix[8];
  if (!std::isfinite(denominator) || std::abs(denominator) < 1e-12) {
    throw std::invalid_argument("homography maps point to infinity");
  }
  const Point2d output{
    (matrix[0] * point.x + matrix[1] * point.y + matrix[2]) / denominator,
    (matrix[3] * point.x + matrix[4] * point.y + matrix[5]) / denominator};
  if (!is_finite(output)) {
    throw std::invalid_argument("homography produced a non-finite point");
  }
  return output;
}

inline Point2d camera_centered_pixel(
  const Point2d & image_pixel, const double image_width, const double image_height,
  const bool y_axis_up)
{
  if (!is_finite(image_pixel) || !std::isfinite(image_width) ||
    !std::isfinite(image_height) || image_width <= 0.0 || image_height <= 0.0)
  {
    throw std::invalid_argument("invalid image point or dimensions");
  }
  Point2d output{
    image_pixel.x - image_width * 0.5,
    image_pixel.y - image_height * 0.5};
  if (y_axis_up) {
    output.y = -output.y;
  }
  return output;
}

inline double normalize_half_turn_deg(double angle_deg)
{
  if (!std::isfinite(angle_deg)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  angle_deg = std::fmod(angle_deg + 90.0, 180.0);
  if (angle_deg < 0.0) {
    angle_deg += 180.0;
  }
  return angle_deg - 90.0;
}

inline double normalize_full_turn_deg(double angle_deg)
{
  if (!std::isfinite(angle_deg)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  angle_deg = std::fmod(angle_deg + 180.0, 360.0);
  if (angle_deg < 0.0) {
    angle_deg += 360.0;
  }
  return angle_deg - 180.0;
}

inline double image_rotation_rad_to_ccw_delta_deg(const double image_rotation_rad)
{
  if (!std::isfinite(image_rotation_rad)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
  // Image/A4 coordinates use +Y down, so their positive mathematical rotation is
  // clockwise in the top view. The controller convention is counter-clockwise positive.
  return normalize_full_turn_deg(-image_rotation_rad * kRadiansToDegrees);
}

inline double signed_half_turn_delta_deg(
  const double target_angle_deg, const double source_angle_deg)
{
  return normalize_half_turn_deg(target_angle_deg - source_angle_deg);
}

inline double dominant_edge_angle_deg(const std::vector<Point2d> & polygon)
{
  if (polygon.size() < 2U) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double best_length_squared = -1.0;
  double best_angle_deg = std::numeric_limits<double>::quiet_NaN();
  constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
  for (std::size_t index = 0; index < polygon.size(); ++index) {
    const auto & first = polygon[index];
    const auto & second = polygon[(index + 1U) % polygon.size()];
    if (!is_finite(first) || !is_finite(second)) {
      continue;
    }
    const double dx = second.x - first.x;
    const double dy = second.y - first.y;
    const double length_squared = dx * dx + dy * dy;
    if (length_squared > best_length_squared) {
      best_length_squared = length_squared;
      best_angle_deg = std::atan2(-dy, dx) * kRadiansToDegrees;
    }
  }
  return best_length_squared > 1e-12 ?
    normalize_half_turn_deg(best_angle_deg) : std::numeric_limits<double>::quiet_NaN();
}

}  // namespace puzzle_geometry

#endif  // PUZZLE_GEOMETRY__PLANAR_GEOMETRY_HPP_
