#include "puzzle_geometry/planar_geometry.hpp"

#include <cmath>
#include <iostream>
#include <vector>

namespace
{
bool near(const double actual, const double expected, const double tolerance = 1e-6)
{
  if (std::abs(actual - expected) <= tolerance) {
    return true;
  }
  std::cerr << "expected " << expected << " but got " << actual << '\n';
  return false;
}
}  // namespace

int main()
{
  using puzzle_geometry::Point2d;
  bool success = true;
  const std::array<double, 9> matrix{2.0, 0.0, 10.0, 0.0, 3.0, -5.0, 0.0, 0.0, 1.0};
  const auto mapped = puzzle_geometry::apply_homography(matrix, Point2d{4.0, 5.0});
  success &= near(mapped.x, 18.0) && near(mapped.y, 10.0);

  const auto centered = puzzle_geometry::camera_centered_pixel(
    Point2d{700.0, 300.0}, 1280.0, 720.0, false);
  success &= near(centered.x, 60.0) && near(centered.y, -60.0);
  const auto centered_y_up = puzzle_geometry::camera_centered_pixel(
    Point2d{700.0, 300.0}, 1280.0, 720.0, true);
  success &= near(centered_y_up.x, 60.0) && near(centered_y_up.y, 60.0);

  success &= near(puzzle_geometry::normalize_half_turn_deg(100.0), -80.0);
  success &= near(puzzle_geometry::signed_half_turn_delta_deg(-80.0, 80.0), 20.0);
  success &= near(puzzle_geometry::normalize_full_turn_deg(200.0), -160.0);
  success &= near(
    puzzle_geometry::image_rotation_rad_to_ccw_delta_deg(-3.14159265358979323846 / 6.0),
    30.0);
  success &= near(
    puzzle_geometry::image_rotation_rad_to_ccw_delta_deg(3.14159265358979323846 / 4.0),
    -45.0);
  const std::vector<Point2d> polygon{{0.0, 0.0}, {40.0, 0.0}, {40.0, 10.0}, {0.0, 10.0}};
  success &= near(puzzle_geometry::dominant_edge_angle_deg(polygon), 0.0);
  const std::vector<Point2d> rising_edge{{0.0, 0.0}, {20.0, -20.0}, {20.0, -10.0}};
  success &= near(puzzle_geometry::dominant_edge_angle_deg(rising_edge), 45.0);
  return success ? 0 : 1;
}
