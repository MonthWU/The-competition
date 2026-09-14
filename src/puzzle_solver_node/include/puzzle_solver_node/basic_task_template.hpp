#ifndef PUZZLE_SOLVER_NODE__BASIC_TASK_TEMPLATE_HPP_
#define PUZZLE_SOLVER_NODE__BASIC_TASK_TEMPLATE_HPP_

#include <array>
#include <cstddef>
#include <vector>

#include <opencv2/core.hpp>

namespace puzzle_solver
{

constexpr double kBasicTaskWidthMm = 100.0;
constexpr double kBasicTaskHeightMm = 60.0;
constexpr std::size_t kBasicTaskPieceCount = 4U;

struct BasicTaskTemplatePiece
{
  const char * name;
  std::vector<cv::Point2d> target_polygon_mm;
  double area_mm2;
};

inline const std::array<BasicTaskTemplatePiece, kBasicTaskPieceCount> &
basic_task_template_pieces()
{
  // Coordinates follow the supplied top-view drawing: origin at the finished
  // rectangle's upper-left corner, +X right and +Y down, all values in mm.
  static const std::array<BasicTaskTemplatePiece, kBasicTaskPieceCount> pieces{{
    {"right_triangle", {{20.0, 0.0}, {100.0, 0.0}, {100.0, 60.0}}, 2400.0},
    {"upper_quadrilateral", {{0.0, 0.0}, {20.0, 0.0}, {36.0, 12.0}, {0.0, 20.0}}, 480.0},
    {"middle_quadrilateral", {{0.0, 20.0}, {36.0, 12.0}, {76.0, 42.0}, {0.0, 30.0}}, 1080.0},
    {"lower_quadrilateral", {{0.0, 30.0}, {76.0, 42.0}, {100.0, 60.0}, {0.0, 60.0}}, 2040.0}
  }};
  return pieces;
}

}  // namespace puzzle_solver

#endif  // PUZZLE_SOLVER_NODE__BASIC_TASK_TEMPLATE_HPP_
