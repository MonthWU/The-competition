#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>

#include <opencv2/core.hpp>

#include "puzzle_solver_node/basic_task_template.hpp"
#include "puzzle_solver_node/puzzle_solver_core.hpp"

namespace
{
puzzle_solver::PieceModel rectangle_piece(
  const std::uint32_t id, const double width, const double height)
{
  puzzle_solver::PieceModel piece;
  piece.id = id;
  piece.area_mm2 = width * height;
  piece.polygon_local_mm = {
    {-width / 2.0, -height / 2.0},
    {width / 2.0, -height / 2.0},
    {width / 2.0, height / 2.0},
    {-width / 2.0, height / 2.0}};
  return piece;
}

puzzle_solver::PieceModel card_half_piece(
  const std::uint32_t id, const bool left_half)
{
  auto piece = rectangle_piece(id, 50.0, 60.0);
  // Edge order from rectangle_piece(): top, right, bottom, left.  A left card
  // half exposes top/bottom/left white frame edges; a right half exposes
  // top/right/bottom. The cut edge is the only permitted internal seam.
  piece.white_border_edge_confidence = left_half ?
    std::vector<double>{1.0, 0.0, 1.0, 1.0} :
    std::vector<double>{1.0, 1.0, 1.0, 0.0};
  return piece;
}

cv::Point2d rotate_point(const cv::Point2d & point, const double angle_rad)
{
  return cv::Point2d(
    std::cos(angle_rad) * point.x - std::sin(angle_rad) * point.y,
    std::sin(angle_rad) * point.x + std::cos(angle_rad) * point.y);
}

cv::Point2d polygon_centroid(const std::vector<cv::Point2d> & polygon)
{
  double signed_twice_area = 0.0;
  cv::Point2d weighted(0.0, 0.0);
  for (std::size_t index = 0; index < polygon.size(); ++index) {
    const auto & first = polygon[index];
    const auto & second = polygon[(index + 1U) % polygon.size()];
    const double cross = first.x * second.y - second.x * first.y;
    signed_twice_area += cross;
    weighted += (first + second) * cross;
  }
  return weighted * (1.0 / (3.0 * signed_twice_area));
}

puzzle_solver::PieceModel rotated_rectangle_piece(
  const std::uint32_t id, const double width, const double height,
  const double rotation_rad, const cv::Point2d & source_center)
{
  auto piece = rectangle_piece(id, width, height);
  for (auto & point : piece.polygon_local_mm) {
    point = rotate_point(point, rotation_rad);
  }
  piece.source_center_a4_mm = source_center;
  return piece;
}

puzzle_solver::PieceModel measured_polygon_piece(
  const std::uint32_t id, const cv::Point2d & center, const double measured_area_mm2,
  const std::vector<cv::Point2d> & vertices_a4_mm)
{
  puzzle_solver::PieceModel piece;
  piece.id = id;
  piece.source_center_a4_mm = center;
  piece.area_mm2 = measured_area_mm2;
  for (const auto & vertex : vertices_a4_mm) {
    piece.polygon_local_mm.push_back(vertex - center);
  }
  return piece;
}

puzzle_solver::PieceModel basic_template_piece(
  const std::size_t template_index, const double source_rotation_rad)
{
  const auto & target = puzzle_solver::basic_task_template_pieces()[template_index];
  const cv::Point2d area_center = polygon_centroid(target.target_polygon_mm);
  puzzle_solver::PieceModel piece;
  piece.id = static_cast<std::uint32_t>(template_index + 1U);
  piece.area_mm2 = target.area_mm2;
  for (const auto & point : target.target_polygon_mm) {
    piece.polygon_local_mm.push_back(rotate_point(point - area_center, source_rotation_rad));
  }
  return piece;
}
}  // namespace

TEST(PuzzleSolverCore, SolvesSuppliedBasicTaskFourPieceTemplate)
{
  puzzle_solver::SolverConfig config;
  config.basic_task_template_enabled = true;
  config.target_long_min_mm = 96.0;
  config.target_long_max_mm = 104.0;
  config.target_short_min_mm = 56.0;
  config.target_short_max_mm = 64.0;
  config.target_center_x_a4_mm = 50.0;
  config.target_center_y_a4_mm = 30.0;
  puzzle_solver::PuzzleSolverCore solver(config);

  const std::array<double, 4> source_rotations{0.35, -0.70, 1.10, -1.45};
  std::vector<puzzle_solver::PieceModel> pieces;
  for (std::size_t index = 0; index < source_rotations.size(); ++index) {
    pieces.push_back(basic_template_piece(index, source_rotations[index]));
  }
  std::reverse(pieces[1].polygon_local_mm.begin(), pieces[1].polygon_local_mm.end());
  std::reverse(pieces[3].polygon_local_mm.begin(), pieces[3].polygon_local_mm.end());
  const auto result = solver.solve(pieces, cv::Mat(), 3.0);
  ASSERT_TRUE(result.solved) << result.status << " score=" << result.score;
  EXPECT_NEAR(result.target_width_mm, 100.0, 1e-6);
  EXPECT_NEAR(result.target_height_mm, 60.0, 1e-6);
  ASSERT_EQ(result.target_pose_by_piece.size(), 4U);

  const auto & templates = puzzle_solver::basic_task_template_pieces();
  std::array<cv::Point2d, 4> clearance_offsets;
  for (std::size_t piece_index = 0; piece_index < pieces.size(); ++piece_index) {
    EXPECT_NEAR(
      result.target_pose_by_piece[piece_index].angle_rad,
      -source_rotations[piece_index], 1e-6);
    const cv::Point2d target_center =
      result.target_pose_by_piece[piece_index].apply(cv::Point2d(0.0, 0.0));
    clearance_offsets[piece_index] =
      target_center - polygon_centroid(templates[piece_index].target_polygon_mm);
    for (const auto & source_point : pieces[piece_index].polygon_local_mm) {
      const cv::Point2d mapped = result.target_pose_by_piece[piece_index].apply(source_point);
      double nearest = std::numeric_limits<double>::infinity();
      for (const auto & expected : templates[piece_index].target_polygon_mm) {
        nearest = std::min(
          nearest, cv::norm(mapped - (expected + clearance_offsets[piece_index])));
      }
      EXPECT_LT(nearest, 1e-5);
    }
  }

  const auto seam_gap = [&clearance_offsets](
    const std::size_t first, const std::size_t second, cv::Point2d tangent) {
      tangent *= 1.0 / cv::norm(tangent);
      cv::Point2d normal(-tangent.y, tangent.x);
      const auto & templates = puzzle_solver::basic_task_template_pieces();
      if ((polygon_centroid(templates[second].target_polygon_mm) -
        polygon_centroid(templates[first].target_polygon_mm)).dot(normal) < 0.0)
      {
        normal *= -1.0;
      }
      return (clearance_offsets[second] - clearance_offsets[first]).dot(normal);
    };
  EXPECT_NEAR(seam_gap(0, 1, {16.0, 12.0}), 0.2, 1e-6);
  EXPECT_NEAR(seam_gap(0, 2, {40.0, 30.0}), 0.2, 1e-6);
  EXPECT_NEAR(seam_gap(0, 3, {24.0, 18.0}), 0.2, 1e-6);
  EXPECT_NEAR(seam_gap(1, 2, {36.0, -8.0}), 0.2, 1e-6);
  EXPECT_NEAR(seam_gap(2, 3, {76.0, 12.0}), 0.2, 1e-6);
}

TEST(PuzzleSolverCore, RejectsBasicTaskTemplateMismatch)
{
  puzzle_solver::SolverConfig config;
  config.basic_task_template_enabled = true;
  puzzle_solver::PuzzleSolverCore solver(config);
  std::vector<puzzle_solver::PieceModel> pieces;
  for (std::size_t index = 0; index < puzzle_solver::kBasicTaskPieceCount; ++index) {
    pieces.push_back(basic_template_piece(index, 0.0));
  }
  pieces[1].area_mm2 = 100.0;
  const auto result = solver.solve(pieces, cv::Mat(), 3.0);
  EXPECT_FALSE(result.solved);
  EXPECT_EQ(result.status, "BASIC_TEMPLATE_MISMATCH");
}

TEST(PuzzleSolverCore, SolvesBasicTaskMeasuredLayoutWithoutRectangleGate)
{
  puzzle_solver::SolverConfig config;
  config.basic_task_template_enabled = true;
  config.target_piece_gap_mm = 0.2;
  puzzle_solver::PuzzleSolverCore solver(config);

  std::vector<puzzle_solver::PieceModel> pieces;
  for (std::size_t index = 0; index < puzzle_solver::kBasicTaskPieceCount; ++index) {
    pieces.push_back(basic_template_piece(index, 0.0));
  }
  pieces[0].polygon_local_mm[0].x += 4.0;
  pieces[0].polygon_local_mm[0].y += 3.0;
  pieces[2].polygon_local_mm[2].x -= 4.0;
  pieces[2].polygon_local_mm[2].y -= 3.0;

  const auto result = solver.solve(pieces, cv::Mat(), 3.0);
  EXPECT_TRUE(result.solved) << result.status;
  EXPECT_EQ(result.status, "SOLVED");
  EXPECT_EQ(result.target_pose_by_piece.size(), puzzle_solver::kBasicTaskPieceCount);
}

TEST(PuzzleSolverCore, IdentifiesBasicPiecesThatRequirePhysicalFlipping)
{
  puzzle_solver::SolverConfig config;
  config.basic_task_template_enabled = true;
  puzzle_solver::PuzzleSolverCore solver(config);
  std::vector<puzzle_solver::PieceModel> pieces;
  for (std::size_t index = 0; index < puzzle_solver::kBasicTaskPieceCount; ++index) {
    pieces.push_back(basic_template_piece(index, 0.2 * static_cast<double>(index + 1U)));
  }
  for (const std::size_t index : {1U, 3U}) {
    for (auto & point : pieces[index].polygon_local_mm) {
      point.x = -point.x;
    }
  }
  const auto result = solver.solve(pieces, cv::Mat(), 3.0);
  EXPECT_FALSE(result.solved);
  EXPECT_EQ(result.status, "BASIC_TEMPLATE_FLIP_REQUIRED:P2,P4");
  EXPECT_TRUE(result.target_pose_by_piece.empty());
}

TEST(PuzzleSolverCore, SolvesFourQuadrantsIntoAllowedRectangle)
{
  puzzle_solver::SolverConfig config;
  config.target_long_min_mm = 95.0;
  config.target_long_max_mm = 125.0;
  config.target_short_min_mm = 45.0;
  config.target_short_max_mm = 65.0;
  config.minimum_rectangularity = 0.95;
  config.minimum_aspect_ratio = 1.2;
  config.max_states_per_subset = 200;
  puzzle_solver::PuzzleSolverCore solver(config);

  std::vector<puzzle_solver::PieceModel> pieces;
  for (std::uint32_t id = 1; id <= 4; ++id) {
    pieces.push_back(rectangle_piece(id, 50.0, 30.0));
  }
  const auto result = solver.solve(pieces, cv::Mat(), 3.0);
  EXPECT_TRUE(result.solved);
  EXPECT_GE(result.target_width_mm, 95.0);
  EXPECT_LE(result.target_width_mm, 125.0);
  EXPECT_GE(result.target_height_mm, 45.0);
  EXPECT_LE(result.target_height_mm, 65.0);
  EXPECT_EQ(result.target_pose_by_piece.size(), 4U);
}

TEST(PuzzleSolverCore, RejectsInvalidPieceCount)
{
  puzzle_solver::PuzzleSolverCore solver(puzzle_solver::SolverConfig{});
  const auto result = solver.solve({}, cv::Mat(), 3.0);
  EXPECT_FALSE(result.solved);
  EXPECT_EQ(result.status, "PIECE_COUNT_INVALID");
}

TEST(PuzzleSolverCore, RejectsSinglePieceOutsideRequiredRectangleDimensions)
{
  puzzle_solver::SolverConfig config;
  config.target_long_min_mm = 90.0;
  config.target_long_max_mm = 120.0;
  config.target_short_min_mm = 50.0;
  config.target_short_max_mm = 90.0;
  config.minimum_rectangularity = 0.9;
  config.minimum_aspect_ratio = 1.0;
  puzzle_solver::PuzzleSolverCore solver(config);
  const auto result = solver.solve(
    std::vector<puzzle_solver::PieceModel>{rectangle_piece(1, 73.9, 22.4)}, cv::Mat(), 3.0);
  EXPECT_FALSE(result.solved);
  EXPECT_EQ(result.status, "NO_FEASIBLE_PLACEMENT");
}

TEST(PuzzleSolverCore, RejectsLayoutAboveAbsoluteQualityGate)
{
  puzzle_solver::SolverConfig config;
  config.target_long_min_mm = 95.0;
  config.target_long_max_mm = 105.0;
  config.target_short_min_mm = 55.0;
  config.target_short_max_mm = 65.0;
  config.minimum_rectangularity = 0.9;
  config.maximum_solution_score = -1.0;
  config.max_states_per_subset = 200;
  puzzle_solver::PuzzleSolverCore solver(config);
  std::vector<puzzle_solver::PieceModel> pieces{
    rectangle_piece(1, 50.0, 60.0), rectangle_piece(2, 50.0, 60.0)};
  const auto result = solver.solve(pieces, cv::Mat(), 3.0);
  EXPECT_FALSE(result.solved);
  EXPECT_EQ(result.status, "LOW_QUALITY_LAYOUT");
  EXPECT_TRUE(std::isfinite(result.score));
}

TEST(PuzzleSolverCore, SupportsTJunctionThroughPartialEdgeMerges)
{
  puzzle_solver::SolverConfig config;
  config.target_long_min_mm = 95.0;
  config.target_long_max_mm = 105.0;
  config.target_short_min_mm = 55.0;
  config.target_short_max_mm = 65.0;
  config.minimum_rectangularity = 0.9;
  config.minimum_partial_edge_ratio = 0.4;
  config.max_states_per_subset = 300;
  puzzle_solver::PuzzleSolverCore solver(config);

  std::vector<puzzle_solver::PieceModel> pieces;
  pieces.push_back(rectangle_piece(1, 40.0, 60.0));
  pieces.push_back(rectangle_piece(2, 60.0, 30.0));
  pieces.push_back(rectangle_piece(3, 60.0, 30.0));
  const auto result = solver.solve(pieces, cv::Mat(), 3.0);
  EXPECT_TRUE(result.solved);
  EXPECT_NEAR(result.target_width_mm, 100.0, 2.0);
  EXPECT_NEAR(result.target_height_mm, 60.0, 2.0);
}

TEST(PuzzleSolverCore, RejectsLengthAreaBoundaryMismatch)
{
  puzzle_solver::SolverConfig config;
  config.target_long_min_mm = 95.0;
  config.target_long_max_mm = 105.0;
  config.target_short_min_mm = 55.0;
  config.target_short_max_mm = 65.0;
  config.minimum_rectangularity = 0.9;
  config.minimum_aspect_ratio = 1.0;
  config.max_states_per_subset = 300;
  puzzle_solver::PuzzleSolverCore solver(config);

  std::vector<puzzle_solver::PieceModel> pieces{
    measured_polygon_piece(1, {50.0, 30.0}, 5500.0,
      {{0.0, 0.0}, {100.0, 0.0}, {100.0, 60.0}, {0.0, 50.0}})};

  const auto result = solver.solve(pieces, cv::Mat(), 3.0);
  EXPECT_FALSE(result.solved);
  EXPECT_EQ(result.status, "NO_FEASIBLE_PLACEMENT");
}

TEST(PuzzleSolverCore, ExecutesCombinedTextureScoringOnAValidTwoPieceLayout)
{
  puzzle_solver::SolverConfig config;
  config.target_long_min_mm = 95.0;
  config.target_long_max_mm = 105.0;
  config.target_short_min_mm = 55.0;
  config.target_short_max_mm = 65.0;
  config.minimum_rectangularity = 0.95;
  config.pattern_enabled = true;
  config.texture_color_cost_weight = 1.0;
  config.texture_gray_zncc_weight = 1.0;
  config.texture_gradient_zncc_weight = 1.0;
  config.texture_ssim_weight = 1.0;
  config.max_states_per_subset = 200;
  puzzle_solver::PuzzleSolverCore solver(config);

  cv::Mat texture(240, 320, CV_8UC3);
  for (int y = 0; y < texture.rows; ++y) {
    for (int x = 0; x < texture.cols; ++x) {
      texture.at<cv::Vec3b>(y, x) = cv::Vec3b(
        static_cast<std::uint8_t>((x + y) % 256),
        static_cast<std::uint8_t>((2 * x + y) % 256),
        static_cast<std::uint8_t>((x + 2 * y) % 256));
    }
  }
  std::vector<puzzle_solver::PieceModel> pieces;
  pieces.push_back(rectangle_piece(1, 50.0, 60.0));
  pieces.push_back(rectangle_piece(2, 50.0, 60.0));
  pieces[0].source_center_a4_mm = {80.0, 100.0};
  pieces[1].source_center_a4_mm = {180.0, 100.0};
  const auto result = solver.solve(pieces, texture, 1.0, {0.0, 0.0});
  EXPECT_TRUE(result.solved) << result.status;
  EXPECT_TRUE(std::isfinite(result.score));
  EXPECT_EQ(result.target_pose_by_piece.size(), 2U);
}

TEST(PuzzleSolverCore, ChallengeTwoUsesWhiteBordersAsOuterFrameLock)
{
  puzzle_solver::SolverConfig config;
  config.white_border_fast_solver_enabled = true;
  config.pattern_enabled = false;
  config.target_long_min_mm = 95.0;
  config.target_long_max_mm = 105.0;
  config.target_short_min_mm = 55.0;
  config.target_short_max_mm = 65.0;
  config.minimum_rectangularity = 0.95;
  config.minimum_aspect_ratio = 1.0;
  config.require_each_piece_boundary_edge = true;
  config.minimum_piece_edge_mm = 20.0;
  config.free_target_pose_enabled = true;
  config.placement_frame_origin_x_mm = 0.0;
  config.placement_frame_origin_y_mm = 148.5;
  config.placement_frame_width_mm = 210.0;
  config.placement_frame_height_mm = 148.5;
  config.placement_frame_margin_mm = 3.0;
  puzzle_solver::PuzzleSolverCore solver(config);

  const std::vector<puzzle_solver::PieceModel> pieces{
    card_half_piece(1, true), card_half_piece(2, false)};

  const auto result = solver.solve(pieces, cv::Mat(), 3.0);
  ASSERT_TRUE(result.solved) << result.status;
  EXPECT_NEAR(result.target_width_mm, 100.0, 1e-6);
  EXPECT_NEAR(result.target_height_mm, 60.0, 1e-6);
  EXPECT_EQ(result.target_pose_by_piece.size(), pieces.size());
}

TEST(PuzzleSolverCore, ChallengeTwoRejectsWhenWhiteBordersCannotRemainOuterFrame)
{
  puzzle_solver::SolverConfig config;
  config.white_border_fast_solver_enabled = true;
  config.pattern_enabled = false;
  config.target_long_min_mm = 95.0;
  config.target_long_max_mm = 105.0;
  config.target_short_min_mm = 55.0;
  config.target_short_max_mm = 65.0;
  config.minimum_rectangularity = 0.95;
  config.minimum_aspect_ratio = 1.0;
  config.max_states_per_subset = 50;
  puzzle_solver::PuzzleSolverCore solver(config);

  std::vector<puzzle_solver::PieceModel> pieces{
    rectangle_piece(1, 50.0, 60.0), rectangle_piece(2, 50.0, 60.0)};
  pieces[0].white_border_edge_confidence = {1.0, 1.0, 1.0, 1.0};
  pieces[1].white_border_edge_confidence = {1.0, 1.0, 1.0, 1.0};

  const auto result = solver.solve(pieces, cv::Mat(), 3.0);
  EXPECT_FALSE(result.solved);
  EXPECT_EQ(result.status, "NO_FEASIBLE_PLACEMENT");
}

TEST(PuzzleSolverCore, ChallengePlacesCompleteRectangleInsideLowerHalf)
{
  puzzle_solver::SolverConfig config;
  config.target_long_min_mm = 90.0;
  config.target_long_max_mm = 120.0;
  config.target_short_min_mm = 50.0;
  config.target_short_max_mm = 90.0;
  config.minimum_aspect_ratio = 1.0;
  config.minimum_rectangularity = 0.9;
  config.max_states_per_subset = 300;
  config.free_target_pose_enabled = true;
  config.require_each_piece_boundary_edge = true;
  config.minimum_piece_edge_mm = 20.0;
  config.placement_frame_origin_x_mm = 0.0;
  config.placement_frame_origin_y_mm = 148.5;
  config.placement_frame_width_mm = 210.0;
  config.placement_frame_height_mm = 148.5;
  config.placement_frame_margin_mm = 3.0;
  puzzle_solver::PuzzleSolverCore solver(config);

  constexpr double source_rotation = 0.37;
  std::vector<puzzle_solver::PieceModel> pieces;
  pieces.push_back(rotated_rectangle_piece(1, 50.0, 30.0, source_rotation, {55.0, 45.0}));
  pieces.push_back(rotated_rectangle_piece(2, 50.0, 30.0, source_rotation, {150.0, 45.0}));
  pieces.push_back(rotated_rectangle_piece(3, 50.0, 30.0, source_rotation, {55.0, 105.0}));
  pieces.push_back(rotated_rectangle_piece(4, 50.0, 30.0, source_rotation, {150.0, 105.0}));

  const auto result = solver.solve(pieces, cv::Mat(), 3.0);
  ASSERT_TRUE(result.solved) << result.status;
  ASSERT_EQ(result.target_pose_by_piece.size(), pieces.size());
  for (std::size_t index = 0; index < pieces.size(); ++index) {
    const auto & target = result.target_pose_by_piece[index];
    const cv::Point2d target_center = target.apply({0.0, 0.0});
    EXPECT_GT(target_center.y, 148.5);
    for (const auto & local_point : pieces[index].polygon_local_mm) {
      const cv::Point2d target_point = target.apply(local_point);
      EXPECT_GE(target_point.x, 3.0 - 1e-6);
      EXPECT_LE(target_point.x, 207.0 + 1e-6);
      EXPECT_GE(target_point.y, 151.5 - 1e-6);
      EXPECT_LE(target_point.y, 294.0 + 1e-6);
    }
  }
}

TEST(PuzzleSolverCore, ChallengeRejectsPiecesWithoutRequiredBoundaryEdgeLength)
{
  puzzle_solver::SolverConfig config;
  config.target_long_min_mm = 95.0;
  config.target_long_max_mm = 105.0;
  config.target_short_min_mm = 55.0;
  config.target_short_max_mm = 65.0;
  config.minimum_aspect_ratio = 1.0;
  config.minimum_rectangularity = 0.9;
  config.max_states_per_subset = 200;
  config.free_target_pose_enabled = true;
  config.require_each_piece_boundary_edge = true;
  config.minimum_piece_edge_mm = 65.0;
  config.placement_frame_origin_x_mm = 0.0;
  config.placement_frame_origin_y_mm = 148.5;
  config.placement_frame_width_mm = 210.0;
  config.placement_frame_height_mm = 148.5;
  config.placement_frame_margin_mm = 3.0;
  puzzle_solver::PuzzleSolverCore solver(config);

  std::vector<puzzle_solver::PieceModel> pieces{
    rectangle_piece(1, 50.0, 60.0), rectangle_piece(2, 50.0, 60.0)};
  const auto result = solver.solve(pieces, cv::Mat(), 3.0);
  EXPECT_FALSE(result.solved);
  EXPECT_EQ(result.status, "NO_FEASIBLE_PLACEMENT");
}

TEST(PuzzleSolverCore, SolvesMeasuredChallengeOneWithoutContourMutation)
{
  puzzle_solver::SolverConfig config;
  config.basic_task_template_enabled = false;
  config.target_long_min_mm = 90.0;
  config.target_long_max_mm = 120.0;
  config.target_short_min_mm = 50.0;
  config.target_short_max_mm = 90.0;
  config.minimum_rectangularity = 0.95;
  config.minimum_aspect_ratio = 1.0;
  config.edge_length_relative_tolerance = 0.10;
  config.max_states_per_subset = 300;
  config.free_target_pose_enabled = true;
  config.require_each_piece_boundary_edge = false;
  config.minimum_piece_edge_mm = 20.0;
  config.placement_frame_origin_x_mm = 0.0;
  config.placement_frame_origin_y_mm = 148.5;
  config.placement_frame_width_mm = 210.0;
  config.placement_frame_height_mm = 148.5;
  config.placement_frame_margin_mm = 3.0;
  puzzle_solver::PuzzleSolverCore solver(config);
  const std::vector<puzzle_solver::PieceModel> pieces{
    measured_polygon_piece(1, {45.53173, 39.30023}, 2989.7778,
      {{14.66667, 7.66667}, {7.66667, 55.0}, {110.0, 50.0}}),
    measured_polygon_piece(2, {155.02042, 41.67985}, 3885.7778,
      {{110.0, 3.33333}, {106.0, 33.0}, {200.0, 75.33334}, {205.33333, 44.0}}),
    measured_polygon_piece(3, {58.06778, 87.29179}, 979.0555,
      {{30.66667, 98.66666}, {103.0, 86.66666}, {35.0, 75.0}}),
    measured_polygon_piece(4, {141.76807, 101.35207}, 722.0,
      {{106.0, 93.0}, {160.66667, 116.0}, {158.33333, 92.66666}})};

  const auto result = solver.solve(pieces, cv::Mat(), 3.0);
  ASSERT_TRUE(result.solved) << result.status;
  EXPECT_GE(result.target_width_mm, config.target_long_min_mm);
  EXPECT_LE(result.target_width_mm, config.target_long_max_mm);
  EXPECT_GE(result.target_height_mm, config.target_short_min_mm);
  EXPECT_LE(result.target_height_mm, config.target_short_max_mm);
  EXPECT_EQ(result.target_pose_by_piece.size(), pieces.size());
}
