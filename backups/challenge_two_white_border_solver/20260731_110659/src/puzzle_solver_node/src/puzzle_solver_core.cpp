#include "puzzle_solver_node/puzzle_solver_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>
#include <sstream>
#include <unordered_set>
#include <utility>

#include <opencv2/imgproc.hpp>

#include "puzzle_solver_node/basic_task_template.hpp"

namespace puzzle_solver
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

cv::Point2d rotate_point(const cv::Point2d & point, const double angle)
{
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);
  return cv::Point2d(
    cosine * point.x - sine * point.y,
    sine * point.x + cosine * point.y);
}

template<typename EdgeType>
double edge_length(const EdgeType & edge)
{
  return cv::norm(edge.second - edge.first);
}

int bit_count(int value)
{
  int count = 0;
  while (value != 0) {
    value &= value - 1;
    ++count;
  }
  return count;
}

double polygon_area(const std::vector<cv::Point2d> & polygon)
{
  std::vector<cv::Point2f> points;
  points.reserve(polygon.size());
  for (const auto & point : polygon) {
    points.emplace_back(static_cast<float>(point.x), static_cast<float>(point.y));
  }
  return std::abs(cv::contourArea(points));
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
  if (std::abs(signed_twice_area) > 1e-9) {
    return weighted * (1.0 / (3.0 * signed_twice_area));
  }
  return std::accumulate(
    polygon.begin(), polygon.end(), cv::Point2d(0.0, 0.0)) *
    (1.0 / std::max<std::size_t>(1U, polygon.size()));
}

double cross_2d(const cv::Point2d & first, const cv::Point2d & second)
{
  return first.x * second.y - first.y * second.x;
}

struct ClearanceConstraint
{
  std::size_t first_piece{0U};
  std::size_t second_piece{0U};
  cv::Point2d normal{0.0, 0.0};
};

struct Bounds2d
{
  double min_x{0.0};
  double min_y{0.0};
  double max_x{0.0};
  double max_y{0.0};
};

Bounds2d polygon_bounds(const std::vector<cv::Point2d> & polygon)
{
  Bounds2d bounds;
  if (polygon.empty()) {
    return bounds;
  }
  bounds.min_x = bounds.max_x = polygon.front().x;
  bounds.min_y = bounds.max_y = polygon.front().y;
  for (const auto & point : polygon) {
    bounds.min_x = std::min(bounds.min_x, point.x);
    bounds.min_y = std::min(bounds.min_y, point.y);
    bounds.max_x = std::max(bounds.max_x, point.x);
    bounds.max_y = std::max(bounds.max_y, point.y);
  }
  return bounds;
}

bool bounds_intersect(const Bounds2d & first, const Bounds2d & second)
{
  return first.min_x <= second.max_x && first.max_x >= second.min_x &&
         first.min_y <= second.max_y && first.max_y >= second.min_y;
}

std::vector<cv::Point2f> to_point2f_polygon(const std::vector<cv::Point2d> & polygon)
{
  std::vector<cv::Point2f> output;
  output.reserve(polygon.size());
  for (const auto & point : polygon) {
    output.emplace_back(static_cast<float>(point.x), static_cast<float>(point.y));
  }
  return output;
}

bool convex_overlap_area(
  const std::vector<cv::Point2d> & first, const std::vector<cv::Point2d> & second,
  double & area_mm2)
{
  area_mm2 = 0.0;
  if (first.size() < 3U || second.size() < 3U) {
    return false;
  }
  const auto first_f = to_point2f_polygon(first);
  const auto second_f = to_point2f_polygon(second);
  if (!cv::isContourConvex(first_f) || !cv::isContourConvex(second_f)) {
    return false;
  }
  std::vector<cv::Point2f> intersection;
  const float area = cv::intersectConvexConvex(first_f, second_f, intersection, true);
  if (!std::isfinite(area)) {
    return false;
  }
  area_mm2 = std::max(0.0, static_cast<double>(area));
  return true;
}

std::vector<ClearanceConstraint> find_clearance_constraints(
  const std::vector<std::vector<cv::Point2d>> & polygons)
{
  std::vector<ClearanceConstraint> constraints;
  std::vector<cv::Point2d> centroids;
  centroids.reserve(polygons.size());
  for (const auto & polygon : polygons) {
    centroids.push_back(polygon_centroid(polygon));
  }

  constexpr double kCollinearToleranceMm = 1e-4;
  for (std::size_t first_piece = 0; first_piece < polygons.size(); ++first_piece) {
    for (std::size_t second_piece = first_piece + 1U;
      second_piece < polygons.size(); ++second_piece)
    {
      double longest_shared_segment = 0.0;
      cv::Point2d best_normal(0.0, 0.0);
      const auto & first_polygon = polygons[first_piece];
      const auto & second_polygon = polygons[second_piece];
      for (std::size_t first_edge = 0; first_edge < first_polygon.size(); ++first_edge) {
        const cv::Point2d first_start = first_polygon[first_edge];
        const cv::Point2d first_finish =
          first_polygon[(first_edge + 1U) % first_polygon.size()];
        const cv::Point2d first_direction = first_finish - first_start;
        const double first_length = cv::norm(first_direction);
        if (first_length < 1e-6) {
          continue;
        }
        const cv::Point2d tangent = first_direction * (1.0 / first_length);
        for (std::size_t second_edge = 0; second_edge < second_polygon.size(); ++second_edge) {
          const cv::Point2d second_start = second_polygon[second_edge];
          const cv::Point2d second_finish =
            second_polygon[(second_edge + 1U) % second_polygon.size()];
          const cv::Point2d second_direction = second_finish - second_start;
          const double second_length = cv::norm(second_direction);
          if (second_length < 1e-6 ||
            std::abs(cross_2d(tangent, second_direction * (1.0 / second_length))) >
            kCollinearToleranceMm ||
            std::abs(cross_2d(tangent, second_start - first_start)) >
            kCollinearToleranceMm ||
            std::abs(cross_2d(tangent, second_finish - first_start)) >
            kCollinearToleranceMm)
          {
            continue;
          }
          const double first_min = std::min(
            first_start.dot(tangent), first_finish.dot(tangent));
          const double first_max = std::max(
            first_start.dot(tangent), first_finish.dot(tangent));
          const double second_min = std::min(
            second_start.dot(tangent), second_finish.dot(tangent));
          const double second_max = std::max(
            second_start.dot(tangent), second_finish.dot(tangent));
          const double overlap = std::min(first_max, second_max) -
            std::max(first_min, second_min);
          if (overlap <= kCollinearToleranceMm || overlap <= longest_shared_segment) {
            continue;
          }
          cv::Point2d normal(-tangent.y, tangent.x);
          if ((centroids[second_piece] - centroids[first_piece]).dot(normal) < 0.0) {
            normal *= -1.0;
          }
          longest_shared_segment = overlap;
          best_normal = normal;
        }
      }
      if (longest_shared_segment > 0.0) {
        constraints.push_back(ClearanceConstraint{
            first_piece, second_piece, best_normal});
      }
    }
  }
  return constraints;
}

bool compute_piece_clearance_offsets(
  const std::vector<std::vector<cv::Point2d>> & polygons, const double gap_mm,
  std::vector<cv::Point2d> & offsets)
{
  offsets.assign(polygons.size(), cv::Point2d(0.0, 0.0));
  if (gap_mm <= 0.0 || polygons.size() < 2U) {
    return true;
  }
  const auto constraints = find_clearance_constraints(polygons);
  if (constraints.empty()) {
    return true;
  }

  const int variable_count = static_cast<int>(2U * polygons.size());
  const int row_count = static_cast<int>(constraints.size()) + 2;
  cv::Mat matrix = cv::Mat::zeros(row_count, variable_count, CV_64F);
  cv::Mat values = cv::Mat::zeros(row_count, 1, CV_64F);
  for (std::size_t row = 0; row < constraints.size(); ++row) {
    const auto & constraint = constraints[row];
    matrix.at<double>(static_cast<int>(row), 2 * constraint.first_piece) =
      -constraint.normal.x;
    matrix.at<double>(static_cast<int>(row), 2 * constraint.first_piece + 1) =
      -constraint.normal.y;
    matrix.at<double>(static_cast<int>(row), 2 * constraint.second_piece) =
      constraint.normal.x;
    matrix.at<double>(static_cast<int>(row), 2 * constraint.second_piece + 1) =
      constraint.normal.y;
    values.at<double>(static_cast<int>(row), 0) = gap_mm;
  }

  double total_area = 0.0;
  std::vector<double> areas;
  areas.reserve(polygons.size());
  for (const auto & polygon : polygons) {
    const double area = std::max(1e-9, polygon_area(polygon));
    areas.push_back(area);
    total_area += area;
  }
  const int x_anchor_row = static_cast<int>(constraints.size());
  const int y_anchor_row = x_anchor_row + 1;
  for (std::size_t piece = 0; piece < polygons.size(); ++piece) {
    const double weight = areas[piece] / total_area;
    matrix.at<double>(x_anchor_row, 2 * piece) = weight;
    matrix.at<double>(y_anchor_row, 2 * piece + 1) = weight;
  }

  cv::Mat solution;
  cv::Mat pseudo_inverse;
  cv::invert(matrix, pseudo_inverse, cv::DECOMP_SVD);
  solution = pseudo_inverse * values;
  if (solution.rows != variable_count || solution.cols != 1)
  {
    return false;
  }
  for (std::size_t piece = 0; piece < polygons.size(); ++piece) {
    offsets[piece] = cv::Point2d(
      solution.at<double>(static_cast<int>(2 * piece), 0),
      solution.at<double>(static_cast<int>(2 * piece + 1), 0));
  }
  for (const auto & constraint : constraints) {
    const double actual_gap =
      (offsets[constraint.second_piece] - offsets[constraint.first_piece]).dot(
      constraint.normal);
    if (!std::isfinite(actual_gap) || std::abs(actual_gap - gap_mm) > 1e-6) {
      return false;
    }
  }
  return true;
}

bool signed_length_balance_possible(
  const std::vector<double> & lengths, const double tolerance_mm)
{
  std::vector<double> useful_lengths;
  useful_lengths.reserve(lengths.size());
  for (const double length : lengths) {
    if (length > 1e-6) {
      useful_lengths.push_back(length);
    }
  }
  if (useful_lengths.empty()) {
    return true;
  }
  std::sort(useful_lengths.begin(), useful_lengths.end(), std::greater<double>());
  std::vector<double> remaining(useful_lengths.size() + 1U, 0.0);
  for (int index = static_cast<int>(useful_lengths.size()) - 1; index >= 0; --index) {
    remaining[static_cast<std::size_t>(index)] =
      remaining[static_cast<std::size_t>(index + 1)] +
      useful_lengths[static_cast<std::size_t>(index)];
  }

  std::function<bool(std::size_t, double)> search =
    [&](const std::size_t index, const double balance) {
      if (index == useful_lengths.size()) {
        return std::abs(balance) <= tolerance_mm;
      }
      if (std::abs(balance) - remaining[index] > tolerance_mm) {
        return false;
      }
      const double length = useful_lengths[index];
      return search(index + 1U, balance + length) ||
             search(index + 1U, balance - length);
    };
  return search(0U, 0.0);
}

struct LengthAreaPartitionResult
{
  bool valid{false};
  double long_mm{0.0};
  double short_mm{0.0};
  double score{std::numeric_limits<double>::infinity()};
};

LengthAreaPartitionResult evaluate_length_area_partition(
  const std::vector<PieceModel> & pieces,
  const double expected_long_mm,
  const double expected_short_mm,
  const double total_area_mm2,
  const SolverConfig & config)
{
  struct SideSubset
  {
    std::uint64_t mask{0U};
    double length_mm{0.0};
  };

  LengthAreaPartitionResult best;
  std::vector<double> edge_lengths;
  for (const auto & piece : pieces) {
    const auto & polygon = piece.polygon_local_mm;
    for (std::size_t index = 0; index < polygon.size(); ++index) {
      const double length = cv::norm(polygon[(index + 1U) % polygon.size()] - polygon[index]);
      if (length > 1e-6) {
        edge_lengths.push_back(length);
      }
    }
  }
  if (edge_lengths.size() < 4U || edge_lengths.size() > 30U) {
    return best;
  }

  const double boundary_tolerance = std::max(
    std::max(0.0, config.boundary_edge_tolerance_mm), config.endpoint_tolerance_mm);
  const double relative_length_tolerance = std::max(0.01, config.edge_length_relative_tolerance);
  auto close_length = [&](const double first, const double second) {
      const double scale = std::max({1.0, std::abs(first), std::abs(second)});
      return std::abs(first - second) <=
             std::max(boundary_tolerance, relative_length_tolerance * scale);
    };

  std::vector<SideSubset> long_subsets;
  std::vector<SideSubset> short_subsets;
  const std::uint64_t limit = 1ULL << edge_lengths.size();
  for (std::uint64_t mask = 1U; mask < limit; ++mask) {
    double sum = 0.0;
    for (std::size_t index = 0; index < edge_lengths.size(); ++index) {
      if ((mask & (1ULL << index)) != 0U) {
        sum += edge_lengths[index];
      }
    }
    if (sum >= config.target_long_min_mm - boundary_tolerance &&
      sum <= config.target_long_max_mm + boundary_tolerance &&
      close_length(sum, expected_long_mm))
    {
      long_subsets.push_back(SideSubset{mask, sum});
    }
    if (sum >= config.target_short_min_mm - boundary_tolerance &&
      sum <= config.target_short_max_mm + boundary_tolerance &&
      close_length(sum, expected_short_mm))
    {
      short_subsets.push_back(SideSubset{mask, sum});
    }
  }
  auto subset_score = [&](const SideSubset & subset, const double expected) {
      return std::abs(subset.length_mm - expected) / std::max(1e-6, expected);
    };
  std::sort(long_subsets.begin(), long_subsets.end(), [&](const auto & first, const auto & second) {
      return subset_score(first, expected_long_mm) < subset_score(second, expected_long_mm);
    });
  std::sort(short_subsets.begin(), short_subsets.end(), [&](const auto & first, const auto & second) {
      return subset_score(first, expected_short_mm) < subset_score(second, expected_short_mm);
    });

  const double area_relative_tolerance = std::max(
    0.05, std::min(0.20, 2.0 * relative_length_tolerance));
  constexpr std::size_t kMaxSubsetPairsPerSide = 256U;
  for (std::size_t first_long_index = 0;
    first_long_index < long_subsets.size() && first_long_index < kMaxSubsetPairsPerSide;
    ++first_long_index)
  {
    const auto & first_long = long_subsets[first_long_index];
    for (std::size_t second_long_index = first_long_index + 1U;
      second_long_index < long_subsets.size() && second_long_index < kMaxSubsetPairsPerSide;
      ++second_long_index)
    {
      const auto & second_long = long_subsets[second_long_index];
      if ((first_long.mask & second_long.mask) != 0U ||
        !close_length(first_long.length_mm, second_long.length_mm))
      {
        continue;
      }
      const double candidate_long = 0.5 * (first_long.length_mm + second_long.length_mm);
      const std::uint64_t used_long_mask = first_long.mask | second_long.mask;
      for (std::size_t first_short_index = 0;
        first_short_index < short_subsets.size() && first_short_index < kMaxSubsetPairsPerSide;
        ++first_short_index)
      {
        const auto & first_short = short_subsets[first_short_index];
        if ((first_short.mask & used_long_mask) != 0U) {
          continue;
        }
        for (std::size_t second_short_index = first_short_index + 1U;
          second_short_index < short_subsets.size() && second_short_index < kMaxSubsetPairsPerSide;
          ++second_short_index)
        {
          const auto & second_short = short_subsets[second_short_index];
          if (((second_short.mask & (used_long_mask | first_short.mask)) != 0U) ||
            !close_length(first_short.length_mm, second_short.length_mm))
          {
            continue;
          }
          const double candidate_short =
            0.5 * (first_short.length_mm + second_short.length_mm);
          const double candidate_area = std::max(1e-6, candidate_long * candidate_short);
          const double area_relative_error = std::abs(candidate_area - total_area_mm2) /
            std::max(1e-6, candidate_area);
          if (area_relative_error > area_relative_tolerance) {
            continue;
          }
          const std::uint64_t boundary_mask =
            used_long_mask | first_short.mask | second_short.mask;
          std::vector<double> unused_edge_lengths;
          for (std::size_t index = 0; index < edge_lengths.size(); ++index) {
            if ((boundary_mask & (1ULL << index)) == 0U) {
              unused_edge_lengths.push_back(edge_lengths[index]);
            }
          }
          const double balance_tolerance = std::max(
            boundary_tolerance,
            relative_length_tolerance * std::max(candidate_long, candidate_short));
          if (!signed_length_balance_possible(unused_edge_lengths, balance_tolerance)) {
            continue;
          }
          const double score =
            std::abs(first_long.length_mm - second_long.length_mm) /
            std::max(1e-6, candidate_long) +
            std::abs(first_short.length_mm - second_short.length_mm) /
            std::max(1e-6, candidate_short) +
            std::abs(candidate_long - expected_long_mm) / std::max(1e-6, expected_long_mm) +
            std::abs(candidate_short - expected_short_mm) / std::max(1e-6, expected_short_mm) +
            area_relative_error;
          if (!best.valid || score < best.score) {
            best.valid = true;
            best.long_mm = candidate_long;
            best.short_mm = candidate_short;
            best.score = score;
          }
        }
      }
    }
  }
  return best;
}

struct TemplateFit
{
  bool valid{false};
  double score{std::numeric_limits<double>::infinity()};
  RigidTransform transform;
};

TemplateFit fit_template_piece(
  const PieceModel & piece, const BasicTaskTemplatePiece & target_piece,
  const cv::Point2d & target_offset, const SolverConfig & config,
  const bool reflect_source = false)
{
  TemplateFit best;
  std::vector<cv::Point2d> reflected_source;
  if (reflect_source) {
    reflected_source.reserve(piece.polygon_local_mm.size());
    for (const auto & point : piece.polygon_local_mm) {
      reflected_source.emplace_back(-point.x, point.y);
    }
  }
  const auto & source = reflect_source ? reflected_source : piece.polygon_local_mm;
  if (source.size() != target_piece.target_polygon_mm.size() || source.size() < 3U) {
    return best;
  }

  const double measured_area = piece.area_mm2 > 0.0 ?
    piece.area_mm2 : polygon_area(source);
  const double area_relative_error = std::abs(measured_area - target_piece.area_mm2) /
    std::max(1e-6, target_piece.area_mm2);
  if (area_relative_error > config.basic_template_piece_area_relative_tolerance) {
    return best;
  }

  const std::size_t count = source.size();
  for (int reverse = 0; reverse <= 1; ++reverse) {
    for (std::size_t shift = 0; shift < count; ++shift) {
      std::vector<cv::Point2d> target;
      target.reserve(count);
      for (std::size_t index = 0; index < count; ++index) {
        const std::size_t target_index = reverse == 0 ?
          (shift + index) % count :
          (shift + count - index % count) % count;
        target.push_back(target_piece.target_polygon_mm[target_index] + target_offset);
      }

      const cv::Point2d source_mean = std::accumulate(
        source.begin(), source.end(), cv::Point2d(0.0, 0.0)) * (1.0 / count);
      const cv::Point2d target_mean = std::accumulate(
        target.begin(), target.end(), cv::Point2d(0.0, 0.0)) * (1.0 / count);
      double dot = 0.0;
      double cross = 0.0;
      for (std::size_t index = 0; index < count; ++index) {
        const cv::Point2d source_centered = source[index] - source_mean;
        const cv::Point2d target_centered = target[index] - target_mean;
        dot += source_centered.dot(target_centered);
        cross += source_centered.x * target_centered.y -
          source_centered.y * target_centered.x;
      }
      if (std::abs(dot) + std::abs(cross) < 1e-9) {
        continue;
      }

      RigidTransform transform;
      transform.angle_rad = std::atan2(cross, dot);
      transform.translation = target_mean - rotate_point(source_mean, transform.angle_rad);
      double squared_error = 0.0;
      double max_error = 0.0;
      for (std::size_t index = 0; index < count; ++index) {
        const cv::Point2d delta = transform.apply(source[index]) - target[index];
        const double error = cv::norm(delta);
        squared_error += error * error;
        max_error = std::max(max_error, error);
      }
      const double rms = std::sqrt(squared_error / count);
      if (rms > config.basic_template_max_vertex_rms_mm ||
        max_error > config.basic_template_max_vertex_error_mm)
      {
        continue;
      }
      const double score = rms + config.basic_template_area_weight * area_relative_error;
      if (score < best.score) {
        best.valid = true;
        best.score = score;
        best.transform = transform;
      }
    }
  }
  return best;
}
}  // namespace

cv::Point2d RigidTransform::apply(const cv::Point2d & point) const
{
  return rotate_point(point, angle_rad) + translation;
}

PuzzleSolverCore::PuzzleSolverCore(SolverConfig config)
: config_(std::move(config))
{
}

PuzzleSolverCore::Assembly PuzzleSolverCore::make_leaf(
  const PieceModel & piece, const int piece_index) const
{
  Assembly assembly;
  assembly.mask = 1 << piece_index;
  assembly.poses.push_back(PiecePose{piece_index, RigidTransform{}});
  for (std::size_t edge_index = 0; edge_index < piece.polygon_local_mm.size(); ++edge_index) {
    assembly.exposed_edges.push_back(EdgeRef{
      piece_index,
      static_cast<int>(edge_index),
      piece.polygon_local_mm[edge_index],
      piece.polygon_local_mm[(edge_index + 1) % piece.polygon_local_mm.size()]});
  }
  return assembly;
}

SolveResult PuzzleSolverCore::solve(
  const std::vector<PieceModel> & pieces,
  const cv::Mat & rectified_bgr,
  const double pixels_per_mm,
  const cv::Point2d & rectified_origin_a4_mm) const
{
  SolveResult result;
  if (pieces.empty() || pieces.size() > 4) {
    result.status = "PIECE_COUNT_INVALID";
    return result;
  }
  if (config_.basic_task_template_enabled) {
    return solve_basic_task_template(pieces);
  }

  std::vector<PieceModel> prepared_pieces = pieces;
  for (auto & piece : prepared_pieces) {
    piece.confidence = std::clamp(piece.confidence, 0.0, 1.0);
    if (piece.contour_uncertainty_mm <= 0.0) {
      piece.contour_uncertainty_mm = std::max(
        0.0, config_.contour_uncertainty_base_mm +
        (1.0 - piece.confidence) * config_.contour_uncertainty_low_confidence_mm);
    }
  }

  const int state_count = 1 << static_cast<int>(prepared_pieces.size());
  std::vector<std::vector<Assembly>> states(static_cast<std::size_t>(state_count));
  for (std::size_t index = 0; index < prepared_pieces.size(); ++index) {
    states[1U << index].push_back(make_leaf(prepared_pieces[index], static_cast<int>(index)));
  }

  for (int size = 2; size <= static_cast<int>(prepared_pieces.size()); ++size) {
    for (int mask = 1; mask < state_count; ++mask) {
      if (bit_count(mask) != size) {
        continue;
      }
      std::vector<Assembly> candidates;
      for (int left_mask = (mask - 1) & mask; left_mask > 0;
        left_mask = (left_mask - 1) & mask)
      {
        const int right_mask = mask ^ left_mask;
        if (right_mask == 0 || left_mask >= right_mask) {
          continue;
        }
        for (const auto & left : states[static_cast<std::size_t>(left_mask)]) {
          for (const auto & right : states[static_cast<std::size_t>(right_mask)]) {
            struct EdgePairJob
            {
              const EdgeRef * left_edge{nullptr};
              const EdgeRef * right_edge{nullptr};
              double relative_error{0.0};
              double pattern_priority{0.0};
            };
            std::vector<EdgePairJob> edge_jobs;
            for (const auto & left_edge : left.exposed_edges) {
              for (const auto & right_edge : right.exposed_edges) {
                const double left_length = edge_length(left_edge);
                const double right_length = edge_length(right_edge);
                const double relative_error = std::abs(left_length - right_length) /
                  std::max(1e-6, std::max(left_length, right_length));
                const double partial_ratio = std::min(left_length, right_length) /
                  std::max(1e-6, std::max(left_length, right_length));
                if (relative_error > config_.edge_length_relative_tolerance &&
                  partial_ratio < config_.minimum_partial_edge_ratio)
                {
                  continue;
                }
                double pattern_priority = 0.0;
                if (config_.pattern_enabled && config_.pattern_edge_priority_enabled &&
                  relative_error <= config_.edge_length_relative_tolerance &&
                  !rectified_bgr.empty() && pixels_per_mm > 1e-6)
                {
                  const double left_strength = edge_pattern_strength(
                    prepared_pieces[static_cast<std::size_t>(left_edge.piece_index)],
                    left_edge.edge_index, rectified_bgr, pixels_per_mm, rectified_origin_a4_mm);
                  const double right_strength = edge_pattern_strength(
                    prepared_pieces[static_cast<std::size_t>(right_edge.piece_index)],
                    right_edge.edge_index, rectified_bgr, pixels_per_mm, rectified_origin_a4_mm);
                  const double shared_strength = std::min(left_strength, right_strength);
                  if (shared_strength >= config_.pattern_edge_min_strength) {
                    pattern_priority = shared_strength;
                  }
                }
                edge_jobs.push_back(EdgePairJob{
                    &left_edge, &right_edge, relative_error, pattern_priority});
              }
            }
            std::stable_sort(edge_jobs.begin(), edge_jobs.end(),
              [](const EdgePairJob & first, const EdgePairJob & second) {
                if (std::abs(first.pattern_priority - second.pattern_priority) > 1e-9) {
                  return first.pattern_priority > second.pattern_priority;
                }
                return first.relative_error < second.relative_error;
              });
            for (const auto & job : edge_jobs) {
              // BUG_POINT:PATTERN_EDGE_PRIORITY - In task-3 pattern mode, edges
              // with near-boundary marks are tried first, but all geometric
              // candidates remain eligible.
              // BUG_POINT:PARTIAL_EDGE_ALIGNMENT - Unequal edges may be a valid
              // partial match, so test both endpoint alignments. Near-equal edges
              // must keep one canonical alignment to avoid duplicate states
              // displacing valid assemblies during bounded subset pruning.
              const int variants =
                job.relative_error <= config_.edge_length_relative_tolerance ? 1 : 2;
              for (int variant = 0; variant < variants; ++variant) {
                Assembly merged;
                if (merge(
                    left, right, *job.left_edge, *job.right_edge, variant, prepared_pieces,
                    rectified_bgr, pixels_per_mm, rectified_origin_a4_mm, merged))
                {
                  candidates.push_back(std::move(merged));
                }
              }
            }
          }
        }
      }
      std::sort(candidates.begin(), candidates.end(), [](const Assembly & first, const Assembly & second) {
        return first.score < second.score;
      });
      std::vector<Assembly> unique_candidates;
      std::unordered_set<std::string> signatures;
      for (auto & candidate : candidates) {
        if (signatures.insert(assembly_signature(candidate)).second) {
          unique_candidates.push_back(std::move(candidate));
          if (unique_candidates.size() >= static_cast<std::size_t>(config_.max_states_per_subset)) {
            break;
          }
        }
      }
      states[static_cast<std::size_t>(mask)] = std::move(unique_candidates);
    }
  }

  struct FinalCandidate
  {
    double score;
    RectangleEvaluation rectangle;
    Assembly assembly;
  };
  std::vector<FinalCandidate> finals;
  for (const auto & assembly : states.back()) {
    const RectangleEvaluation rectangle = evaluate_rectangle(assembly, prepared_pieces);
    if (rectangle.valid) {
      finals.push_back(FinalCandidate{
          assembly.score + rectangle.score + rectangle.placement_cost, rectangle, assembly});
    }
  }
  if (finals.empty()) {
    result.status = "NO_FEASIBLE_PLACEMENT";
    return result;
  }
  std::sort(finals.begin(), finals.end(), [](const FinalCandidate & first, const FinalCandidate & second) {
    return first.score < second.score;
  });
  std::vector<FinalCandidate> unique_finals;
  std::unordered_set<std::string> final_signatures;
  for (auto & candidate : finals) {
    if (final_signatures.insert(assembly_signature(candidate.assembly)).second) {
      unique_finals.push_back(std::move(candidate));
    }
  }

  const auto & best = unique_finals.front();
  if (best.score > config_.maximum_solution_score) {
    result.status = "LOW_QUALITY_LAYOUT";
    result.score = best.score;
    result.second_score = unique_finals.size() > 1 ?
      unique_finals[1].score : best.score + 1000.0;
    return result;
  }
  result.solved = true;
  result.status = "SOLVED";
  result.target_width_mm = best.rectangle.width_mm;
  result.target_height_mm = best.rectangle.height_mm;
  result.score = best.score;
  result.second_score = unique_finals.size() > 1 ?
    unique_finals[1].score : best.score + 1000.0;
  result.target_pose_by_piece.resize(prepared_pieces.size());
  for (const auto & pose : best.assembly.poses) {
    result.target_pose_by_piece[static_cast<std::size_t>(pose.piece_index)] =
      compose(best.rectangle.assembly_to_target, pose.transform);
  }
  if (config_.target_piece_gap_mm > 0.0) {
    std::vector<std::vector<cv::Point2d>> target_polygons;
    target_polygons.reserve(prepared_pieces.size());
    for (std::size_t index = 0; index < prepared_pieces.size(); ++index) {
      target_polygons.push_back(transformed_polygon(
          prepared_pieces[index], result.target_pose_by_piece[index]));
    }
    std::vector<cv::Point2d> clearance_offsets;
    if (!compute_piece_clearance_offsets(
        target_polygons, config_.target_piece_gap_mm, clearance_offsets))
    {
      result.solved = false;
      result.status = "TARGET_GAP_LAYOUT_INVALID";
      result.target_pose_by_piece.clear();
      return result;
    }
    cv::Point2d common_shift(0.0, 0.0);
    if (config_.free_target_pose_enabled) {
      double min_x = std::numeric_limits<double>::infinity();
      double min_y = std::numeric_limits<double>::infinity();
      double max_x = -std::numeric_limits<double>::infinity();
      double max_y = -std::numeric_limits<double>::infinity();
      for (std::size_t index = 0; index < target_polygons.size(); ++index) {
        for (const auto & point : target_polygons[index]) {
          const cv::Point2d shifted = point + clearance_offsets[index];
          min_x = std::min(min_x, shifted.x);
          min_y = std::min(min_y, shifted.y);
          max_x = std::max(max_x, shifted.x);
          max_y = std::max(max_y, shifted.y);
        }
      }
      const double frame_min_x = config_.placement_frame_origin_x_mm +
        config_.placement_frame_margin_mm;
      const double frame_min_y = config_.placement_frame_origin_y_mm +
        config_.placement_frame_margin_mm;
      const double frame_max_x = config_.placement_frame_origin_x_mm +
        config_.placement_frame_width_mm - config_.placement_frame_margin_mm;
      const double frame_max_y = config_.placement_frame_origin_y_mm +
        config_.placement_frame_height_mm - config_.placement_frame_margin_mm;
      if (max_x - min_x > frame_max_x - frame_min_x + 1e-6 ||
        max_y - min_y > frame_max_y - frame_min_y + 1e-6)
      {
        result.solved = false;
        result.status = "TARGET_GAP_OUTSIDE_PLACEMENT_FRAME";
        result.target_pose_by_piece.clear();
        return result;
      }
      if (min_x < frame_min_x) {
        common_shift.x = frame_min_x - min_x;
      } else if (max_x > frame_max_x) {
        common_shift.x = frame_max_x - max_x;
      }
      if (min_y < frame_min_y) {
        common_shift.y = frame_min_y - min_y;
      } else if (max_y > frame_max_y) {
        common_shift.y = frame_max_y - max_y;
      }
    }
    for (std::size_t index = 0; index < result.target_pose_by_piece.size(); ++index) {
      result.target_pose_by_piece[index].translation += clearance_offsets[index] + common_shift;
    }
  }
  return result;
}

SolveResult PuzzleSolverCore::solve_basic_task_template(
  const std::vector<PieceModel> & pieces) const
{
  SolveResult result;
  if (pieces.size() != kBasicTaskPieceCount) {
    result.status = "BASIC_TEMPLATE_PIECE_COUNT_INVALID";
    return result;
  }

  const cv::Point2d target_offset(
    config_.target_center_x_a4_mm - 0.5 * kBasicTaskWidthMm,
    config_.target_center_y_a4_mm - 0.5 * kBasicTaskHeightMm);
  const auto & templates = basic_task_template_pieces();
  std::vector<std::vector<cv::Point2d>> target_polygons;
  target_polygons.reserve(templates.size());
  for (const auto & target_piece : templates) {
    std::vector<cv::Point2d> polygon;
    polygon.reserve(target_piece.target_polygon_mm.size());
    for (const auto & point : target_piece.target_polygon_mm) {
      polygon.push_back(point + target_offset);
    }
    target_polygons.push_back(std::move(polygon));
  }
  std::vector<cv::Point2d> clearance_offsets;
  if (!compute_piece_clearance_offsets(
      target_polygons, config_.target_piece_gap_mm, clearance_offsets))
  {
    result.status = "TARGET_GAP_LAYOUT_INVALID";
    return result;
  }
  std::array<std::array<TemplateFit, kBasicTaskPieceCount>, kBasicTaskPieceCount> fits;
  for (std::size_t piece_index = 0; piece_index < kBasicTaskPieceCount; ++piece_index) {
    for (std::size_t template_index = 0; template_index < kBasicTaskPieceCount; ++template_index) {
      fits[piece_index][template_index] = fit_template_piece(
        pieces[piece_index], templates[template_index],
        target_offset + clearance_offsets[template_index], config_);
    }
  }

  std::array<std::size_t, kBasicTaskPieceCount> assignment{0U, 1U, 2U, 3U};
  double best_score = std::numeric_limits<double>::infinity();
  double second_score = std::numeric_limits<double>::infinity();
  std::vector<RigidTransform> best_transforms;
  std::array<std::size_t, kBasicTaskPieceCount> best_assignment{0U, 1U, 2U, 3U};
  do {
    double score = 0.0;
    bool valid = true;
    std::vector<RigidTransform> transforms(kBasicTaskPieceCount);
    for (std::size_t piece_index = 0; piece_index < kBasicTaskPieceCount; ++piece_index) {
      const auto & fit = fits[piece_index][assignment[piece_index]];
      if (!fit.valid) {
        valid = false;
        break;
      }
      score += fit.score;
      transforms[piece_index] = fit.transform;
    }
    if (!valid) {
      continue;
    }
    if (score < best_score) {
      second_score = best_score;
      best_score = score;
      best_transforms = std::move(transforms);
      best_assignment = assignment;
    } else if (score < second_score) {
      second_score = score;
    }
  } while (std::next_permutation(assignment.begin(), assignment.end()));

  if (best_transforms.empty()) {
    std::array<std::array<TemplateFit, kBasicTaskPieceCount>, kBasicTaskPieceCount>
    reflected_fits;
    for (std::size_t piece_index = 0; piece_index < kBasicTaskPieceCount; ++piece_index) {
      for (std::size_t template_index = 0; template_index < kBasicTaskPieceCount;
        ++template_index)
      {
        reflected_fits[piece_index][template_index] = fit_template_piece(
          pieces[piece_index], templates[template_index],
          target_offset + clearance_offsets[template_index], config_, true);
      }
    }
    assignment = {0U, 1U, 2U, 3U};
    double best_flip_score = std::numeric_limits<double>::infinity();
    std::array<bool, kBasicTaskPieceCount> best_requires_flip{};
    do {
      bool valid = true;
      bool any_flip = false;
      double score = 0.0;
      std::array<bool, kBasicTaskPieceCount> requires_flip{};
      for (std::size_t piece_index = 0; piece_index < kBasicTaskPieceCount; ++piece_index) {
        const std::size_t template_index = assignment[piece_index];
        const auto & rigid_fit = fits[piece_index][template_index];
        const auto & reflected_fit = reflected_fits[piece_index][template_index];
        if (rigid_fit.valid && (!reflected_fit.valid || rigid_fit.score <= reflected_fit.score)) {
          score += rigid_fit.score;
        } else if (reflected_fit.valid) {
          score += reflected_fit.score;
          requires_flip[piece_index] = true;
          any_flip = true;
        } else {
          valid = false;
          break;
        }
      }
      if (valid && any_flip && score < best_flip_score) {
        best_flip_score = score;
        best_requires_flip = requires_flip;
      }
    } while (std::next_permutation(assignment.begin(), assignment.end()));

    if (std::isfinite(best_flip_score)) {
      // BUG_POINT:BASIC_TEMPLATE_FLIP_REQUIRED - A planar X/Y/rotation command
      // cannot realize a reflection. Report the visually identified pieces and
      // publish no placements instead of disguising a face-down piece as solved.
      std::ostringstream status;
      status << "BASIC_TEMPLATE_FLIP_REQUIRED:";
      bool first = true;
      for (std::size_t piece_index = 0; piece_index < kBasicTaskPieceCount; ++piece_index) {
        if (!best_requires_flip[piece_index]) {
          continue;
        }
        if (!first) {
          status << ',';
        }
        status << 'P' << pieces[piece_index].id;
        first = false;
      }
      result.status = status.str();
      return result;
    }
    // BUG_POINT:BASIC_TEMPLATE_MISMATCH - A missing vertex, incorrect piece area or
    // excessive plane-mapping error must cause a retry instead of a generic rectangle guess.
    result.status = "BASIC_TEMPLATE_MISMATCH";
    return result;
  }

  // The local origin supplied by perception is the measured area centroid.
  // Once a fixed template is accepted, use its exact nominal centroid for the
  // target centre; measurement residual remains a confidence gate and must not
  // bias the commanded target position.
  for (std::size_t piece_index = 0; piece_index < best_transforms.size(); ++piece_index) {
    const std::size_t template_index = best_assignment[piece_index];
    best_transforms[piece_index].translation =
      polygon_centroid(templates[template_index].target_polygon_mm) +
      target_offset + clearance_offsets[template_index];
  }

  result.solved = true;
  result.status = "SOLVED";
  result.target_width_mm = kBasicTaskWidthMm;
  result.target_height_mm = kBasicTaskHeightMm;
  result.score = best_score;
  result.second_score = std::isfinite(second_score) ? second_score : best_score + 1000.0;
  result.target_pose_by_piece = std::move(best_transforms);
  return result;
}

bool PuzzleSolverCore::merge(
  const Assembly & left, const Assembly & right,
  const EdgeRef & left_edge, const EdgeRef & right_edge,
  const int alignment_variant,
  const std::vector<PieceModel> & pieces,
  const cv::Mat & rectified_bgr, const double pixels_per_mm,
  const cv::Point2d & rectified_origin_a4_mm,
  Assembly & output) const
{
  const double left_length = edge_length(left_edge);
  const double right_length = edge_length(right_edge);
  if (left_length < 1e-6 || right_length < 1e-6) {
    return false;
  }
  const cv::Point2d left_unit = (left_edge.second - left_edge.first) * (1.0 / left_length);
  const cv::Point2d right_unit = (right_edge.second - right_edge.first) * (1.0 / right_length);
  cv::Point2d left_segment_first = left_edge.first;
  cv::Point2d left_segment_second = left_edge.second;
  cv::Point2d right_segment_first = right_edge.first;
  cv::Point2d right_segment_second = right_edge.second;
  if (left_length > right_length + 1e-6) {
    if (alignment_variant == 0) {
      left_segment_second = left_edge.first + left_unit * right_length;
    } else {
      left_segment_first = left_edge.second - left_unit * right_length;
    }
  } else if (right_length > left_length + 1e-6) {
    if (alignment_variant == 0) {
      right_segment_second = right_edge.first + right_unit * left_length;
    } else {
      right_segment_first = right_edge.second - right_unit * left_length;
    }
  }

  const cv::Point2d left_direction = left_segment_first - left_segment_second;
  const cv::Point2d right_direction = right_segment_second - right_segment_first;
  const double rotation = std::atan2(left_direction.y, left_direction.x) -
    std::atan2(right_direction.y, right_direction.x);
  RigidTransform right_to_left;
  right_to_left.angle_rad = rotation;
  right_to_left.translation = left_segment_second - rotate_point(right_segment_first, rotation);

  const cv::Point2d transformed_endpoint = right_to_left.apply(right_segment_second);
  const double endpoint_error = cv::norm(transformed_endpoint - left_segment_first);
  if (endpoint_error > config_.endpoint_tolerance_mm) {
    return false;
  }

  Assembly transformed_right = right;
  for (auto & pose : transformed_right.poses) {
    pose.transform = compose(right_to_left, pose.transform);
  }
  for (auto & edge : transformed_right.exposed_edges) {
    edge.first = right_to_left.apply(edge.first);
    edge.second = right_to_left.apply(edge.second);
  }

  const auto overlap = overlap_evaluation(left, transformed_right, pieces);
  if (overlap.core_area_mm2 > config_.overlap_tolerance_mm2 ||
    (config_.nominal_overlap_tolerance_mm2 > 0.0 &&
    overlap.nominal_area_mm2 > config_.nominal_overlap_tolerance_mm2))
  {
    return false;
  }

  output.mask = left.mask | right.mask;
  output.poses = left.poses;
  output.poses.insert(output.poses.end(), transformed_right.poses.begin(), transformed_right.poses.end());
  auto same_edge = [](const EdgeRef & first, const EdgeRef & second) {
      return first.piece_index == second.piece_index && first.edge_index == second.edge_index &&
             cv::norm(first.first - second.first) < 1e-6 &&
             cv::norm(first.second - second.second) < 1e-6;
    };
  output.exposed_edges.reserve(left.exposed_edges.size() + transformed_right.exposed_edges.size());
  for (const auto & edge : left.exposed_edges) {
    if (!same_edge(edge, left_edge)) {
      output.exposed_edges.push_back(edge);
    }
  }
  if (left_length > right_length + 1e-6) {
    const EdgeRef residual = alignment_variant == 0 ?
      EdgeRef{left_edge.piece_index, left_edge.edge_index, left_segment_second, left_edge.second} :
      EdgeRef{left_edge.piece_index, left_edge.edge_index, left_edge.first, left_segment_first};
    if (edge_length(residual) > 1e-6) {
      output.exposed_edges.push_back(residual);
    }
  }
  EdgeRef transformed_selected_right{
    right_edge.piece_index, right_edge.edge_index,
    right_to_left.apply(right_edge.first), right_to_left.apply(right_edge.second)};
  for (const auto & edge : transformed_right.exposed_edges) {
    if (!same_edge(edge, transformed_selected_right)) {
      output.exposed_edges.push_back(edge);
    }
  }
  if (right_length > left_length + 1e-6) {
    const EdgeRef residual_original = alignment_variant == 0 ?
      EdgeRef{right_edge.piece_index, right_edge.edge_index, right_segment_second, right_edge.second} :
      EdgeRef{right_edge.piece_index, right_edge.edge_index, right_edge.first, right_segment_first};
    EdgeRef residual = residual_original;
    residual.first = right_to_left.apply(residual_original.first);
    residual.second = right_to_left.apply(residual_original.second);
    if (edge_length(residual) > 1e-6) {
      output.exposed_edges.push_back(residual);
    }
  }

  const double length_error = std::abs(left_length - right_length) /
    std::max(1e-6, std::max(left_length, right_length));
  const bool full_edge_match = length_error <= config_.edge_length_relative_tolerance;
  double pattern_cost = 0.0;
  if (config_.pattern_enabled && full_edge_match && !rectified_bgr.empty() && pixels_per_mm > 1e-6) {
    const double seam_cost = texture_cost(
      pieces[static_cast<std::size_t>(left_edge.piece_index)], left_edge.edge_index,
      pieces[static_cast<std::size_t>(right_edge.piece_index)], right_edge.edge_index,
      rectified_bgr, pixels_per_mm, rectified_origin_a4_mm);
    double effective_texture_weight = config_.texture_weight;
    double priority_bonus = 0.0;
    if (config_.pattern_edge_priority_enabled) {
      const double left_strength = edge_pattern_strength(
        pieces[static_cast<std::size_t>(left_edge.piece_index)], left_edge.edge_index,
        rectified_bgr, pixels_per_mm, rectified_origin_a4_mm);
      const double right_strength = edge_pattern_strength(
        pieces[static_cast<std::size_t>(right_edge.piece_index)], right_edge.edge_index,
        rectified_bgr, pixels_per_mm, rectified_origin_a4_mm);
      const double shared_strength = std::min(left_strength, right_strength);
      if (shared_strength >= config_.pattern_edge_min_strength) {
        priority_bonus = config_.pattern_edge_priority_weight * shared_strength * (1.0 - seam_cost);
      } else {
        effective_texture_weight *= 0.35;
      }
    }
    pattern_cost = effective_texture_weight * seam_cost - priority_bonus;
  }
  output.score = left.score + right.score +
    (full_edge_match ? config_.length_weight * length_error :
    config_.partial_edge_weight * (1.0 - std::min(left_length, right_length) /
    std::max(left_length, right_length))) +
    config_.endpoint_weight * endpoint_error +
    config_.overlap_weight * overlap.core_area_mm2 +
    pattern_cost;
  // BUG_POINT:LENGTH_AREA_SOLVER - Non-basic puzzle reconstruction now uses
  // edge lengths, area-product and internal-edge balance as the decision path.
  // Pattern mode only adds soft seam evidence after geometric gates pass. Edges
  // with clear near-boundary suits get a small ranking bonus when they match.
  return true;
}

PuzzleSolverCore::OverlapEvaluation PuzzleSolverCore::overlap_evaluation(
  const Assembly & left, const Assembly & right,
  const std::vector<PieceModel> & pieces) const
{
  OverlapEvaluation evaluation;
  std::vector<std::vector<cv::Point2d>> left_polygons;
  std::vector<std::vector<cv::Point2d>> right_polygons;
  std::vector<Bounds2d> left_bounds;
  std::vector<Bounds2d> right_bounds;
  std::vector<cv::Point2d> all_points;
  for (const auto & pose : left.poses) {
    auto polygon = transformed_polygon(pieces[static_cast<std::size_t>(pose.piece_index)], pose.transform);
    all_points.insert(all_points.end(), polygon.begin(), polygon.end());
    left_bounds.push_back(polygon_bounds(polygon));
    left_polygons.push_back(std::move(polygon));
  }
  for (const auto & pose : right.poses) {
    auto polygon = transformed_polygon(pieces[static_cast<std::size_t>(pose.piece_index)], pose.transform);
    all_points.insert(all_points.end(), polygon.begin(), polygon.end());
    right_bounds.push_back(polygon_bounds(polygon));
    right_polygons.push_back(std::move(polygon));
  }
  if (all_points.empty()) {
    return evaluation;
  }

  double exact_nominal_overlap = 0.0;
  bool exact_overlap_available = true;
  for (std::size_t left_index = 0; left_index < left_polygons.size(); ++left_index) {
    for (std::size_t right_index = 0; right_index < right_polygons.size(); ++right_index) {
      if (!bounds_intersect(left_bounds[left_index], right_bounds[right_index])) {
        continue;
      }
      double pair_overlap = 0.0;
      if (!convex_overlap_area(
          left_polygons[left_index], right_polygons[right_index], pair_overlap))
      {
        exact_overlap_available = false;
        break;
      }
      exact_nominal_overlap += pair_overlap;
    }
    if (!exact_overlap_available) {
      break;
    }
  }

  if (exact_overlap_available) {
    evaluation.nominal_area_mm2 = exact_nominal_overlap;
    if (config_.nominal_overlap_tolerance_mm2 > 0.0 &&
      exact_nominal_overlap > config_.nominal_overlap_tolerance_mm2)
    {
      evaluation.core_area_mm2 = exact_nominal_overlap;
      return evaluation;
    }
    if (exact_nominal_overlap <= config_.overlap_tolerance_mm2) {
      // BUG_POINT:OVERLAP_FAST_PATH - Most valid edge-chain candidates only
      // touch along a seam. Exact convex intersection proves they cannot exceed
      // the overlap gate, so avoid the slower raster mask path.
      evaluation.core_area_mm2 = exact_nominal_overlap;
      return evaluation;
    }
  }

  double min_x = all_points.front().x;
  double min_y = all_points.front().y;
  double max_x = min_x;
  double max_y = min_y;
  for (const auto & point : all_points) {
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
  }
  const double resolution = std::max(0.1, config_.raster_resolution_mm);
  const int width = std::max(3, static_cast<int>(std::ceil((max_x - min_x) / resolution)) + 5);
  const int height = std::max(3, static_cast<int>(std::ceil((max_y - min_y) / resolution)) + 5);
  cv::Mat left_mask(height, width, CV_8UC1, cv::Scalar(0));
  cv::Mat right_mask(height, width, CV_8UC1, cv::Scalar(0));
  cv::Mat left_core_mask(height, width, CV_8UC1, cv::Scalar(0));
  cv::Mat right_core_mask(height, width, CV_8UC1, cv::Scalar(0));
  auto draw_polygons = [&](const std::vector<std::vector<cv::Point2d>> & polygons, cv::Mat & mask) {
      for (const auto & polygon : polygons) {
        std::vector<cv::Point> pixels;
        pixels.reserve(polygon.size());
        for (const auto & point : polygon) {
          pixels.emplace_back(
            static_cast<int>(std::lround((point.x - min_x) / resolution)) + 2,
            static_cast<int>(std::lround((point.y - min_y) / resolution)) + 2);
        }
        cv::fillPoly(mask, std::vector<std::vector<cv::Point>>{pixels}, cv::Scalar(255));
      }
    };
  auto draw_core_polygon = [&](
    const PieceModel & piece, const RigidTransform & transform, cv::Mat & mask) {
      const auto polygon = transformed_polygon(piece, transform);
      if (polygon.empty()) {
        return;
      }
      std::vector<cv::Point> pixels;
      pixels.reserve(polygon.size());
      for (const auto & point : polygon) {
        pixels.emplace_back(
          static_cast<int>(std::lround((point.x - min_x) / resolution)) + 2,
          static_cast<int>(std::lround((point.y - min_y) / resolution)) + 2);
      }
      cv::Mat piece_mask(height, width, CV_8UC1, cv::Scalar(0));
      cv::fillPoly(piece_mask, std::vector<std::vector<cv::Point>>{pixels}, cv::Scalar(255));
      const int radius_px = static_cast<int>(
        std::ceil(std::max(0.0, piece.contour_uncertainty_mm) / resolution));
      if (radius_px > 0) {
        const cv::Mat kernel = cv::getStructuringElement(
          cv::MORPH_RECT, cv::Size(2 * radius_px + 1, 2 * radius_px + 1));
        cv::erode(piece_mask, piece_mask, kernel);
      }
      cv::bitwise_or(mask, piece_mask, mask);
    };
  draw_polygons(left_polygons, left_mask);
  draw_polygons(right_polygons, right_mask);
  for (const auto & pose : left.poses) {
    draw_core_polygon(pieces[static_cast<std::size_t>(pose.piece_index)], pose.transform, left_core_mask);
  }
  for (const auto & pose : right.poses) {
    draw_core_polygon(pieces[static_cast<std::size_t>(pose.piece_index)], pose.transform, right_core_mask);
  }
  cv::Mat intersection;
  cv::bitwise_and(left_mask, right_mask, intersection);
  evaluation.nominal_area_mm2 = cv::countNonZero(intersection) * resolution * resolution;
  cv::bitwise_and(left_core_mask, right_core_mask, intersection);
  evaluation.core_area_mm2 = cv::countNonZero(intersection) * resolution * resolution;
  return evaluation;
}

PuzzleSolverCore::RectangleEvaluation PuzzleSolverCore::evaluate_rectangle(
  const Assembly & assembly, const std::vector<PieceModel> & pieces) const
{
  RectangleEvaluation evaluation;
  std::vector<cv::Point2f> all_points;
  double total_area = 0.0;
  for (const auto & pose : assembly.poses) {
    const auto & piece = pieces[static_cast<std::size_t>(pose.piece_index)];
    const auto polygon = transformed_polygon(piece, pose.transform);
    // BUG_POINT:RECTANGULARITY_CONTOUR_AREA - Solver checks must score the
    // measured physical contour, not a contour mutated to make rectangle gates pass.
    total_area += piece.area_mm2 > 0.0 ? piece.area_mm2 : polygon_area(polygon);
    for (const auto & point : polygon) {
      all_points.emplace_back(static_cast<float>(point.x), static_cast<float>(point.y));
    }
  }
  if (all_points.size() < 3) {
    return evaluation;
  }

  const cv::RotatedRect rectangle = cv::minAreaRect(all_points);
  const double first_dimension = rectangle.size.width;
  const double second_dimension = rectangle.size.height;
  const double long_side = std::max(first_dimension, second_dimension);
  const double short_side = std::min(first_dimension, second_dimension);
  const double rectangle_area = std::max(1e-6, long_side * short_side);
  const double rectangularity = total_area / rectangle_area;
  const double aspect_ratio = long_side / std::max(1e-6, short_side);
  if (long_side < config_.target_long_min_mm - 1e-6 ||
    long_side > config_.target_long_max_mm + 1e-6 ||
    short_side < config_.target_short_min_mm - 1e-6 ||
    short_side > config_.target_short_max_mm + 1e-6 ||
    rectangularity < config_.minimum_rectangularity - 1e-6 ||
    aspect_ratio < config_.minimum_aspect_ratio - 1e-6)
  {
    // BUG_POINT:RECTANGLE_HARD_GATES - Target dimensions, filled rectangularity,
    // and aspect ratio are requirements, not merely soft score preferences.
    return evaluation;
  }

  cv::Point2f box[4];
  rectangle.points(box);

  double longest_edge = -1.0;
  double long_angle = 0.0;
  for (int index = 0; index < 4; ++index) {
    const cv::Point2f direction = box[(index + 1) % 4] - box[index];
    const double length = cv::norm(direction);
    if (length > longest_edge) {
      longest_edge = length;
      long_angle = std::atan2(direction.y, direction.x);
    }
  }

  const cv::Point2d center(rectangle.center.x, rectangle.center.y);
  const cv::Point2d long_axis(std::cos(long_angle), std::sin(long_angle));
  const cv::Point2d short_axis(-long_axis.y, long_axis.x);
  const double long_half = 0.5 * long_side;
  const double short_half = 0.5 * short_side;
  const double boundary_tolerance = std::max(
    std::max(0.0, config_.boundary_edge_tolerance_mm), config_.endpoint_tolerance_mm);
  const auto length_area = evaluate_length_area_partition(
    pieces, long_side, short_side, total_area, config_);
  if (!length_area.valid) {
    // BUG_POINT:LENGTH_AREA_SOLVER - Replacement reconstruction first searches
    // measured edge lengths for two equal long-side chains and two equal
    // short-side chains with L*W close to total area. The remaining measured
    // edges must pass signed-sum balance before pose placement is accepted.
    return evaluation;
  }

  if (config_.require_each_piece_boundary_edge) {
    const double minimum_edge = std::max(0.0, config_.minimum_piece_edge_mm);
    auto has_boundary_edge = [&](const std::vector<cv::Point2d> & polygon) {
        if (polygon.size() < 3U) {
          return false;
        }
        for (std::size_t index = 0; index < polygon.size(); ++index) {
          const cv::Point2d first = polygon[index] - center;
          const cv::Point2d second = polygon[(index + 1U) % polygon.size()] - center;
          const cv::Point2d edge = second - first;
          const double length = cv::norm(edge);
          if (length + 1e-6 < minimum_edge) {
            continue;
          }
          const double first_long = first.dot(long_axis);
          const double second_long = second.dot(long_axis);
          const double first_short = first.dot(short_axis);
          const double second_short = second.dot(short_axis);
          const bool along_long =
            ((std::abs(first_short - short_half) <= boundary_tolerance &&
            std::abs(second_short - short_half) <= boundary_tolerance) ||
            (std::abs(first_short + short_half) <= boundary_tolerance &&
            std::abs(second_short + short_half) <= boundary_tolerance)) &&
            std::abs(first_long) <= long_half + boundary_tolerance &&
            std::abs(second_long) <= long_half + boundary_tolerance;
          const bool along_short =
            ((std::abs(first_long - long_half) <= boundary_tolerance &&
            std::abs(second_long - long_half) <= boundary_tolerance) ||
            (std::abs(first_long + long_half) <= boundary_tolerance &&
            std::abs(second_long + long_half) <= boundary_tolerance)) &&
            std::abs(first_short) <= short_half + boundary_tolerance &&
            std::abs(second_short) <= short_half + boundary_tolerance;
          if (along_long || along_short) {
            return true;
          }
        }
        return false;
      };
    for (const auto & pose : assembly.poses) {
      if (!has_boundary_edge(transformed_polygon(
          pieces[static_cast<std::size_t>(pose.piece_index)], pose.transform)))
      {
        // BUG_POINT:CHALLENGE_BOUNDARY_EDGE - Task 2/3 require every supplied
        // piece to contribute at least one complete target-rectangle edge.
        return evaluation;
      }
    }
  }

  if (!config_.free_target_pose_enabled) {
    evaluation.assembly_to_target.angle_rad = -long_angle;
    const cv::Point2d rotated_center = rotate_point(
      cv::Point2d(rectangle.center.x, rectangle.center.y),
      evaluation.assembly_to_target.angle_rad);
    evaluation.assembly_to_target.translation = cv::Point2d(
      config_.target_center_x_a4_mm, config_.target_center_y_a4_mm) - rotated_center;
  } else {
    struct PlacementCandidate
    {
      bool valid{false};
      bool anchor_preserved{false};
      double cost{std::numeric_limits<double>::infinity()};
      RigidTransform transform;
    };
    PlacementCandidate best_candidate;
    const double frame_margin = std::max(0.0, config_.placement_frame_margin_mm);
    const double frame_min_x = config_.placement_frame_origin_x_mm + frame_margin;
    const double frame_min_y = config_.placement_frame_origin_y_mm + frame_margin;
    const double frame_max_x = config_.placement_frame_origin_x_mm +
      config_.placement_frame_width_mm - frame_margin;
    const double frame_max_y = config_.placement_frame_origin_y_mm +
      config_.placement_frame_height_mm - frame_margin;
    if (frame_max_x <= frame_min_x || frame_max_y <= frame_min_y) {
      return evaluation;
    }

    for (const auto & anchor : assembly.poses) {
      PlacementCandidate candidate;
      candidate.transform.angle_rad = -anchor.transform.angle_rad;
      candidate.transform.translation = pieces[static_cast<std::size_t>(anchor.piece_index)].source_center_a4_mm -
        rotate_point(anchor.transform.translation, candidate.transform.angle_rad);

      double min_x = std::numeric_limits<double>::infinity();
      double min_y = std::numeric_limits<double>::infinity();
      double max_x = -std::numeric_limits<double>::infinity();
      double max_y = -std::numeric_limits<double>::infinity();
      for (const auto & pose : assembly.poses) {
        const auto target_transform = compose(candidate.transform, pose.transform);
        for (const auto & point : transformed_polygon(
            pieces[static_cast<std::size_t>(pose.piece_index)], target_transform))
        {
          min_x = std::min(min_x, point.x);
          min_y = std::min(min_y, point.y);
          max_x = std::max(max_x, point.x);
          max_y = std::max(max_y, point.y);
        }
      }
      if (max_x - min_x > frame_max_x - frame_min_x + 1e-6 ||
        max_y - min_y > frame_max_y - frame_min_y + 1e-6)
      {
        continue;
      }

      double shift_x = 0.0;
      double shift_y = 0.0;
      if (min_x < frame_min_x) {
        shift_x = frame_min_x - min_x;
      } else if (max_x > frame_max_x) {
        shift_x = frame_max_x - max_x;
      }
      if (min_y < frame_min_y) {
        shift_y = frame_min_y - min_y;
      } else if (max_y > frame_max_y) {
        shift_y = frame_max_y - max_y;
      }
      candidate.transform.translation += cv::Point2d(shift_x, shift_y);
      candidate.anchor_preserved = std::abs(shift_x) < 1e-6 && std::abs(shift_y) < 1e-6;

      double movement_cost = candidate.anchor_preserved ? 0.0 : config_.anchor_fallback_penalty;
      for (const auto & pose : assembly.poses) {
        const auto target_transform = compose(candidate.transform, pose.transform);
        const cv::Point2d target_center = target_transform.apply(cv::Point2d(0.0, 0.0));
        movement_cost += config_.anchor_move_distance_weight * cv::norm(
          target_center - pieces[static_cast<std::size_t>(pose.piece_index)].source_center_a4_mm);
        const double rotation = std::atan2(std::sin(target_transform.angle_rad),
          std::cos(target_transform.angle_rad));
        movement_cost += config_.anchor_rotation_weight * std::abs(rotation * 180.0 / kPi);
      }
      candidate.valid = true;
      candidate.cost = movement_cost;
      const bool preservation_preferred = candidate.anchor_preserved &&
        !best_candidate.anchor_preserved;
      const bool same_preservation_class = candidate.anchor_preserved ==
        best_candidate.anchor_preserved;
      if (!best_candidate.valid || preservation_preferred ||
        (same_preservation_class && candidate.cost < best_candidate.cost - 1e-9))
      {
        best_candidate = candidate;
      }
    }
    if (!best_candidate.valid) {
      return evaluation;
    }
    evaluation.assembly_to_target = best_candidate.transform;
    evaluation.placement_cost = best_candidate.cost;
  }
  evaluation.valid = true;
  evaluation.width_mm = length_area.long_mm;
  evaluation.height_mm = length_area.short_mm;
  evaluation.score =
    config_.rectangularity_weight * std::abs(1.0 - rectangularity) +
    config_.length_weight * length_area.score;
  return evaluation;
}

std::vector<cv::Point2d> PuzzleSolverCore::transformed_polygon(
  const PieceModel & piece, const RigidTransform & transform) const
{
  std::vector<cv::Point2d> output;
  output.reserve(piece.polygon_local_mm.size());
  for (const auto & point : piece.polygon_local_mm) {
    output.push_back(transform.apply(point));
  }
  return output;
}

double PuzzleSolverCore::edge_pattern_strength(
  const PieceModel & piece, const int edge_index,
  const cv::Mat & rectified_bgr, const double pixels_per_mm,
  const cv::Point2d & rectified_origin_a4_mm) const
{
  if (rectified_bgr.empty() || piece.polygon_local_mm.empty() || pixels_per_mm <= 1e-6) {
    return 0.0;
  }
  cv::Mat gray;
  cv::cvtColor(rectified_bgr, gray, cv::COLOR_BGR2GRAY);
  cv::Mat lab;
  cv::cvtColor(rectified_bgr, lab, cv::COLOR_BGR2Lab);
  cv::Mat gradient_x;
  cv::Mat gradient_y;
  cv::Mat gradient;
  cv::Sobel(gray, gradient_x, CV_32F, 1, 0, 3);
  cv::Sobel(gray, gradient_y, CV_32F, 0, 1, 3);
  cv::magnitude(gradient_x, gradient_y, gradient);

  const auto & local_polygon = piece.polygon_local_mm;
  const cv::Point2d first =
    local_polygon[static_cast<std::size_t>(edge_index)] + piece.source_center_a4_mm;
  const cv::Point2d second =
    local_polygon[(static_cast<std::size_t>(edge_index) + 1) % local_polygon.size()] +
    piece.source_center_a4_mm;
  cv::Point2d direction = second - first;
  const double length = cv::norm(direction);
  if (length < 1e-6) {
    return 0.0;
  }
  direction *= 1.0 / length;
  cv::Point2d normal(-direction.y, direction.x);
  std::vector<cv::Point2f> source_polygon;
  source_polygon.reserve(local_polygon.size());
  for (const auto & point : local_polygon) {
    const cv::Point2d source_point = point + piece.source_center_a4_mm;
    source_polygon.emplace_back(
      static_cast<float>(source_point.x), static_cast<float>(source_point.y));
  }
  const cv::Point2d midpoint = 0.5 * (first + second);
  if (cv::pointPolygonTest(
      source_polygon,
      cv::Point2f(
        static_cast<float>(midpoint.x + normal.x),
        static_cast<float>(midpoint.y + normal.y)), false) < 0.0)
  {
    normal *= -1.0;
  }

  std::vector<double> gray_values;
  std::vector<double> a_values;
  std::vector<double> b_values;
  double gradient_sum = 0.0;
  for (int along_index = 0; along_index < config_.texture_samples_along_edge; ++along_index) {
    const double ratio = (along_index + 0.5) / config_.texture_samples_along_edge;
    const cv::Point2d edge_point = first + ratio * (second - first);
    for (int across_index = 1; across_index <= config_.texture_samples_across_edge; ++across_index) {
      const double distance = config_.texture_band_mm * across_index /
        config_.texture_samples_across_edge;
      const cv::Point2d sample_mm = edge_point + normal * distance;
      const cv::Point2f sample_px(
        static_cast<float>((sample_mm.x - rectified_origin_a4_mm.x) * pixels_per_mm),
        static_cast<float>((sample_mm.y - rectified_origin_a4_mm.y) * pixels_per_mm));
      if (sample_px.x < 0.0F || sample_px.y < 0.0F ||
        sample_px.x >= gray.cols || sample_px.y >= gray.rows)
      {
        return 0.0;
      }
      cv::Mat gray_sample;
      cv::Mat gradient_sample;
      cv::Mat lab_sample;
      cv::getRectSubPix(gray, cv::Size(1, 1), sample_px, gray_sample);
      cv::getRectSubPix(gradient, cv::Size(1, 1), sample_px, gradient_sample);
      cv::getRectSubPix(lab, cv::Size(1, 1), sample_px, lab_sample);
      gray_values.push_back(gray_sample.at<std::uint8_t>(0, 0));
      gradient_sum += gradient_sample.at<float>(0, 0);
      const cv::Vec3b lab_value = lab_sample.at<cv::Vec3b>(0, 0);
      a_values.push_back(lab_value[1]);
      b_values.push_back(lab_value[2]);
    }
  }
  if (gray_values.size() < 4U) {
    return 0.0;
  }

  auto standard_deviation = [](const std::vector<double> & values) {
      const double mean = std::accumulate(values.begin(), values.end(), 0.0) /
        std::max<std::size_t>(1U, values.size());
      double variance = 0.0;
      for (const double value : values) {
        const double delta = value - mean;
        variance += delta * delta;
      }
      variance /= std::max<std::size_t>(1U, values.size());
      return std::sqrt(variance);
    };
  const double gray_strength = standard_deviation(gray_values) / 64.0;
  const double chroma_strength = std::sqrt(
    standard_deviation(a_values) * standard_deviation(a_values) +
    standard_deviation(b_values) * standard_deviation(b_values)) / 48.0;
  const double gradient_strength =
    (gradient_sum / static_cast<double>(gray_values.size())) / 128.0;
  return std::clamp(
    0.35 * gray_strength + 0.40 * chroma_strength + 0.25 * gradient_strength,
    0.0, 1.0);
}

double PuzzleSolverCore::texture_cost(
  const PieceModel & left_piece, const int left_edge,
  const PieceModel & right_piece, const int right_edge,
  const cv::Mat & rectified_bgr, const double pixels_per_mm,
  const cv::Point2d & rectified_origin_a4_mm) const
{
  if (rectified_bgr.empty() || left_piece.polygon_local_mm.empty() || right_piece.polygon_local_mm.empty()) {
    return 0.5;
  }
  cv::Mat gray;
  cv::cvtColor(rectified_bgr, gray, cv::COLOR_BGR2GRAY);
  cv::Mat lab;
  cv::cvtColor(rectified_bgr, lab, cv::COLOR_BGR2Lab);
  cv::Mat gradient_x;
  cv::Mat gradient_y;
  cv::Mat gradient;
  cv::Sobel(gray, gradient_x, CV_32F, 1, 0, 3);
  cv::Sobel(gray, gradient_y, CV_32F, 0, 1, 3);
  cv::magnitude(gradient_x, gradient_y, gradient);

  struct TextureDescriptor
  {
    std::vector<double> gray_values;
    std::vector<double> gradient_values;
    std::vector<cv::Vec3d> lab_values;
  };

  auto descriptor = [&](const PieceModel & piece, const int edge_index, const bool reverse) {
      TextureDescriptor values;
      const auto & local_polygon = piece.polygon_local_mm;
      const cv::Point2d first = local_polygon[static_cast<std::size_t>(edge_index)] + piece.source_center_a4_mm;
      const cv::Point2d second =
        local_polygon[(static_cast<std::size_t>(edge_index) + 1) % local_polygon.size()] +
        piece.source_center_a4_mm;
      cv::Point2d direction = second - first;
      const double length = cv::norm(direction);
      if (length < 1e-6) {
        return values;
      }
      direction *= 1.0 / length;
      cv::Point2d normal(-direction.y, direction.x);
      std::vector<cv::Point2f> source_polygon;
      for (const auto & point : local_polygon) {
        const cv::Point2d source_point = point + piece.source_center_a4_mm;
        source_polygon.emplace_back(static_cast<float>(source_point.x), static_cast<float>(source_point.y));
      }
      const cv::Point2d midpoint = 0.5 * (first + second);
      if (cv::pointPolygonTest(
          source_polygon,
          cv::Point2f(
            static_cast<float>(midpoint.x + normal.x),
            static_cast<float>(midpoint.y + normal.y)), false) < 0.0)
      {
        normal *= -1.0;
      }
      for (int along_index = 0; along_index < config_.texture_samples_along_edge; ++along_index) {
        double ratio = (along_index + 0.5) / config_.texture_samples_along_edge;
        if (reverse) {
          ratio = 1.0 - ratio;
        }
        const cv::Point2d edge_point = first + ratio * (second - first);
        for (int across_index = 1; across_index <= config_.texture_samples_across_edge; ++across_index) {
          const double distance = config_.texture_band_mm * across_index /
            config_.texture_samples_across_edge;
          const cv::Point2d sample_mm = edge_point + normal * distance;
          const cv::Point2f sample_px(
            static_cast<float>((sample_mm.x - rectified_origin_a4_mm.x) * pixels_per_mm),
            static_cast<float>((sample_mm.y - rectified_origin_a4_mm.y) * pixels_per_mm));
          if (sample_px.x < 0.0F || sample_px.y < 0.0F ||
            sample_px.x >= gray.cols || sample_px.y >= gray.rows)
          {
            return TextureDescriptor{};
          }
          cv::Mat gray_sample;
          cv::Mat gradient_sample;
          cv::Mat lab_sample;
          cv::getRectSubPix(gray, cv::Size(1, 1), sample_px, gray_sample);
          cv::getRectSubPix(gradient, cv::Size(1, 1), sample_px, gradient_sample);
          cv::getRectSubPix(lab, cv::Size(1, 1), sample_px, lab_sample);
          values.gray_values.push_back(gray_sample.at<std::uint8_t>(0, 0));
          values.gradient_values.push_back(gradient_sample.at<float>(0, 0));
          const cv::Vec3b lab_value = lab_sample.at<cv::Vec3b>(0, 0);
          values.lab_values.emplace_back(lab_value[0], lab_value[1], lab_value[2]);
        }
      }
      return values;
    };

  const auto left_values = descriptor(left_piece, left_edge, false);
  const auto right_values = descriptor(right_piece, right_edge, true);
  if (left_values.gray_values.empty() ||
    left_values.gray_values.size() != right_values.gray_values.size())
  {
    return 0.5;
  }

  auto moments = [](const std::vector<double> & first, const std::vector<double> & second) {
      struct Result
      {
        double first_mean;
        double second_mean;
        double first_variance;
        double second_variance;
        double covariance;
      } result{};
      const double count = static_cast<double>(first.size());
      result.first_mean = std::accumulate(first.begin(), first.end(), 0.0) / count;
      result.second_mean = std::accumulate(second.begin(), second.end(), 0.0) / count;
      for (std::size_t index = 0; index < first.size(); ++index) {
        const double first_delta = first[index] - result.first_mean;
        const double second_delta = second[index] - result.second_mean;
        result.first_variance += first_delta * first_delta;
        result.second_variance += second_delta * second_delta;
        result.covariance += first_delta * second_delta;
      }
      result.first_variance /= count;
      result.second_variance /= count;
      result.covariance /= count;
      return result;
    };
  auto zncc_cost = [&](
    const std::vector<double> & first, const std::vector<double> & second,
    const double flat_scale) {
      const auto statistics = moments(first, second);
      const double energy = std::sqrt(
        statistics.first_variance * statistics.second_variance);
      if (energy < 1e-6) {
        return std::clamp(
          std::abs(statistics.first_mean - statistics.second_mean) /
          std::max(1e-6, flat_scale), 0.0, 1.0);
      }
      return std::clamp(0.5 * (1.0 - statistics.covariance / energy), 0.0, 1.0);
    };
  auto ssim_cost = [&](const std::vector<double> & first, const std::vector<double> & second) {
      const auto statistics = moments(first, second);
      constexpr double c1 = 6.5025;   // (0.01 * 255)^2
      constexpr double c2 = 58.5225;  // (0.03 * 255)^2
      const double numerator =
        (2.0 * statistics.first_mean * statistics.second_mean + c1) *
        (2.0 * statistics.covariance + c2);
      const double denominator =
        (statistics.first_mean * statistics.first_mean +
        statistics.second_mean * statistics.second_mean + c1) *
        (statistics.first_variance + statistics.second_variance + c2);
      const double ssim = denominator > 1e-9 ? numerator / denominator : 0.0;
      return std::clamp(0.5 * (1.0 - ssim), 0.0, 1.0);
    };

  double color_cost = 0.0;
  for (std::size_t index = 0; index < left_values.lab_values.size(); ++index) {
    color_cost += cv::norm(left_values.lab_values[index] - right_values.lab_values[index]) /
      441.67295593;
  }
  color_cost = std::clamp(
    color_cost / std::max<std::size_t>(1U, left_values.lab_values.size()), 0.0, 1.0);
  const double gray_cost = zncc_cost(
    left_values.gray_values, right_values.gray_values, 255.0);
  const double gradient_cost = zncc_cost(
    left_values.gradient_values, right_values.gradient_values, 1020.0);
  const double structure_cost = ssim_cost(
    left_values.gray_values, right_values.gray_values);
  const double weight_sum = std::max(
    1e-9, config_.texture_color_cost_weight + config_.texture_gray_zncc_weight +
    config_.texture_gradient_zncc_weight + config_.texture_ssim_weight);
  return std::clamp(
    (config_.texture_color_cost_weight * color_cost +
    config_.texture_gray_zncc_weight * gray_cost +
    config_.texture_gradient_zncc_weight * gradient_cost +
    config_.texture_ssim_weight * structure_cost) / weight_sum,
    0.0, 1.0);
}

RigidTransform PuzzleSolverCore::compose(
  const RigidTransform & outer, const RigidTransform & inner)
{
  RigidTransform result;
  result.angle_rad = outer.angle_rad + inner.angle_rad;
  result.translation = rotate_point(inner.translation, outer.angle_rad) + outer.translation;
  return result;
}

std::string PuzzleSolverCore::assembly_signature(const Assembly & assembly)
{
  if (assembly.poses.empty()) {
    return "empty";
  }
  const auto anchor = *std::min_element(
    assembly.poses.begin(), assembly.poses.end(),
    [](const PiecePose & first, const PiecePose & second) {
      return first.piece_index < second.piece_index;
    });
  auto normalize_point = [&](const cv::Point2d & point) {
      return rotate_point(point - anchor.transform.translation, -anchor.transform.angle_rad);
    };
  auto quantize = [](const double value) {
      return static_cast<long long>(std::llround(value * 10.0));
    };

  std::vector<PiecePose> poses = assembly.poses;
  std::sort(poses.begin(), poses.end(), [](const PiecePose & first, const PiecePose & second) {
    return first.piece_index < second.piece_index;
  });
  std::ostringstream stream;
  for (const auto & pose : poses) {
    const cv::Point2d center = normalize_point(pose.transform.translation);
    stream << 'P' << pose.piece_index << ':' << quantize(center.x) << ',' << quantize(center.y) << ',' <<
      quantize((pose.transform.angle_rad - anchor.transform.angle_rad) * 180.0 / kPi) << ';';
  }
  std::vector<std::string> edge_signatures;
  edge_signatures.reserve(assembly.exposed_edges.size());
  for (const auto & edge : assembly.exposed_edges) {
    const cv::Point2d first = normalize_point(edge.first);
    const cv::Point2d second = normalize_point(edge.second);
    std::ostringstream edge_stream;
    edge_stream << edge.piece_index << ':' << edge.edge_index << ':' <<
      quantize(first.x) << ',' << quantize(first.y) << ',' <<
      quantize(second.x) << ',' << quantize(second.y);
    edge_signatures.push_back(edge_stream.str());
  }
  std::sort(edge_signatures.begin(), edge_signatures.end());
  for (const auto & edge : edge_signatures) {
    stream << 'E' << edge << ';';
  }
  return stream.str();
}

double PuzzleSolverCore::normalized_angle_difference_deg(double first, double second)
{
  double difference = std::fmod(std::abs(first - second), 360.0);
  if (difference > 180.0) {
    difference = 360.0 - difference;
  }
  return difference;
}
}  // namespace puzzle_solver
