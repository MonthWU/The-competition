#include <chrono>
#include <iostream>
#include <vector>

#include "puzzle_solver_node/puzzle_solver_core.hpp"

puzzle_solver::PieceModel piece(
  unsigned id, std::vector<cv::Point2d> a4, cv::Point2d center, double area, double conf)
{
  puzzle_solver::PieceModel p;
  p.id = id;
  p.source_center_a4_mm = center;
  p.pick_point_a4_mm = center;
  p.area_mm2 = area;
  p.confidence = conf;
  for (auto & v : a4) {
    p.polygon_local_mm.push_back(v - center);
  }
  return p;
}

puzzle_solver::SolverConfig base_config()
{
  puzzle_solver::SolverConfig c;
  c.basic_task_template_enabled = false;
  c.target_long_min_mm = 90.0;
  c.target_long_max_mm = 120.0;
  c.target_short_min_mm = 50.0;
  c.target_short_max_mm = 90.0;
  c.minimum_rectangularity = 0.93;
  c.minimum_aspect_ratio = 1.0;
  c.edge_length_relative_tolerance = 0.08;
  c.minimum_partial_edge_ratio = 0.25;
  c.endpoint_tolerance_mm = 4.0;
  c.overlap_tolerance_mm2 = 10.0;
  c.nominal_overlap_tolerance_mm2 = 0.0;
  c.contour_uncertainty_base_mm = 1.5;
  c.contour_uncertainty_low_confidence_mm = 2.0;
  c.raster_resolution_mm = 0.75;
  c.free_target_pose_enabled = true;
  c.placement_frame_origin_x_mm = 0.0;
  c.placement_frame_origin_y_mm = 148.5;
  c.placement_frame_width_mm = 210.0;
  c.placement_frame_height_mm = 148.5;
  c.placement_frame_margin_mm = 3.0;
  c.minimum_piece_edge_mm = 20.0;
  c.require_each_piece_boundary_edge = true;
  c.boundary_edge_tolerance_mm = 2.0;
  c.max_states_per_subset = 300;
  return c;
}

int main()
{
  std::cout.setf(std::ios::unitbuf);
  std::vector<puzzle_solver::PieceModel> pieces{
    piece(
      1,
      {{74.6667, 12.3333}, {77.3333, 40.6667}, {139.0, 60.0},
        {183.6667, 51.3333}, {179.0, 19.6667}},
      {129.9, 34.6}, 3919.8, 0.976),
    piece(
      2,
      {{31.3333, 100.3333}, {62.3333, 100.6667}, {131.3333, 69.0}, {29.0, 53.3333}},
      {66.0, 76.4}, 2992.5, 0.965),
    piece(
      3,
      {{182.6667, 114.3333}, {177.0, 92.0}, {125.0, 102.0}},
      {161.0, 103.4}, 724.3, 0.841),
    piece(
      4,
      {{104.3333, 107.3333}, {35.3333, 110.0}, {35.3333, 133.6667}},
      {60.4, 117.1}, 973.8, 0.838),
  };

  struct Case
  {
    const char * name;
    bool boundary;
    double boundary_tol;
    double edge_tol;
    double endpoint_tol;
    double rectangularity;
    int max_states;
  };

  std::vector<Case> cases{
    {"boundary_tol5", true, 5.0, 0.08, 4.0, 0.93, 300},
    {"boundary_tol8", true, 8.0, 0.08, 4.0, 0.93, 300},
  };

  for (auto cs : cases) {
    auto c = base_config();
    c.require_each_piece_boundary_edge = cs.boundary;
    c.boundary_edge_tolerance_mm = cs.boundary_tol;
    c.edge_length_relative_tolerance = cs.edge_tol;
    c.endpoint_tolerance_mm = cs.endpoint_tol;
    c.minimum_rectangularity = cs.rectangularity;
    c.max_states_per_subset = cs.max_states;
    puzzle_solver::PuzzleSolverCore solver(c);
    const auto started = std::chrono::steady_clock::now();
    const auto result = solver.solve(pieces, cv::Mat(), 3.0, {0.0, 0.0});
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    std::cout << cs.name
              << " solved=" << result.solved
              << " status=" << result.status
              << " ms=" << elapsed_ms
              << " target=" << result.target_width_mm << "x" << result.target_height_mm
              << " score=" << result.score
              << " second=" << result.second_score
              << " poses=" << result.target_pose_by_piece.size()
              << "\n";
  }
  return 0;
}
