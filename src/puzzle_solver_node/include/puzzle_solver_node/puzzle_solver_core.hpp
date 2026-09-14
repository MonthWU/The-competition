#ifndef PUZZLE_SOLVER_NODE__PUZZLE_SOLVER_CORE_HPP_
#define PUZZLE_SOLVER_NODE__PUZZLE_SOLVER_CORE_HPP_

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace puzzle_solver
{
struct RigidTransform
{
  double angle_rad{0.0};
  cv::Point2d translation{0.0, 0.0};

  cv::Point2d apply(const cv::Point2d & point) const;
};

struct PieceModel
{
  std::uint32_t id{0};
  std::vector<cv::Point2d> polygon_local_mm;
  cv::Point2d source_center_a4_mm{0.0, 0.0};
  cv::Point2d pick_point_a4_mm{0.0, 0.0};
  double source_angle_deg{0.0};
  double area_mm2{0.0};
  double confidence{1.0};
  double contour_uncertainty_mm{0.0};
  std::vector<double> white_border_edge_confidence;
};

struct SolverConfig
{
  bool basic_task_template_enabled{false};
  double basic_template_max_vertex_rms_mm{3.0};
  double basic_template_max_vertex_error_mm{5.0};
  double basic_template_piece_area_relative_tolerance{0.18};
  double basic_template_area_weight{10.0};
  double basic_template_max_layout_hole_mm2{100.0};
  double basic_template_max_layout_outside_mm2{100.0};
  double basic_template_max_layout_overlap_mm2{30.0};
  double edge_length_relative_tolerance{0.08};
  double minimum_partial_edge_ratio{0.25};
  double endpoint_tolerance_mm{4.0};
  double overlap_tolerance_mm2{10.0};
  double nominal_overlap_tolerance_mm2{0.0};
  double contour_uncertainty_base_mm{1.5};
  double contour_uncertainty_low_confidence_mm{2.0};
  double raster_resolution_mm{0.75};
  double target_long_min_mm{90.0};
  double target_long_max_mm{120.0};
  double target_short_min_mm{50.0};
  double target_short_max_mm{90.0};
  double minimum_rectangularity{0.93};
  double minimum_aspect_ratio{1.05};
  double target_center_x_a4_mm{105.0};
  double target_center_y_a4_mm{222.75};
  // Desired normal clearance between target polygons. A positive value is
  // applied after solving and never relaxes the overlap rejection gates.
  double target_piece_gap_mm{0.2};
  // Challenge tasks allow free translation and rotation inside a configured
  // A4 sub-region. The default origin and size cover the full portrait page.
  bool free_target_pose_enabled{false};
  double placement_frame_origin_x_mm{0.0};
  double placement_frame_origin_y_mm{0.0};
  double placement_frame_width_mm{210.0};
  double placement_frame_height_mm{297.0};
  double placement_frame_margin_mm{3.0};
  double minimum_piece_edge_mm{20.0};
  bool require_each_piece_boundary_edge{false};
  double boundary_edge_tolerance_mm{2.0};
  double anchor_move_distance_weight{0.02};
  double anchor_rotation_weight{0.02};
  double anchor_fallback_penalty{100.0};
  double length_weight{20.0};
  double partial_edge_weight{3.0};
  double endpoint_weight{2.0};
  double overlap_weight{10.0};
  double rectangularity_weight{300.0};
  bool pattern_enabled{false};
  bool challenge_one_legacy_algorithm_enabled{false};
  bool challenge_two_specialized_logic_enabled{false};
  double texture_weight{20.0};
  double texture_band_mm{3.0};
  int texture_samples_along_edge{24};
  int texture_samples_across_edge{3};
  bool pattern_edge_priority_enabled{true};
  double pattern_edge_min_strength{0.12};
  double pattern_edge_priority_weight{2.0};
  double texture_color_cost_weight{1.0};
  double texture_gray_zncc_weight{1.0};
  double texture_gradient_zncc_weight{1.0};
  double texture_ssim_weight{1.0};
  bool white_border_fast_solver_enabled{false};
  double white_border_min_confidence{0.72};
  double white_border_band_mm{2.5};
  double white_border_max_chroma{45.0};
  double white_border_min_luma{178.0};
  double white_border_layout_weight{80.0};
  double seam_gradient_min_continuity{0.20};
  double center_symmetry_pass_threshold{0.50};
  double center_symmetry_direct_threshold{0.90};
  double center_symmetry_weight{35.0};
  double diagonal_specialness_weight{8.0};
  double non_character_corner_white_pass_threshold{0.60};
  double non_character_corner_white_weight{25.0};
  double card_corner_sample_mm{10.0};
  double small_rectangle_min_rectangularity{0.96};
  double small_rectangle_min_aspect_ratio{1.0};
  double same_shape_relative_tolerance{0.05};
  double same_shape_angle_tolerance_deg{6.0};
  int maximum_enumerated_edges{18};
  int enumeration_worker_threads{5};
  int max_states_per_subset{80};
  double maximum_solution_score{std::numeric_limits<double>::infinity()};
};

struct SolveResult
{
  bool solved{false};
  std::string status;
  double target_width_mm{0.0};
  double target_height_mm{0.0};
  double score{0.0};
  double second_score{0.0};
  std::vector<RigidTransform> target_pose_by_piece;
};

class PuzzleSolverCore
{
public:
  explicit PuzzleSolverCore(SolverConfig config);
  SolveResult solve(
    const std::vector<PieceModel> & pieces,
    const cv::Mat & rectified_bgr,
    double pixels_per_mm,
    const cv::Point2d & rectified_origin_a4_mm = cv::Point2d(0.0, 0.0)) const;

private:
  SolveResult solve_basic_task_template(const std::vector<PieceModel> & pieces) const;
  SolveResult solve_challenge_one_edge_enumeration(
    const std::vector<PieceModel> & pieces,
    const cv::Mat & rectified_bgr, double pixels_per_mm,
    const cv::Point2d & rectified_origin_a4_mm) const;
  SolveResult solve_challenge_two_rectangular_grid(
    const std::vector<PieceModel> & pieces,
    const cv::Mat & rectified_bgr, double pixels_per_mm,
    const cv::Point2d & rectified_origin_a4_mm) const;

  struct PiecePose
  {
    int piece_index{-1};
    RigidTransform transform;
  };

  struct EdgeRef
  {
    int piece_index{-1};
    int edge_index{-1};
    cv::Point2d first;
    cv::Point2d second;
  };

  struct Assembly
  {
    int mask{0};
    std::vector<PiecePose> poses;
    std::vector<EdgeRef> exposed_edges;
    std::vector<double> seam_continuities;
    double score{0.0};
  };

  struct RectangleEvaluation
  {
    bool valid{false};
    double score{0.0};
    double width_mm{0.0};
    double height_mm{0.0};
    double placement_cost{0.0};
    bool direct_accept{false};
    cv::Point2d center{0.0, 0.0};
    cv::Point2d long_axis{1.0, 0.0};
    cv::Point2d short_axis{0.0, 1.0};
    double long_half{0.0};
    double short_half{0.0};
    RigidTransform assembly_to_target;
  };

  struct OverlapEvaluation
  {
    double core_area_mm2{0.0};
    double nominal_area_mm2{0.0};
  };

  struct WhiteBorderEvaluation
  {
    bool valid{false};
    double score{0.0};
  };

  struct AmbiguityEvaluation
  {
    bool valid{false};
    bool direct_accept{false};
    double score{0.0};
  };

  struct RectangleShapeEvaluation
  {
    bool valid{false};
    double score{std::numeric_limits<double>::infinity()};
    double rectangularity{0.0};
    double long_side{0.0};
    double short_side{0.0};
  };

  struct SeamContinuityEvaluation
  {
    bool pass{true};
    bool evidence_available{false};
    int seam_count{0};
    double minimum{1.0};
    double average{1.0};
    double score{0.0};
  };

  struct SmallRectangleEvaluation
  {
    bool found{false};
    bool pass{false};
    double score{std::numeric_limits<double>::infinity()};
  };

  Assembly make_leaf(const PieceModel & piece, int piece_index) const;
  bool merge(
    const Assembly & left, const Assembly & right,
    const EdgeRef & left_edge, const EdgeRef & right_edge,
    int alignment_variant,
    const std::vector<PieceModel> & pieces,
    const cv::Mat & rectified_bgr, double pixels_per_mm,
    const cv::Point2d & rectified_origin_a4_mm,
    bool white_border_evidence_active,
    Assembly & output) const;
  OverlapEvaluation overlap_evaluation(
    const Assembly & left, const Assembly & right,
    const std::vector<PieceModel> & pieces) const;
  RectangleEvaluation evaluate_rectangle(
    const Assembly & assembly, const std::vector<PieceModel> & pieces,
    const cv::Mat & rectified_bgr, double pixels_per_mm,
    const cv::Point2d & rectified_origin_a4_mm,
    bool white_border_evidence_active) const;
  std::vector<cv::Point2d> transformed_polygon(
    const PieceModel & piece, const RigidTransform & transform) const;
  std::vector<PieceModel> prepare_pieces(
    const std::vector<PieceModel> & pieces,
    const cv::Mat & rectified_bgr, double pixels_per_mm,
    const cv::Point2d & rectified_origin_a4_mm) const;
  double white_border_edge_score(
    const PieceModel & piece, int edge_index,
    const cv::Mat & rectified_bgr, double pixels_per_mm,
    const cv::Point2d & rectified_origin_a4_mm) const;
  WhiteBorderEvaluation evaluate_white_border_layout(
    const Assembly & assembly, const std::vector<PieceModel> & pieces,
    const cv::Point2d & center, const cv::Point2d & long_axis,
    const cv::Point2d & short_axis, double long_half, double short_half,
    const cv::Mat & rectified_bgr, double pixels_per_mm,
    const cv::Point2d & rectified_origin_a4_mm) const;
  AmbiguityEvaluation evaluate_card_ambiguity(
    const Assembly & assembly, const std::vector<PieceModel> & pieces,
    const cv::Point2d & center, const cv::Point2d & long_axis,
    const cv::Point2d & short_axis, double long_half, double short_half,
    const cv::Mat & rectified_bgr, double pixels_per_mm,
    const cv::Point2d & rectified_origin_a4_mm) const;
  RectangleShapeEvaluation evaluate_rectangle_shape(
    const std::vector<PiecePose> & poses,
    const std::vector<PieceModel> & pieces) const;
  SeamContinuityEvaluation evaluate_internal_seam_continuity(
    const std::vector<PiecePose> & poses,
    const std::vector<PieceModel> & pieces,
    const cv::Mat & rectified_bgr, double pixels_per_mm,
    const cv::Point2d & rectified_origin_a4_mm) const;
  SmallRectangleEvaluation evaluate_small_rectangle_priority(
    const Assembly & assembly, const std::vector<PieceModel> & pieces,
    const cv::Mat & rectified_bgr, double pixels_per_mm,
    const cv::Point2d & rectified_origin_a4_mm) const;
  bool has_exchangeable_piece_pair(const std::vector<PieceModel> & pieces) const;
  bool equivalent_piece_shape(
    const PieceModel & first, const PieceModel & second) const;
  double texture_cost(
    const PieceModel & left_piece, int left_edge,
    const PieceModel & right_piece, int right_edge,
    const cv::Mat & rectified_bgr, double pixels_per_mm,
    const cv::Point2d & rectified_origin_a4_mm) const;
  double edge_pattern_strength(
    const PieceModel & piece, int edge_index,
    const cv::Mat & rectified_bgr, double pixels_per_mm,
    const cv::Point2d & rectified_origin_a4_mm) const;
  static RigidTransform compose(
    const RigidTransform & outer, const RigidTransform & inner);
  static std::string assembly_signature(const Assembly & assembly);
  static double normalized_angle_difference_deg(double first, double second);

  SolverConfig config_;
};
}  // namespace puzzle_solver

#endif  // PUZZLE_SOLVER_NODE__PUZZLE_SOLVER_CORE_HPP_
