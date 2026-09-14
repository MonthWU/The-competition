#include "puzzle_solver_node/puzzle_solver_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>
#include <vector>

#include <geometry_msgs/msg/point32.hpp>
#include <opencv2/imgproc.hpp>
#include <puzzle_geometry/planar_geometry.hpp>

namespace
{
constexpr double kPi = 3.14159265358979323846;

bool from_ros_image(const sensor_msgs::msg::Image & image, cv::Mat & output)
{
  if (image.height == 0 || image.width == 0 || image.data.empty()) {
    output.release();
    return false;
  }
  if (image.encoding != "bgr8" && image.encoding != "rgb8") {
    output.release();
    return false;
  }
  const std::size_t minimum_step = static_cast<std::size_t>(image.width) * 3U;
  const std::size_t required_size = static_cast<std::size_t>(image.step) * image.height;
  if (image.step < minimum_step || image.data.size() < required_size) {
    output.release();
    return false;
  }
  const cv::Mat wrapped(
    static_cast<int>(image.height), static_cast<int>(image.width), CV_8UC3,
    const_cast<std::uint8_t *>(image.data.data()), static_cast<std::size_t>(image.step));
  if (image.encoding == "rgb8") {
    cv::cvtColor(wrapped, output, cv::COLOR_RGB2BGR);
  } else {
    output = wrapped.clone();
  }
  return true;
}

geometry_msgs::msg::Point32 to_point32(const cv::Point2d & point)
{
  geometry_msgs::msg::Point32 output;
  output.x = static_cast<float>(point.x);
  output.y = static_cast<float>(point.y);
  output.z = 0.0F;
  return output;
}

geometry_msgs::msg::Point32 to_point32(const puzzle_geometry::Point2d & point)
{
  geometry_msgs::msg::Point32 output;
  output.x = static_cast<float>(point.x);
  output.y = static_cast<float>(point.y);
  output.z = 0.0F;
  return output;
}

sensor_msgs::msg::Image to_bgr8_image(
  const std_msgs::msg::Header & header, const cv::Mat & image)
{
  sensor_msgs::msg::Image output;
  output.header = header;
  if (image.empty() || image.type() != CV_8UC3) {
    return output;
  }
  const cv::Mat contiguous = image.isContinuous() ? image : image.clone();
  output.height = static_cast<std::uint32_t>(contiguous.rows);
  output.width = static_cast<std::uint32_t>(contiguous.cols);
  output.encoding = "bgr8";
  output.is_bigendian = false;
  output.step = static_cast<sensor_msgs::msg::Image::_step_type>(contiguous.cols * 3);
  const auto byte_count = contiguous.total() * contiguous.elemSize();
  output.data.assign(contiguous.data, contiguous.data + byte_count);
  return output;
}
}  // namespace

PuzzleSolverNode::PuzzleSolverNode()
: Node("puzzle_solver_node")
{
  scene_topic_ = declare_parameter<std::string>("scene_topic", "puzzle/scene");
  plan_topic_ = declare_parameter<std::string>("plan_topic", "puzzle/plan");
  status_topic_ = declare_parameter<std::string>("status_topic", "puzzle/solver_status");
  task_session_topic_ = declare_parameter<std::string>(
    "task_session_topic", "puzzle/task_session");
  debug_image_topic_ = declare_parameter<std::string>(
    "solver_debug_image_topic", "puzzle/solver_image_debug");
  pixels_per_mm_ = declare_parameter<double>("pixels_per_mm", 3.0);
  placed_position_tolerance_mm_ = declare_parameter<double>("placed_position_tolerance_mm", 5.0);
  placed_angle_tolerance_deg_ = declare_parameter<double>("placed_angle_tolerance_deg", 5.0);
  minimum_score_margin_ = declare_parameter<double>("minimum_score_margin", 2.0);
  challenge_minimum_score_margin_ = declare_parameter<double>(
    "challenge_minimum_score_margin", 0.0);
  challenge_two_minimum_score_margin_ = declare_parameter<double>(
    "challenge_two_minimum_score_margin", 0.0);
  require_workspace_mapping_ = declare_parameter<bool>("require_workspace_mapping", true);
  camera_y_axis_up_ = declare_parameter<bool>("camera_y_axis_up", false);
  debug_mode_ = declare_parameter<bool>("debug_mode", false);

  puzzle_solver::SolverConfig config;
  config.basic_task_template_enabled = declare_parameter<bool>(
    "basic_task_template_enabled", true);
  config.basic_template_max_vertex_rms_mm = std::max(
    0.0, declare_parameter<double>("basic_template_max_vertex_rms_mm", 3.0));
  config.basic_template_max_vertex_error_mm = std::max(
    0.0, declare_parameter<double>("basic_template_max_vertex_error_mm", 5.0));
  config.basic_template_piece_area_relative_tolerance = std::clamp(
    declare_parameter<double>("basic_template_piece_area_relative_tolerance", 0.18),
    0.0, 1.0);
  config.basic_template_area_weight = std::max(
    0.0, declare_parameter<double>("basic_template_area_weight", 10.0));
  config.basic_template_max_layout_hole_mm2 = std::max(
    0.0, declare_parameter<double>("basic_template_max_layout_hole_mm2", 100.0));
  config.basic_template_max_layout_outside_mm2 = std::max(
    0.0, declare_parameter<double>("basic_template_max_layout_outside_mm2", 100.0));
  config.basic_template_max_layout_overlap_mm2 = std::max(
    0.0, declare_parameter<double>("basic_template_max_layout_overlap_mm2", 30.0));
  config.edge_length_relative_tolerance = declare_parameter<double>("edge_length_relative_tolerance", 0.08);
  config.minimum_partial_edge_ratio = declare_parameter<double>("minimum_partial_edge_ratio", 0.25);
  config.endpoint_tolerance_mm = declare_parameter<double>("endpoint_tolerance_mm", 4.0);
  config.overlap_tolerance_mm2 = declare_parameter<double>("overlap_tolerance_mm2", 10.0);
  config.nominal_overlap_tolerance_mm2 = declare_parameter<double>(
    "nominal_overlap_tolerance_mm2", 0.0);
  config.contour_uncertainty_base_mm = declare_parameter<double>(
    "contour_uncertainty_base_mm", 1.5);
  config.contour_uncertainty_low_confidence_mm = declare_parameter<double>(
    "contour_uncertainty_low_confidence_mm", 2.0);
  config.raster_resolution_mm = declare_parameter<double>("raster_resolution_mm", 0.75);
  config.target_long_min_mm = declare_parameter<double>("target_long_min_mm", 90.0);
  config.target_long_max_mm = declare_parameter<double>("target_long_max_mm", 120.0);
  config.target_short_min_mm = declare_parameter<double>("target_short_min_mm", 50.0);
  config.target_short_max_mm = declare_parameter<double>("target_short_max_mm", 90.0);
  config.minimum_rectangularity = declare_parameter<double>("minimum_rectangularity", 0.93);
  config.minimum_aspect_ratio = declare_parameter<double>("minimum_aspect_ratio", 1.05);
  config.target_center_x_a4_mm = declare_parameter<double>("target_center_x_a4_mm", 105.0);
  config.target_center_y_a4_mm = declare_parameter<double>("target_center_y_a4_mm", 222.75);
  target_piece_gap_mm_ = std::max(
    0.0, declare_parameter<double>("target_piece_gap_mm", 0.2));
  config.target_piece_gap_mm = target_piece_gap_mm_;
  config.length_weight = declare_parameter<double>("length_weight", 20.0);
  config.partial_edge_weight = declare_parameter<double>("partial_edge_weight", 3.0);
  config.endpoint_weight = declare_parameter<double>("endpoint_weight", 2.0);
  config.overlap_weight = declare_parameter<double>("overlap_weight", 10.0);
  config.rectangularity_weight = declare_parameter<double>("rectangularity_weight", 300.0);
  config.pattern_enabled = declare_parameter<bool>("pattern_enabled", false);
  config.texture_weight = declare_parameter<double>("texture_weight", 20.0);
  config.texture_band_mm = declare_parameter<double>("texture_band_mm", 3.0);
  config.texture_samples_along_edge = declare_parameter<int>("texture_samples_along_edge", 24);
  config.texture_samples_across_edge = declare_parameter<int>("texture_samples_across_edge", 3);
  config.pattern_edge_priority_enabled = declare_parameter<bool>(
    "pattern_edge_priority_enabled", true);
  config.pattern_edge_min_strength = declare_parameter<double>(
    "pattern_edge_min_strength", 0.12);
  config.pattern_edge_priority_weight = declare_parameter<double>(
    "pattern_edge_priority_weight", 2.0);
  config.texture_color_cost_weight = declare_parameter<double>(
    "texture_color_cost_weight", 1.0);
  config.texture_gray_zncc_weight = declare_parameter<double>(
    "texture_gray_zncc_weight", 1.0);
  config.texture_gradient_zncc_weight = declare_parameter<double>(
    "texture_gradient_zncc_weight", 1.0);
  config.texture_ssim_weight = declare_parameter<double>(
    "texture_ssim_weight", 1.0);
  config.max_states_per_subset = declare_parameter<int>("max_states_per_subset", 80);
  basic_solver_ = std::make_unique<puzzle_solver::PuzzleSolverCore>(config);

  puzzle_solver::SolverConfig challenge_config = config;
  challenge_config.basic_task_template_enabled = false;
  challenge_config.target_long_min_mm = declare_parameter<double>(
    "challenge_target_long_min_mm", 90.0);
  challenge_config.target_long_max_mm = declare_parameter<double>(
    "challenge_target_long_max_mm", 120.0);
  challenge_config.target_short_min_mm = declare_parameter<double>(
    "challenge_target_short_min_mm", 50.0);
  challenge_config.target_short_max_mm = declare_parameter<double>(
    "challenge_target_short_max_mm", 90.0);
  challenge_config.minimum_aspect_ratio = declare_parameter<double>(
    "challenge_minimum_aspect_ratio", 1.0);
  challenge_config.free_target_pose_enabled = true;
  challenge_config.placement_frame_origin_x_mm = declare_parameter<double>(
    "challenge_placement_frame_origin_x_mm", 0.0);
  challenge_config.placement_frame_origin_y_mm = declare_parameter<double>(
    "challenge_placement_frame_origin_y_mm", 148.5);
  challenge_config.placement_frame_width_mm = declare_parameter<double>(
    "challenge_placement_frame_width_mm", 210.0);
  challenge_config.placement_frame_height_mm = declare_parameter<double>(
    "challenge_placement_frame_height_mm", 148.5);
  challenge_config.placement_frame_margin_mm = declare_parameter<double>(
    "challenge_placement_frame_margin_mm", 3.0);
  challenge_config.minimum_piece_edge_mm = declare_parameter<double>(
    "challenge_minimum_piece_edge_mm", 20.0);
  challenge_config.require_each_piece_boundary_edge = declare_parameter<bool>(
    "challenge_require_each_piece_boundary_edge", true);
  challenge_config.boundary_edge_tolerance_mm = declare_parameter<double>(
    "challenge_boundary_edge_tolerance_mm", 2.0);
  challenge_config.anchor_move_distance_weight = declare_parameter<double>(
    "challenge_anchor_move_distance_weight", 0.02);
  challenge_config.anchor_rotation_weight = declare_parameter<double>(
    "challenge_anchor_rotation_weight", 0.02);
  challenge_config.anchor_fallback_penalty = declare_parameter<double>(
    "challenge_anchor_fallback_penalty", 100.0);
  challenge_one_solver_ = std::make_unique<puzzle_solver::PuzzleSolverCore>(challenge_config);

  // Task 3 uses card white borders as outer-frame evidence; pattern settings
  // only solve rotation/same-shape ambiguity after geometry has a layout.
  puzzle_solver::SolverConfig challenge_two_config = challenge_config;
  challenge_two_config.target_long_min_mm = declare_parameter<double>(
    "challenge_two_target_long_min_mm", 90.0);
  challenge_two_config.target_long_max_mm = declare_parameter<double>(
    "challenge_two_target_long_max_mm", 120.0);
  challenge_two_config.target_short_min_mm = declare_parameter<double>(
    "challenge_two_target_short_min_mm", 50.0);
  challenge_two_config.target_short_max_mm = declare_parameter<double>(
    "challenge_two_target_short_max_mm", 90.0);
  challenge_two_config.minimum_aspect_ratio = declare_parameter<double>(
    "challenge_two_minimum_aspect_ratio", 1.0);
  challenge_two_config.minimum_piece_edge_mm = declare_parameter<double>(
    "challenge_two_minimum_boundary_edge_mm", 5.0);
  challenge_two_config.require_each_piece_boundary_edge = declare_parameter<bool>(
    "challenge_two_require_each_piece_boundary_edge", true);
  challenge_two_config.pattern_enabled = declare_parameter<bool>(
    "challenge_two_pattern_enabled", true);
  challenge_two_config.pattern_edge_priority_enabled = declare_parameter<bool>(
    "challenge_two_pattern_edge_priority_enabled", true);
  challenge_two_config.pattern_edge_min_strength = declare_parameter<double>(
    "challenge_two_pattern_edge_min_strength", config.pattern_edge_min_strength);
  challenge_two_config.pattern_edge_priority_weight = declare_parameter<double>(
    "challenge_two_pattern_edge_priority_weight", config.pattern_edge_priority_weight);
  challenge_two_config.white_border_fast_solver_enabled = declare_parameter<bool>(
    "challenge_two_white_border_fast_solver_enabled", true);
  challenge_two_config.white_border_min_confidence = declare_parameter<double>(
    "challenge_two_white_border_min_confidence", 0.72);
  challenge_two_config.white_border_band_mm = declare_parameter<double>(
    "challenge_two_white_border_band_mm", 2.5);
  challenge_two_config.white_border_max_chroma = declare_parameter<double>(
    "challenge_two_white_border_max_chroma", 45.0);
  challenge_two_config.white_border_min_luma = declare_parameter<double>(
    "challenge_two_white_border_min_luma", 178.0);
  challenge_two_config.white_border_layout_weight = declare_parameter<double>(
    "challenge_two_white_border_layout_weight", 80.0);
  challenge_two_config.seam_gradient_min_continuity = declare_parameter<double>(
    "challenge_two_seam_gradient_min_continuity", 0.20);
  challenge_two_config.center_symmetry_pass_threshold = declare_parameter<double>(
    "challenge_two_center_symmetry_pass_threshold", 0.50);
  challenge_two_config.center_symmetry_direct_threshold = declare_parameter<double>(
    "challenge_two_center_symmetry_direct_threshold", 0.90);
  challenge_two_config.center_symmetry_weight = declare_parameter<double>(
    "challenge_two_center_symmetry_weight", 35.0);
  challenge_two_config.diagonal_specialness_weight = declare_parameter<double>(
    "challenge_two_diagonal_specialness_weight", 8.0);
  challenge_two_config.non_character_corner_white_pass_threshold = declare_parameter<double>(
    "challenge_two_non_character_corner_white_pass_threshold", 0.60);
  challenge_two_config.non_character_corner_white_weight = declare_parameter<double>(
    "challenge_two_non_character_corner_white_weight", 25.0);
  challenge_two_config.card_corner_sample_mm = declare_parameter<double>(
    "challenge_two_card_corner_sample_mm", 10.0);
  const double challenge_two_maximum_solution_score = declare_parameter<double>(
    "challenge_two_maximum_solution_score", 0.0);
  challenge_two_config.maximum_solution_score = challenge_two_maximum_solution_score > 0.0 ?
    challenge_two_maximum_solution_score : std::numeric_limits<double>::infinity();
  challenge_two_config.max_states_per_subset = declare_parameter<int>(
    "challenge_two_max_states_per_subset", 300);
  challenge_two_solver_ = std::make_unique<puzzle_solver::PuzzleSolverCore>(challenge_two_config);

  plan_publisher_ = create_publisher<vision_interfaces::msg::PuzzlePlan>(plan_topic_, 1);
  status_publisher_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);
  debug_image_publisher_ = create_publisher<sensor_msgs::msg::Image>(
    debug_image_topic_, rclcpp::SensorDataQoS());
  scene_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  task_session_callback_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions scene_options;
  scene_options.callback_group = scene_callback_group_;
  scene_subscription_ = create_subscription<vision_interfaces::msg::PuzzleScene>(
    scene_topic_, 1, std::bind(&PuzzleSolverNode::on_scene, this, std::placeholders::_1),
    scene_options);
  rclcpp::SubscriptionOptions session_options;
  session_options.callback_group = task_session_callback_group_;
  task_session_subscription_ = create_subscription<vision_interfaces::msg::TaskSession>(
    task_session_topic_, 10,
    std::bind(&PuzzleSolverNode::on_task_session, this, std::placeholders::_1),
    session_options);
}

void PuzzleSolverNode::on_task_session(
  const vision_interfaces::msg::TaskSession::ConstSharedPtr message)
{
  if (message->generation < active_generation_.load()) {
    return;
  }
  active_task_.store(static_cast<int>(message->task_id));
  active_generation_.store(message->generation);
}

void PuzzleSolverNode::on_scene(const vision_interfaces::msg::PuzzleScene::ConstSharedPtr message)
{
  const auto started = std::chrono::steady_clock::now();
  const int task_id = static_cast<int>(message->task_id);
  const std::uint32_t generation = message->generation;
  const auto active_generation = active_generation_.load();
  const auto active_task = active_task_.load();
  if (generation < active_generation ||
    (generation == active_generation && task_id != active_task))
  {
    // BUG_POINT:TASK_SESSION_SOLVER - Interrupted-task scenes never consume the
    // current solver state or produce a plan for the coordinator.
    RCLCPP_WARN(
      get_logger(), "BUG_POINT:TASK_SESSION_SOLVER ignored task=%d generation=%u; "
      "active_task=%d active_generation=%u",
      task_id, generation, active_task, active_generation);
    return;
  }
  if (generation > active_generation) {
    // Cross-topic DDS delivery may deliver ScanRequest/PuzzleScene before TaskSession.
    active_task_.store(task_id);
    active_generation_.store(generation);
  }
  vision_interfaces::msg::PuzzlePlan plan;
  plan.header = message->header;
  plan.task_id = message->task_id;
  plan.generation = generation;
  if (!message->valid) {
    plan.status = "SCENE_INVALID:" + message->status;
    plan_publisher_->publish(plan);
    publish_status(plan.status, 0.0, message->pieces.size(), task_id, generation);
    return;
  }
  if (require_workspace_mapping_ && !message->workspace_mapping_valid) {
    // BUG_POINT:ABSOLUTE_COORDINATE_INVALID - Local A4 coordinates must not reach the controller as absolute.
    plan.status = "WORKSPACE_MAPPING_REQUIRED";
    plan_publisher_->publish(plan);
    publish_status(plan.status, 0.0, message->pieces.size(), task_id, generation);
    return;
  }

  std::vector<puzzle_solver::PieceModel> pieces;
  pieces.reserve(message->pieces.size());
  for (const auto & piece_message : message->pieces) {
    if (piece_message.polygon_a4_mm.size() < 3) {
      continue;
    }
    puzzle_solver::PieceModel piece;
    piece.id = piece_message.id;
    piece.source_center_a4_mm = cv::Point2d(
      piece_message.center_a4_mm.x, piece_message.center_a4_mm.y);
    piece.pick_point_a4_mm = cv::Point2d(
      piece_message.pick_point_a4_mm.x, piece_message.pick_point_a4_mm.y);
    piece.source_angle_deg = piece_message.orientation_deg;
    piece.area_mm2 = piece_message.area_mm2;
    piece.confidence = piece_message.confidence;
    for (const auto & vertex : piece_message.polygon_a4_mm) {
      piece.polygon_local_mm.emplace_back(
        vertex.x - piece.source_center_a4_mm.x,
        vertex.y - piece.source_center_a4_mm.y);
    }
    pieces.push_back(std::move(piece));
  }

  cv::Mat rectified;
  if (!from_ros_image(message->rectified_image, rectified)) {
    RCLCPP_WARN(
      get_logger(), "BUG_POINT:SCENE_IMAGE invalid encoding=%s size=%ux%u step=%u",
      message->rectified_image.encoding.c_str(), message->rectified_image.width,
      message->rectified_image.height, message->rectified_image.step);
  }
  if (task_id < 1 || task_id > 3) {
    plan.status = "TASK_" + std::to_string(task_id) + "_NOT_IMPLEMENTED";
    plan_publisher_->publish(plan);
    publish_status(plan.status, 0.0, pieces.size(), task_id, generation);
    return;
  }
  const auto & solver = task_id == 1 ? basic_solver_ :
    (task_id == 2 ? challenge_one_solver_ : challenge_two_solver_);
  const auto result = solver->solve(
    pieces, rectified, pixels_per_mm_,
    cv::Point2d(
      message->rectified_origin_a4_mm.x,
      message->rectified_origin_a4_mm.y));
  plan.solved = result.solved;
  plan.status = result.status;
  plan.target_width_mm = static_cast<float>(result.target_width_mm);
  plan.target_height_mm = static_cast<float>(result.target_height_mm);
  plan.score = static_cast<float>(result.score);
  plan.score_margin = static_cast<float>(result.second_score - result.score);
  const double required_score_margin = task_id == 2 ? challenge_minimum_score_margin_ :
    (task_id == 3 ? challenge_two_minimum_score_margin_ : minimum_score_margin_);
  if (plan.solved && plan.score_margin < required_score_margin) {
    plan.solved = false;
    plan.status = "AMBIGUOUS_LAYOUT";
  }

  const bool has_target_pose = result.target_pose_by_piece.size() == pieces.size();
  if (has_target_pose) {
    for (std::size_t index = 0; index < pieces.size(); ++index) {
      const auto & source = pieces[index];
      const auto & target_transform = result.target_pose_by_piece[index];
      vision_interfaces::msg::PuzzlePlacement placement;
      placement.task_id = message->task_id;
      placement.generation = generation;
      placement.piece_id = source.id;
      placement.workspace_mapping_valid = message->workspace_mapping_valid;
      placement.source_center_a4_mm = to_point32(source.source_center_a4_mm);
      const cv::Point2d target_center = target_transform.apply(cv::Point2d(0.0, 0.0));
      placement.target_center_a4_mm = to_point32(target_center);
      if (message->workspace_mapping_valid) {
        placement.source_center_workspace_mm = to_point32(puzzle_geometry::apply_homography(
          message->a4_to_workspace_homography,
          puzzle_geometry::Point2d{source.source_center_a4_mm.x, source.source_center_a4_mm.y}));
        placement.target_center_workspace_mm = to_point32(puzzle_geometry::apply_homography(
          message->a4_to_workspace_homography,
          puzzle_geometry::Point2d{target_center.x, target_center.y}));
        placement.pick_point_workspace_mm = to_point32(puzzle_geometry::apply_homography(
          message->a4_to_workspace_homography,
          puzzle_geometry::Point2d{source.pick_point_a4_mm.x, source.pick_point_a4_mm.y}));
      }
      try {
        const auto source_image = puzzle_geometry::apply_homography(
          message->a4_to_image_homography,
          puzzle_geometry::Point2d{source.source_center_a4_mm.x, source.source_center_a4_mm.y});
        const auto target_image = puzzle_geometry::apply_homography(
          message->a4_to_image_homography,
          puzzle_geometry::Point2d{target_center.x, target_center.y});
        placement.source_center_camera_px = to_point32(puzzle_geometry::camera_centered_pixel(
          source_image, message->image_width, message->image_height, camera_y_axis_up_));
        placement.target_center_camera_px = to_point32(puzzle_geometry::camera_centered_pixel(
          target_image, message->image_width, message->image_height, camera_y_axis_up_));
      } catch (const std::exception & exception) {
        // BUG_POINT:CAMERA_COORDINATE - The five-field command must never contain a guessed point.
        RCLCPP_ERROR(get_logger(), "BUG_POINT:CAMERA_COORDINATE %s", exception.what());
        plan.solved = false;
        plan.status = "CAMERA_COORDINATE_INVALID";
        plan.placements.clear();
        break;
      }
      placement.source_angle_deg = static_cast<float>(source.source_angle_deg);
      std::vector<puzzle_geometry::Point2d> target_points;
      for (const auto & local_point : source.polygon_local_mm) {
        const cv::Point2d target_point = target_transform.apply(local_point);
        placement.target_polygon_a4_mm.push_back(to_point32(target_point));
        if (message->workspace_mapping_valid) {
          placement.target_polygon_workspace_mm.push_back(to_point32(
            puzzle_geometry::apply_homography(
              message->a4_to_workspace_homography,
              puzzle_geometry::Point2d{target_point.x, target_point.y})));
        }
        target_points.push_back(puzzle_geometry::Point2d{target_point.x, target_point.y});
      }
      placement.target_angle_deg = static_cast<float>(
        puzzle_geometry::dominant_edge_angle_deg(target_points));
      placement.rotation_delta_deg = static_cast<float>(
        puzzle_geometry::image_rotation_rad_to_ccw_delta_deg(target_transform.angle_rad));
      const double position_error = cv::norm(target_center - source.source_center_a4_mm);
      const double angle_error = std::abs(placement.rotation_delta_deg);
      placement.already_placed =
        position_error <= placed_position_tolerance_mm_ &&
        angle_error <= placed_angle_tolerance_deg_;
      placement.placement_score = static_cast<float>(result.score);
      plan.placements.push_back(std::move(placement));
    }
    std::stable_sort(plan.placements.begin(), plan.placements.end(),
      [](const auto & first, const auto & second) {
        if (first.already_placed != second.already_placed) {
          return !first.already_placed;
        }
        return first.piece_id < second.piece_id;
      });
  }

  if (generation != active_generation_.load() || task_id != active_task_.load()) {
    // BUG_POINT:TASK_SESSION_SOLVE_CANCEL - The solver core is not preemptible,
    // but an interrupted result is discarded before any observable publication.
    RCLCPP_WARN(
      get_logger(), "BUG_POINT:TASK_SESSION_SOLVE_CANCEL discarded task=%d generation=%u",
      task_id, generation);
    return;
  }
  if (debug_mode_) {
    publish_debug_plan(*message, plan, rectified);
  }

  plan_publisher_->publish(plan);
  const double solve_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  publish_status(plan.status, solve_ms, pieces.size(), task_id, generation);
}

void PuzzleSolverNode::publish_debug_plan(
  const vision_interfaces::msg::PuzzleScene & scene,
  const vision_interfaces::msg::PuzzlePlan & plan,
  const cv::Mat & rectified) const
{
  if (rectified.empty() || rectified.type() != CV_8UC3) {
    return;
  }
  cv::Mat debug = rectified.clone();
  const cv::Point2d origin(
    scene.rectified_origin_a4_mm.x, scene.rectified_origin_a4_mm.y);
  const auto to_pixel = [&](const geometry_msgs::msg::Point32 & point) {
      return cv::Point(
        cvRound((point.x - origin.x) * pixels_per_mm_),
        cvRound((point.y - origin.y) * pixels_per_mm_));
    };
  const auto a4_pixel = [&](const double x, const double y) {
      return cv::Point(
        cvRound((x - origin.x) * pixels_per_mm_),
        cvRound((y - origin.y) * pixels_per_mm_));
    };
  const std::vector<cv::Point> a4{
    a4_pixel(0.0, 0.0), a4_pixel(210.0, 0.0),
    a4_pixel(210.0, 297.0), a4_pixel(0.0, 297.0)};
  cv::polylines(debug, a4, true, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
  for (const auto & piece : scene.pieces) {
    std::vector<cv::Point> polygon;
    for (const auto & vertex : piece.polygon_a4_mm) {
      polygon.push_back(to_pixel(vertex));
    }
    if (polygon.size() >= 3U) {
      cv::polylines(debug, polygon, true, cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
    }
  }
  std::vector<cv::Point> moved_target_points;
  for (const auto & placement : plan.placements) {
    std::vector<cv::Point> target;
    for (const auto & vertex : placement.target_polygon_a4_mm) {
      target.push_back(to_pixel(vertex));
    }
    if (target.size() >= 3U) {
      moved_target_points.insert(moved_target_points.end(), target.begin(), target.end());
      cv::polylines(debug, target, true, cv::Scalar(0, 255, 255), 3, cv::LINE_AA);
    }
    const cv::Point center = to_pixel(placement.target_center_a4_mm);
    cv::drawMarker(
      debug, center, cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 18, 2, cv::LINE_AA);
    std::ostringstream label;
    label << "P" << placement.piece_id << " d=" <<
      std::fixed << std::setprecision(1) << placement.rotation_delta_deg << "deg";
    cv::putText(
      debug, label.str(), center + cv::Point(8, -8), cv::FONT_HERSHEY_SIMPLEX,
      0.55, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
  }
  if (moved_target_points.size() >= 3U) {
    std::vector<cv::Point> hull;
    cv::convexHull(moved_target_points, hull);
    if (hull.size() >= 3U) {
      cv::polylines(debug, hull, true, cv::Scalar(255, 255, 0), 4, cv::LINE_AA);
    }
    const cv::RotatedRect moved_rect = cv::minAreaRect(moved_target_points);
    cv::Point2f rect_corners[4];
    moved_rect.points(rect_corners);
    std::vector<cv::Point> rect_polygon;
    rect_polygon.reserve(4U);
    for (const auto & point : rect_corners) {
      rect_polygon.emplace_back(cvRound(point.x), cvRound(point.y));
    }
    cv::polylines(debug, rect_polygon, true, cv::Scalar(255, 128, 0), 3, cv::LINE_AA);
    std::ostringstream rect_label;
    rect_label << "MOVED OUTLINE / RECT " << std::fixed << std::setprecision(1) <<
      plan.target_width_mm << "x" << plan.target_height_mm << "mm";
    cv::putText(
      debug, rect_label.str(), cv::Point(16, 58), cv::FONT_HERSHEY_SIMPLEX,
      0.65, cv::Scalar(255, 255, 0), 2, cv::LINE_AA);
  }
  std::ostringstream summary;
  summary << plan.status << " score=" << std::fixed << std::setprecision(2) << plan.score <<
    " margin=" << plan.score_margin << " gap=" << target_piece_gap_mm_ << "mm";
  cv::putText(
    debug, summary.str(), cv::Point(16, 30), cv::FONT_HERSHEY_SIMPLEX,
    0.65, plan.solved ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
  debug_image_publisher_->publish(to_bgr8_image(scene.header, debug));
}

void PuzzleSolverNode::publish_status(
  const std::string & status, const double solve_ms, const std::size_t piece_count,
  const int task_id, const std::uint32_t generation)
{
  std_msgs::msg::String message;
  std::ostringstream stream;
  stream << "{\"status\":\"" << status << "\",\"active_task\":" << task_id <<
    ",\"generation\":" << generation <<
    ",\"piece_count\":" << piece_count <<
    ",\"solve_ms\":" << solve_ms << "}";
  message.data = stream.str();
  status_publisher_->publish(message);
}
