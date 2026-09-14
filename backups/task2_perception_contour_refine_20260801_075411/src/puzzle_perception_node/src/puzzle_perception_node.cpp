#include "puzzle_perception_node/puzzle_perception_node.hpp"
#include "puzzle_perception_node/green_a4_detector.hpp"
#include "puzzle_perception_node/hmi_exposure.hpp"
#include "puzzle_perception_node/hsv_object_mask.hpp"
#include "puzzle_perception_node/lab_background_mask.hpp"
#include "puzzle_perception_node/piece_foreground_mask.hpp"
#include "puzzle_perception_node/polygon_refinement.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <geometry_msgs/msg/point32.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <puzzle_geometry/planar_geometry.hpp>

namespace
{
constexpr double kA4WidthMm = 210.0;
constexpr double kA4HeightMm = 297.0;

int make_odd(const int value)
{
  const int positive = std::max(1, value);
  return positive % 2 == 0 ? positive + 1 : positive;
}

puzzle_perception_node::HsvRange declare_hsv_range(
  rclcpp::Node & node, const std::string & prefix,
  const puzzle_perception_node::HsvRange & defaults)
{
  const auto declare_component = [&node, &prefix](
    const std::string & suffix, const int default_value) {
      return static_cast<int>(
        node.declare_parameter<int>(prefix + suffix, default_value));
    };
  return puzzle_perception_node::HsvRange{
    declare_component("_h_min", defaults.h_min),
    declare_component("_h_max", defaults.h_max),
    declare_component("_s_min", defaults.s_min),
    declare_component("_s_max", defaults.s_max),
    declare_component("_v_min", defaults.v_min),
    declare_component("_v_max", defaults.v_max)};
}

void validate_hsv_configuration(
  const puzzle_perception_node::HsvRange & green,
  const puzzle_perception_node::HsvRange & white,
  const bool remove_magnet,
  const puzzle_perception_node::HsvRange & magnet)
{
  if (!puzzle_perception_node::valid_hsv_range(green) ||
    !puzzle_perception_node::valid_hsv_range(white) ||
    (remove_magnet && !puzzle_perception_node::valid_hsv_range(magnet)))
  {
    throw std::invalid_argument(
            "BUG_POINT:HSV_RANGE object HSV bounds must use OpenCV H=0..179, S/V=0..255");
  }
  if (puzzle_perception_node::hsv_ranges_overlap(green, white)) {
    throw std::invalid_argument(
            "BUG_POINT:HSV_OBJECT_OVERLAP green-paper and white-piece HSV ranges must not overlap");
  }
  if (remove_magnet && puzzle_perception_node::hsv_ranges_overlap(white, magnet)) {
    throw std::invalid_argument(
            "BUG_POINT:HSV_EXCLUSION_OVERLAP white-piece and magnet HSV ranges must not overlap");
  }
}

geometry_msgs::msg::Point32 to_point32(const cv::Point2f & point)
{
  geometry_msgs::msg::Point32 output;
  output.x = point.x;
  output.y = point.y;
  output.z = 0.0F;
  return output;
}

std::array<cv::Point2f, 4> estimated_a4_corners_for_debug(
  const cv::Size & image_size, const std::vector<double> & fixed_a4_corners_px)
{
  if (fixed_a4_corners_px.size() == 8U &&
    std::all_of(fixed_a4_corners_px.begin(), fixed_a4_corners_px.end(), [](double value) {
      return value >= 0.0;
    }))
  {
    return {
      cv::Point2f(
        static_cast<float>(fixed_a4_corners_px[0]), static_cast<float>(fixed_a4_corners_px[1])),
      cv::Point2f(
        static_cast<float>(fixed_a4_corners_px[2]), static_cast<float>(fixed_a4_corners_px[3])),
      cv::Point2f(
        static_cast<float>(fixed_a4_corners_px[4]), static_cast<float>(fixed_a4_corners_px[5])),
      cv::Point2f(
        static_cast<float>(fixed_a4_corners_px[6]), static_cast<float>(fixed_a4_corners_px[7]))};
  }

  const double height = std::max(1.0, std::min(
      0.82 * static_cast<double>(image_size.height),
      0.82 * static_cast<double>(image_size.width) * kA4HeightMm / kA4WidthMm));
  const double width = height * kA4WidthMm / kA4HeightMm;
  const double left = 0.5 * (static_cast<double>(image_size.width) - width);
  const double top = 0.5 * (static_cast<double>(image_size.height) - height);
  return {
    cv::Point2f(static_cast<float>(left), static_cast<float>(top)),
    cv::Point2f(static_cast<float>(left + width), static_cast<float>(top)),
    cv::Point2f(static_cast<float>(left + width), static_cast<float>(top + height)),
    cv::Point2f(static_cast<float>(left), static_cast<float>(top + height))};
}

void draw_estimated_basic_target_debug(
  cv::Mat & debug_image, const std::vector<double> & fixed_a4_corners_px,
  const int task_id)
{
  if (debug_image.empty() || task_id != 1) {
    return;
  }
  const auto estimated_a4 = estimated_a4_corners_for_debug(debug_image.size(), fixed_a4_corners_px);
  const std::array<cv::Point2f, 4> local_a4 = {
    cv::Point2f(0.0F, 0.0F), cv::Point2f(static_cast<float>(kA4WidthMm), 0.0F),
    cv::Point2f(static_cast<float>(kA4WidthMm), static_cast<float>(kA4HeightMm)),
    cv::Point2f(0.0F, static_cast<float>(kA4HeightMm))};
  const cv::Mat a4_to_image = cv::getPerspectiveTransform(local_a4.data(), estimated_a4.data());
  const double left = 0.5 * (kA4WidthMm - 100.0);
  const double top = 0.75 * kA4HeightMm - 30.0;
  const std::vector<cv::Point2f> target_a4{
    cv::Point2f(static_cast<float>(left), static_cast<float>(top)),
    cv::Point2f(static_cast<float>(left + 100.0), static_cast<float>(top)),
    cv::Point2f(static_cast<float>(left + 100.0), static_cast<float>(top + 60.0)),
    cv::Point2f(static_cast<float>(left), static_cast<float>(top + 60.0))};
  std::vector<cv::Point2f> target_image;
  cv::perspectiveTransform(target_a4, target_image, a4_to_image);
  std::vector<cv::Point> a4_polygon;
  std::vector<cv::Point> target_polygon;
  a4_polygon.reserve(estimated_a4.size());
  target_polygon.reserve(target_image.size());
  for (const auto & point : estimated_a4) {
    a4_polygon.emplace_back(cvRound(point.x), cvRound(point.y));
  }
  for (const auto & point : target_image) {
    target_polygon.emplace_back(cvRound(point.x), cvRound(point.y));
  }
  cv::polylines(debug_image, a4_polygon, true, cv::Scalar(0, 180, 255), 2, cv::LINE_AA);
  cv::polylines(debug_image, target_polygon, true, cv::Scalar(0, 255, 255), 4, cv::LINE_AA);
  cv::putText(
    debug_image, "EST BASIC TARGET 100x60 - A4 NOT FOUND",
    target_polygon.front() + cv::Point(8, -10), cv::FONT_HERSHEY_SIMPLEX,
    0.65, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
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

sensor_msgs::msg::Image to_yuyv_image(
  const std_msgs::msg::Header & header, const cv::Mat & image)
{
  sensor_msgs::msg::Image output;
  output.header = header;
  if (image.empty() || image.type() != CV_8UC2) {
    return output;
  }
  const cv::Mat contiguous = image.isContinuous() ? image : image.clone();
  output.height = static_cast<std::uint32_t>(contiguous.rows);
  output.width = static_cast<std::uint32_t>(contiguous.cols);
  output.encoding = "yuyv";
  output.is_bigendian = false;
  output.step = static_cast<sensor_msgs::msg::Image::_step_type>(contiguous.cols * 2);
  const auto byte_count = contiguous.total() * contiguous.elemSize();
  output.data.assign(contiguous.data, contiguous.data + byte_count);
  return output;
}
}  // namespace

PuzzlePerceptionNode::PuzzlePerceptionNode()
: Node("puzzle_perception_node"),
  scan_pending_(false),
  collected_frames_(0),
  best_sharpness_(-1.0),
  scan_sequence_(0)
{
  scan_request_topic_ = declare_parameter<std::string>("scan_request_topic", "puzzle/scan_request");
  task_session_topic_ = declare_parameter<std::string>(
    "task_session_topic", "puzzle/task_session");
  scene_topic_ = declare_parameter<std::string>("scene_topic", "puzzle/scene");
  debug_image_topic_ = declare_parameter<std::string>("debug_image_topic", "puzzle/image_debug");
  status_topic_ = declare_parameter<std::string>("status_topic", "puzzle/perception_status");
  hmi_command_topic_ = declare_parameter<std::string>(
    "hmi_command_topic", "vision/hmi/command");
  camera_pipeline_ = declare_parameter<std::string>("camera_pipeline", "");
  frame_id_ = declare_parameter<std::string>("frame_id", "workspace_camera");
  camera_index_ = declare_parameter<int>("camera_index", 0);
  input_width_ = declare_parameter<int>("input_width", 1280);
  input_height_ = declare_parameter<int>("input_height", 720);
  capture_fps_ = declare_parameter<double>("capture_fps", 30.0);
  pixels_per_mm_ = declare_parameter<double>("pixels_per_mm", 3.0);
  stable_frames_ = std::max(1, static_cast<int>(declare_parameter<int>("stable_frames", 3)));
  publish_debug_image_ = declare_parameter<bool>("publish_debug_image", false);
  debug_mode_ = declare_parameter<bool>("debug_mode", false);
  continuous_task_debug_detection_ = declare_parameter<bool>(
    "continuous_task_debug_detection", false);
  workspace_mapping_valid_ = declare_parameter<bool>("workspace_mapping_valid", false);
  fixed_a4_corners_px_ = declare_parameter<std::vector<double>>(
    "fixed_a4_corners_px", std::vector<double>(8, -1.0));
  workspace_homography_values_ = declare_parameter<std::vector<double>>(
    "workspace_homography_px_to_mm", {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
  a4_min_area_ratio_ = declare_parameter<double>("a4_min_area_ratio", 0.25);
  a4_max_area_ratio_ = declare_parameter<double>("a4_max_area_ratio", 0.98);
  green_frame_min_short_long_ratio_ = std::clamp(
    declare_parameter<double>("green_frame_min_short_long_ratio", 0.30), 0.0, 1.0);
  require_green_a4_ = declare_parameter<bool>("require_green_a4", true);
  green_a4_hsv_ = declare_hsv_range(
    *this, "green_a4", puzzle_perception_node::HsvRange{60, 90, 78, 201, 91, 245});
  green_a4_morph_kernel_ = make_odd(declare_parameter<int>("green_a4_morph_kernel", 7));
  green_a4_morph_iterations_ = std::max(
    0, static_cast<int>(declare_parameter<int>("green_a4_morph_iterations", 2)));
  a4_polygon_epsilon_ratio_ = declare_parameter<double>("a4_polygon_epsilon_ratio", 0.02);
  green_a4_min_border_coverage_ = declare_parameter<double>(
    "green_a4_min_border_coverage", 0.35);
  green_a4_line_fit_band_px_ = std::max(
    0.5, declare_parameter<double>("green_a4_line_fit_band_px", 6.0));
  green_a4_line_fit_min_points_ = std::max(
    2, static_cast<int>(declare_parameter<int>("green_a4_line_fit_min_points", 12)));
  green_a4_max_corner_refine_px_ = std::max(
    1.0, declare_parameter<double>("green_a4_max_corner_refine_px", 25.0));
  white_piece_hsv_ = declare_hsv_range(
    *this, "white_piece", puzzle_perception_node::HsvRange{0, 179, 0, 77, 160, 255});
  const bool legacy_basic_background_inversion = declare_parameter<bool>(
    "basic_use_a4_background_inversion", true);
  plain_piece_use_a4_background_inversion_ = declare_parameter<bool>(
    "plain_piece_use_a4_background_inversion", legacy_basic_background_inversion);
  const double legacy_basic_background_distance_threshold = declare_parameter<double>(
    "basic_background_distance_threshold", 12.0);
  plain_piece_background_distance_threshold_ = declare_parameter<double>(
    "plain_piece_background_distance_threshold", legacy_basic_background_distance_threshold);
  const double legacy_basic_background_max_mad = declare_parameter<double>(
    "basic_background_max_mad", 8.0);
  plain_piece_background_max_mad_ = declare_parameter<double>(
    "plain_piece_background_max_mad", legacy_basic_background_max_mad);
  const int legacy_basic_background_sample_stride = declare_parameter<int>(
    "basic_background_sample_stride", 4);
  plain_piece_background_sample_stride_ = declare_parameter<int>(
    "plain_piece_background_sample_stride", legacy_basic_background_sample_stride);
  const bool legacy_basic_background_chroma_only = declare_parameter<bool>(
    "basic_background_chroma_only", true);
  plain_piece_background_chroma_only_ = declare_parameter<bool>(
    "plain_piece_background_chroma_only", legacy_basic_background_chroma_only);
  challenge_piece_use_canny_ = declare_parameter<bool>("challenge_piece_use_canny", true);
  challenge_piece_use_lab_fill_ = declare_parameter<bool>("challenge_piece_use_lab_fill", true);
  canny_low_threshold_ = std::max(
    1.0, declare_parameter<double>("canny_low_threshold", 40.0));
  canny_high_threshold_ = std::max(
    canny_low_threshold_ + 1.0, declare_parameter<double>("canny_high_threshold", 120.0));
  canny_blur_kernel_ = make_odd(declare_parameter<int>("canny_blur_kernel", 3));
  const int configured_canny_aperture = make_odd(declare_parameter<int>("canny_aperture_size", 3));
  canny_aperture_size_ = std::clamp(configured_canny_aperture, 3, 7);
  canny_l2_gradient_ = declare_parameter<bool>("canny_l2_gradient", false);
  canny_edge_dilate_iterations_ = std::max(
    0, static_cast<int>(declare_parameter<int>("canny_edge_dilate_iterations", 0)));
  canny_close_kernel_ = make_odd(declare_parameter<int>("canny_close_kernel", 3));
  canny_close_iterations_ = std::max(
    0, static_cast<int>(declare_parameter<int>("canny_close_iterations", 0)));
  canny_flood_fill_background_ = declare_parameter<bool>("canny_flood_fill_background", true);
  page_margin_mm_ = declare_parameter<double>("page_margin_mm", 3.0);
  region_split_ratio_ = std::clamp(
    declare_parameter<double>("region_split_ratio", 0.5), 0.1, 0.9);
  divider_exclusion_mm_ = declare_parameter<double>("divider_exclusion_mm", 4.0);
  morph_kernel_ = make_odd(declare_parameter<int>("morph_kernel", 3));
  morph_iterations_ = std::max(
    0, static_cast<int>(declare_parameter<int>("morph_iterations", 1)));
  min_piece_area_mm2_ = declare_parameter<double>("min_piece_area_mm2", 100.0);
  max_piece_area_mm2_ = declare_parameter<double>("max_piece_area_mm2", 12000.0);
  min_piece_edge_mm_ = declare_parameter<double>("min_piece_edge_mm", 20.0);
  basic_min_piece_edge_mm_ = std::max(
    0.0, declare_parameter<double>("basic_min_piece_edge_mm", 7.0));
  basic_contour_epsilon_max_mm_ = std::max(
    0.1, declare_parameter<double>("basic_contour_epsilon_max_mm", 3.0));
  polygon_epsilon_mm_ = declare_parameter<double>("polygon_epsilon_mm", 1.0);
  max_polygon_vertices_ = declare_parameter<int>("max_polygon_vertices", 5);
  max_piece_count_ = declare_parameter<int>("max_piece_count", 4);
  challenge_one_expected_piece_count_ = std::clamp(
    static_cast<int>(declare_parameter<int>("challenge_one_expected_piece_count", 0)),
    0, max_piece_count_);
  basic_template_piece_area_relative_tolerance_ = std::clamp(
    declare_parameter<double>("basic_template_piece_area_relative_tolerance", 0.18),
    0.0, 1.0);
  remove_magnet_color_ = declare_parameter<bool>("remove_magnet_color", true);
  magnet_hsv_ = declare_hsv_range(
    *this, "magnet", puzzle_perception_node::HsvRange{100, 140, 100, 255, 60, 255});
  camera_calibration_valid_ = declare_parameter<bool>("camera_calibration_valid", false);
  calibration_image_width_ = declare_parameter<int>("calibration_image_width", 0);
  calibration_image_height_ = declare_parameter<int>("calibration_image_height", 0);
  camera_matrix_values_ = declare_parameter<std::vector<double>>(
    "camera_matrix", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});
  distortion_coefficients_ = declare_parameter<std::vector<double>>(
    "distortion_coefficients", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});
  challenge_two_background_distance_threshold_ = declare_parameter<double>(
    "challenge_two_background_distance_threshold", 24.0);
  challenge_two_background_max_mad_ = declare_parameter<double>(
    "challenge_two_background_max_mad", 10.0);
  challenge_two_background_sample_stride_ = declare_parameter<int>(
    "challenge_two_background_sample_stride", 4);
  challenge_two_min_piece_edge_mm_ = declare_parameter<double>(
    "challenge_two_min_piece_edge_mm", 5.0);
  challenge_two_rectangle_detection_enabled_ = declare_parameter<bool>(
    "challenge_two_rectangle_detection_enabled", true);
  challenge_two_rectangle_min_area_mm2_ = declare_parameter<double>(
    "challenge_two_rectangle_min_area_mm2", 1000.0);
  challenge_two_rectangle_max_area_mm2_ = declare_parameter<double>(
    "challenge_two_rectangle_max_area_mm2", 6000.0);
  challenge_two_rectangle_min_rectangularity_ = std::clamp(
    declare_parameter<double>("challenge_two_rectangle_min_rectangularity", 0.55),
    0.0, 1.0);
  challenge_two_rectangle_max_aspect_ratio_ = std::max(
    1.0, declare_parameter<double>("challenge_two_rectangle_max_aspect_ratio", 3.2));
  contour_epsilon_min_mm_ = declare_parameter<double>("contour_epsilon_min_mm", 0.6);
  contour_epsilon_max_mm_ = declare_parameter<double>("contour_epsilon_max_mm", 2.0);
  contour_epsilon_steps_ = declare_parameter<int>("contour_epsilon_steps", 5);
  contour_min_points_per_edge_ = declare_parameter<int>("contour_min_points_per_edge", 8);
  contour_max_line_rms_mm_ = declare_parameter<double>("contour_max_line_rms_mm", 1.2);
  contour_max_line_residual_mm_ = declare_parameter<double>(
    "contour_max_line_residual_mm", 3.0);
  validate_hsv_configuration(
    green_a4_hsv_, white_piece_hsv_, remove_magnet_color_, magnet_hsv_);

  scene_publisher_ = create_publisher<vision_interfaces::msg::PuzzleScene>(scene_topic_, 1);
  // Debug frames are latest-only sensor data; reliable delivery of 1280x720 BGR
  // messages can block the 30 FPS capture timer behind stale preview frames.
  debug_image_publisher_ = create_publisher<sensor_msgs::msg::Image>(
    debug_image_topic_, rclcpp::SensorDataQoS());
  status_publisher_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);
  scan_subscription_ = create_subscription<vision_interfaces::msg::ScanRequest>(
    scan_request_topic_, 10,
    std::bind(&PuzzlePerceptionNode::on_scan_request, this, std::placeholders::_1));
  task_session_subscription_ = create_subscription<vision_interfaces::msg::TaskSession>(
    task_session_topic_, 10,
    std::bind(&PuzzlePerceptionNode::on_task_session, this, std::placeholders::_1));
  hmi_command_subscription_ = create_subscription<std_msgs::msg::String>(
    hmi_command_topic_, 10,
    std::bind(&PuzzlePerceptionNode::on_hmi_command, this, std::placeholders::_1));

  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, capture_fps_));
  capture_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&PuzzlePerceptionNode::on_capture_timer, this));

  if (publish_debug_image_ || debug_mode_) {
    debug_processing_thread_ = std::thread(&PuzzlePerceptionNode::debug_processing_loop, this);
  }

  publish_status("READY");
}

PuzzlePerceptionNode::~PuzzlePerceptionNode()
{
  {
    std::lock_guard<std::mutex> lock(debug_mutex_);
    debug_processing_stop_ = true;
  }
  debug_condition_.notify_all();
  if (debug_processing_thread_.joinable()) {
    debug_processing_thread_.join();
  }
  std::lock_guard<std::mutex> camera_lock(camera_mutex_);
  capture_.release();
}

void PuzzlePerceptionNode::on_task_session(
  const vision_interfaces::msg::TaskSession::ConstSharedPtr message)
{
  const auto current_generation = active_generation_.load();
  if (message->generation < current_generation) {
    return;
  }
  active_task_.store(static_cast<int>(message->task_id));
  active_generation_.store(message->generation);
  if (scan_pending_ &&
    (scan_task_ != static_cast<int>(message->task_id) ||
    scan_generation_ != message->generation))
  {
    // BUG_POINT:TASK_SESSION_SCAN_CANCEL - Discard buffered frames from the interrupted task.
    scan_pending_ = false;
    collected_frames_ = 0;
    best_frame_.release();
    best_sharpness_ = -1.0;
    publish_status("SCAN_CANCELLED_BY_TASK_SWITCH");
  }
}

void PuzzlePerceptionNode::on_scan_request(
  const vision_interfaces::msg::ScanRequest::ConstSharedPtr message)
{
  if (message->task_id < 1U || message->task_id > 3U || message->generation == 0U) {
    RCLCPP_WARN(
      get_logger(), "BUG_POINT:SCAN_SESSION rejected task=%u generation=%u",
      message->task_id, message->generation);
    return;
  }
  const auto current_generation = active_generation_.load();
  if (message->generation < current_generation) {
    RCLCPP_WARN(
      get_logger(), "BUG_POINT:SCAN_SESSION ignored stale task=%u generation=%u current=%u",
      message->task_id, message->generation, current_generation);
    return;
  }
  // The scan request is authoritative if DDS delivers it before TaskSession on another topic.
  active_task_.store(static_cast<int>(message->task_id));
  active_generation_.store(message->generation);
  scan_task_ = static_cast<int>(message->task_id);
  scan_generation_ = message->generation;
  scan_pending_ = true;
  collected_frames_ = 0;
  best_frame_.release();
  best_sharpness_ = -1.0;
  publish_status("SCAN_REQUESTED");
}

void PuzzlePerceptionNode::on_hmi_command(const std_msgs::msg::String::ConstSharedPtr message)
{
  const auto exposure_time = puzzle_perception_node::parse_hmi_flash_command(message->data);
  if (!exposure_time) {
    return;
  }
  reopen_camera_with_exposure(*exposure_time);
}

void PuzzlePerceptionNode::reopen_camera_with_exposure(const int exposure_time)
{
  const auto updated_pipeline = puzzle_perception_node::camera_pipeline_with_exposure(
    camera_pipeline_, exposure_time);
  if (!updated_pipeline) {
    // BUG_POINT:HMI_EXPOSURE_PIPELINE - Keep the current camera alive when the
    // configured pipeline has no unique exposure_time_absolute field.
    RCLCPP_ERROR(
      get_logger(), "BUG_POINT:HMI_EXPOSURE_PIPELINE rejected exposure=%d", exposure_time);
    publish_status("CAMERA_EXPOSURE_PIPELINE_INVALID");
    return;
  }

  const auto exposure_text = std::to_string(exposure_time);
  publish_status("CAMERA_EXPOSURE_REOPENING_" + exposure_text);
  bool applied = false;
  bool recovered_previous_pipeline = false;
  {
    // The current executable uses a single-threaded executor, but this lock keeps
    // capture read/release safe if its executor changes later.
    std::lock_guard<std::mutex> camera_lock(camera_mutex_);
    const std::string previous_pipeline = camera_pipeline_;
    camera_generation_.fetch_add(1U);
    capture_.release();
    camera_pipeline_ = *updated_pipeline;
    applied = open_camera_locked();
    if (!applied) {
      capture_.release();
      camera_pipeline_ = previous_pipeline;
      recovered_previous_pipeline = open_camera_locked();
    }
  }

  if (scan_pending_) {
    // BUG_POINT:HMI_EXPOSURE_SCAN_RESET - Never combine frames captured with two
    // exposure values in the same stability selection window.
    collected_frames_ = 0;
    best_frame_.release();
    best_sharpness_ = -1.0;
  }
  {
    std::lock_guard<std::mutex> debug_lock(debug_mutex_);
    pending_debug_frame_.release();
    latest_debug_frame_.release();
    debug_frame_ready_ = false;
  }

  if (applied) {
    RCLCPP_INFO(get_logger(), "camera reopened with exposure_time_absolute=%d", exposure_time);
    publish_status("CAMERA_EXPOSURE_APPLIED_" + exposure_text);
  } else if (recovered_previous_pipeline) {
    RCLCPP_ERROR(
      get_logger(),
      "BUG_POINT:HMI_EXPOSURE_REOPEN new exposure=%d failed; previous pipeline restored",
      exposure_time);
    publish_status("CAMERA_EXPOSURE_REOPEN_FAILED_RECOVERED_" + exposure_text);
  } else {
    RCLCPP_ERROR(
      get_logger(),
      "BUG_POINT:HMI_EXPOSURE_REOPEN exposure=%d and previous pipeline both failed",
      exposure_time);
    publish_status("CAMERA_EXPOSURE_REOPEN_FAILED_" + exposure_text);
  }
}

bool PuzzlePerceptionNode::open_camera_locked()
{
  if (capture_.isOpened()) {
    return true;
  }

  // BUG_POINT:CAMERA_OPEN - A wrong GStreamer pipeline or occupied device produces no scene.
  const bool opened = camera_pipeline_.empty() ?
    capture_.open(camera_index_, cv::CAP_V4L2) :
    capture_.open(camera_pipeline_, cv::CAP_GSTREAMER);
  if (!opened) {
    publish_status("CAMERA_OPEN_FAILED");
    return false;
  }
  if (camera_pipeline_.empty()) {
    capture_.set(cv::CAP_PROP_FRAME_WIDTH, input_width_);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, input_height_);
    capture_.set(cv::CAP_PROP_FPS, capture_fps_);
    capture_.set(cv::CAP_PROP_BUFFERSIZE, 1);
  }
  return true;
}

void PuzzlePerceptionNode::on_capture_timer()
{
  // Debug mode keeps the camera and OpenCV annotation stream alive between scans.
  // Work mode remains scan-triggered so the real-time path does not pay preview cost.
  if (!scan_pending_ && !(publish_debug_image_ || debug_mode_)) {
    return;
  }
  cv::Mat frame;
  std::uint64_t frame_camera_generation = 0U;
  {
    std::lock_guard<std::mutex> camera_lock(camera_mutex_);
    if (!open_camera_locked()) {
      return;
    }
    if (!capture_.read(frame) || frame.empty()) {
      // BUG_POINT:FRAME_READ - Device negotiation can succeed while frame delivery fails.
      publish_status("FRAME_READ_FAILED");
      capture_.release();
      return;
    }
    frame_camera_generation = camera_generation_.load();
  }

  const auto stamp = now();
  if (publish_debug_image_ || debug_mode_) {
    cv::Mat display_frame;
    {
      std::lock_guard<std::mutex> lock(debug_mutex_);
      display_frame = latest_debug_frame_.empty() ? frame.clone() : latest_debug_frame_.clone();
    }
    publish_debug_frame(display_frame, stamp);
    enqueue_debug_frame(frame, frame_camera_generation);
  }

  if (!scan_pending_) {
    return;
  }

  const double sharpness = sharpness_score(frame);
  if (sharpness > best_sharpness_) {
    best_frame_ = frame.clone();
    best_sharpness_ = sharpness;
  }
  ++collected_frames_;
  if (collected_frames_ < stable_frames_) {
    return;
  }

  scan_pending_ = false;
  auto scene = build_scene(
    best_frame_, stamp, scan_task_, scan_generation_, frame_camera_generation, true);
  scene_publisher_->publish(scene);
  publish_status(scene.status);
  ++scan_sequence_;
}

void PuzzlePerceptionNode::debug_processing_loop()
{
  while (true) {
    cv::Mat frame;
    std::uint64_t frame_camera_generation = 0U;
    {
      std::unique_lock<std::mutex> lock(debug_mutex_);
      debug_condition_.wait(lock, [this]() {
        return debug_processing_stop_ || debug_frame_ready_;
      });
      if (debug_processing_stop_ && !debug_frame_ready_) {
        return;
      }
      frame = pending_debug_frame_.clone();
      frame_camera_generation = pending_debug_camera_generation_;
      debug_frame_ready_ = false;
    }
    if (!frame.empty()) {
      if (frame.type() == CV_8UC2) {
        cv::Mat bgr;
        cv::cvtColor(frame, bgr, cv::COLOR_YUV2BGR_YUY2);
        frame = bgr;
      }
      const int task_id = active_task_.load();
      if (task_id != 0 && !continuous_task_debug_detection_) {
        continue;
      }
      (void)build_scene(
        frame, now(), task_id, active_generation_.load(),
        frame_camera_generation, false);
    }
  }
}

void PuzzlePerceptionNode::enqueue_debug_frame(
  const cv::Mat & frame, const std::uint64_t camera_generation)
{
  {
    std::lock_guard<std::mutex> lock(debug_mutex_);
    pending_debug_frame_ = frame.clone();
    pending_debug_camera_generation_ = camera_generation;
    debug_frame_ready_ = true;
  }
  debug_condition_.notify_one();
}

void PuzzlePerceptionNode::set_latest_debug_frame(
  const cv::Mat & frame, const std::uint64_t camera_generation)
{
  if (camera_generation != camera_generation_.load()) {
    return;
  }
  std::lock_guard<std::mutex> lock(debug_mutex_);
  if (camera_generation != camera_generation_.load()) {
    return;
  }
  latest_debug_frame_ = frame.clone();
}

void PuzzlePerceptionNode::publish_debug_frame(const cv::Mat & frame, const rclcpp::Time & stamp)
{
  std_msgs::msg::Header header;
  header.stamp = stamp;
  header.frame_id = frame_id_;
  if (frame.type() == CV_8UC2) {
    debug_image_publisher_->publish(to_yuyv_image(header, frame));
  } else {
    debug_image_publisher_->publish(to_bgr8_image(header, frame));
  }
}

vision_interfaces::msg::PuzzleScene PuzzlePerceptionNode::build_scene(
  cv::Mat frame, const rclcpp::Time & stamp, const int task_id,
  const std::uint32_t generation, const std::uint64_t camera_generation,
  const bool commit_session_progress)
{
  if (frame.type() == CV_8UC2) {
    cv::Mat bgr;
    cv::cvtColor(frame, bgr, cv::COLOR_YUV2BGR_YUY2);
    frame = bgr;
  }
  vision_interfaces::msg::PuzzleScene scene;
  scene.header.stamp = stamp;
  scene.header.frame_id = frame_id_;
  scene.task_id = static_cast<std::uint8_t>(task_id);
  scene.generation = generation;
  scene.a4_detected = false;
  scene.workspace_mapping_valid = workspace_mapping_valid_;
  if (task_id == 3) {
    cv::Mat undistorted;
    if (!undistort_for_challenge_two(frame, undistorted)) {
      // BUG_POINT:TASK3_CALIBRATION - Task 3 physical dimensions must fail closed
      // until a measured camera calibration is explicitly marked valid.
      scene.valid = false;
      scene.status = "CALIBRATION_REQUIRED";
      scene.image_width = static_cast<std::uint32_t>(std::max(0, frame.cols));
      scene.image_height = static_cast<std::uint32_t>(std::max(0, frame.rows));
      if (publish_debug_image_ || debug_mode_) {
        cv::Mat debug_image = frame.clone();
        cv::putText(
          debug_image, scene.status, cv::Point(24, 44), cv::FONT_HERSHEY_SIMPLEX,
          1.0, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        set_latest_debug_frame(debug_image, camera_generation);
      }
      return scene;
    }
    frame = std::move(undistorted);
  }
  scene.image_width = static_cast<std::uint32_t>(std::max(0, frame.cols));
  scene.image_height = static_cast<std::uint32_t>(std::max(0, frame.rows));

  std::array<cv::Point2f, 4> a4_corners;
  cv::Mat green_mask;
  if (!detect_a4_quad(frame, a4_corners, green_mask)) {
    scene.valid = false;
    scene.status = "GREEN_A4_NOT_FOUND";
    if (publish_debug_image_ || debug_mode_) {
      cv::Mat debug_image = frame.clone();
      if (!green_mask.empty()) {
        std::vector<std::vector<cv::Point>> green_contours;
        cv::findContours(
          green_mask.clone(), green_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::drawContours(debug_image, green_contours, -1, cv::Scalar(0, 255, 255), 2);
      }
      cv::putText(
        debug_image, "GREEN_A4_NOT_FOUND", cv::Point(24, 44), cv::FONT_HERSHEY_SIMPLEX,
        1.0, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
      draw_estimated_basic_target_debug(debug_image, fixed_a4_corners_px_, task_id);
        set_latest_debug_frame(debug_image, camera_generation);
    }
    return scene;
  }
  scene.a4_detected = true;

  cv::Point2f green_center(0.0F, 0.0F);
  for (const auto & corner : a4_corners) {
    green_center += corner;
  }
  green_center *= 0.25F;
  const auto centered_green = puzzle_geometry::camera_centered_pixel(
    puzzle_geometry::Point2d{green_center.x, green_center.y},
    static_cast<double>(scene.image_width), static_cast<double>(scene.image_height), false);
  scene.green_frame_center_camera_px = to_point32(cv::Point2f(
      static_cast<float>(centered_green.x), static_cast<float>(centered_green.y)));

  const std::array<cv::Point2f, 4> local_a4 = {
    cv::Point2f(0.0F, 0.0F), cv::Point2f(210.0F, 0.0F),
    cv::Point2f(210.0F, 297.0F), cv::Point2f(0.0F, 297.0F)};
  const cv::Mat a4_to_image = cv::getPerspectiveTransform(local_a4.data(), a4_corners.data());
  const cv::Mat image_to_a4 = cv::getPerspectiveTransform(a4_corners.data(), local_a4.data());
  cv::Mat analysis_image;
  cv::Mat valid_analysis_mask;
  cv::Mat image_to_analysis;
  cv::Point2d analysis_origin_a4_mm(0.0, 0.0);
  std::string extraction_failure;
  const int rectified_width = std::max(
    1, static_cast<int>(std::lround(kA4WidthMm * pixels_per_mm_)));
  const int rectified_height = std::max(
    1, static_cast<int>(std::lround(kA4HeightMm * pixels_per_mm_)));
  const std::array<cv::Point2f, 4> rectified_corners = {
    cv::Point2f(0.0F, 0.0F),
    cv::Point2f(static_cast<float>(rectified_width - 1), 0.0F),
    cv::Point2f(
      static_cast<float>(rectified_width - 1), static_cast<float>(rectified_height - 1)),
    cv::Point2f(0.0F, static_cast<float>(rectified_height - 1))};
  image_to_analysis = cv::getPerspectiveTransform(a4_corners.data(), rectified_corners.data());
  cv::warpPerspective(
    frame, analysis_image, image_to_analysis, cv::Size(rectified_width, rectified_height));
  valid_analysis_mask = cv::Mat(analysis_image.size(), CV_8UC1, cv::Scalar(255));

  cv::Mat annotated_analysis = analysis_image.clone();
  scene.pieces = task_id == 3 ?
    extract_challenge_two_pieces(
      analysis_image, valid_analysis_mask, analysis_origin_a4_mm,
      annotated_analysis, extraction_failure) :
    extract_pieces(analysis_image, annotated_analysis, task_id, extraction_failure);
  scene.rectified_origin_a4_mm = to_point32(cv::Point2f(
      static_cast<float>(analysis_origin_a4_mm.x),
      static_cast<float>(analysis_origin_a4_mm.y)));
  const cv::Mat a4_to_workspace = estimate_a4_to_workspace(a4_corners);
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      const auto index = static_cast<std::size_t>(row * 3 + column);
      scene.a4_to_workspace_homography[index] = a4_to_workspace.at<double>(row, column);
      scene.a4_to_image_homography[index] = a4_to_image.at<double>(row, column);
    }
  }
  for (std::size_t index = 0; index < local_a4.size(); ++index) {
    scene.a4_corners_workspace_mm[index] = to_point32(transform_point(a4_to_workspace, local_a4[index]));
  }

  bool challenge_initial_region_valid = true;
  const bool challenge_task = task_id == 2 || task_id == 3;
  const bool challenge_initial_scan =
    challenge_task && (!challenge_initial_region_confirmed_.load() ||
    challenge_initial_region_generation_.load() != generation);
  const bool require_initial_upper_region = challenge_initial_scan;
  if (require_initial_upper_region) {
    const double upper_limit_mm =
      region_split_ratio_ * kA4HeightMm - 0.5 * divider_exclusion_mm_;
    for (const auto & piece : scene.pieces) {
      const bool polygon_in_upper = !piece.polygon_a4_mm.empty() &&
        std::all_of(
        piece.polygon_a4_mm.begin(), piece.polygon_a4_mm.end(),
        [upper_limit_mm](const auto & point) {
          return point.y >= 0.0F && point.y <= upper_limit_mm;
        });
      if (piece.region != vision_interfaces::msg::PuzzlePiece::REGION_UPPER ||
        !polygon_in_upper)
      {
        challenge_initial_region_valid = false;
        break;
      }
    }
  }
  const bool piece_count_valid = !scene.pieces.empty() &&
    scene.pieces.size() <= static_cast<std::size_t>(max_piece_count_) &&
    (task_id != 2 || challenge_one_expected_piece_count_ == 0 ||
    scene.pieces.size() == static_cast<std::size_t>(challenge_one_expected_piece_count_));
  // BUG_POINT:TASK2_PARTIAL_PIECE_SET - Never solve a plausible rectangle from
  // only a subset when any physical contour was rejected. The expected
  // challenge-one count is a YAML scene constraint, not a solver constant.
  scene.valid = piece_count_valid &&
    challenge_initial_region_valid;
  scene.status = scene.valid ?
    (workspace_mapping_valid_ ? "SCENE_VALID" : "SCENE_VALID_LOCAL_ONLY") :
    (!challenge_initial_region_valid ? "INITIAL_PIECE_OUTSIDE_UPPER_REGION" :
    (extraction_failure.empty() ? "PIECE_COUNT_INVALID" : extraction_failure));
  if (commit_session_progress && challenge_initial_scan && scene.valid) {
    // BUG_POINT:CHALLENGE_REGION_PHASE - Only the first valid challenge scan proves
    // the official initial condition; later scans include lower-half placed pieces.
    challenge_initial_region_generation_.store(generation);
    challenge_initial_region_confirmed_.store(true);
  }
  scene.rectified_image = to_bgr8_image(scene.header, analysis_image);

  if (publish_debug_image_ || debug_mode_) {
    // Project the annotated rectified result back into the original camera frame
    // so tuning sees the source image, A4 boundary, and piece contours together.
    cv::Mat projected_debug;
    cv::warpPerspective(
      annotated_analysis, projected_debug, image_to_analysis.inv(), frame.size(),
      cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    cv::Mat blended;
    cv::addWeighted(frame, 0.55, projected_debug, 0.45, 0.0, blended);
    cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    std::vector<cv::Point> a4_polygon;
    a4_polygon.reserve(a4_corners.size());
    for (const auto & corner : a4_corners) {
      a4_polygon.emplace_back(cv::Point(cvRound(corner.x), cvRound(corner.y)));
    }
    cv::fillConvexPoly(mask, a4_polygon, cv::Scalar(255));
    cv::Mat debug_image = frame.clone();
    blended.copyTo(debug_image, mask);
    cv::polylines(debug_image, a4_polygon, true, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
    const cv::Point green_center_pixel(cvRound(green_center.x), cvRound(green_center.y));
    cv::drawMarker(
      debug_image, green_center_pixel, cv::Scalar(0, 255, 255), cv::MARKER_CROSS,
      24, 2, cv::LINE_AA);
    std::ostringstream green_label;
    green_label << "GREEN C(" << std::lround(centered_green.x) << "," <<
      std::lround(centered_green.y) << ")px";
    cv::putText(
      debug_image, green_label.str(), green_center_pixel + cv::Point(10, 24),
      cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    for (const auto & piece : scene.pieces) {
      try {
        const auto image_point = puzzle_geometry::apply_homography(
          scene.a4_to_image_homography,
          puzzle_geometry::Point2d{piece.center_a4_mm.x, piece.center_a4_mm.y});
        const auto centered = puzzle_geometry::camera_centered_pixel(
          image_point, static_cast<double>(scene.image_width),
          static_cast<double>(scene.image_height), false);
        const cv::Point center(cvRound(image_point.x), cvRound(image_point.y));
        cv::drawMarker(
          debug_image, center, cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 18, 2, cv::LINE_AA);
        std::ostringstream label;
        label << "P" << piece.id << " C(" << std::lround(centered.x) << "," <<
          std::lround(centered.y) << ")px";
        cv::putText(
          debug_image, label.str(), center + cv::Point(10, -10),
          cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
      } catch (const std::exception &) {
        // BUG_POINT:DEBUG_COORDINATE - Invalid homography must not stop scene publication.
      }
    }
    cv::putText(
      debug_image, scene.status, cv::Point(24, 44), cv::FONT_HERSHEY_SIMPLEX,
      1.0, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    set_latest_debug_frame(debug_image, camera_generation);
  }
  return scene;
}

bool PuzzlePerceptionNode::detect_a4_quad(
  const cv::Mat & frame, std::array<cv::Point2f, 4> & corners, cv::Mat & green_mask) const
{
  if (!require_green_a4_ && fixed_a4_corners_px_.size() == 8 &&
    std::all_of(fixed_a4_corners_px_.begin(), fixed_a4_corners_px_.end(), [](double value) {
      return value >= 0.0;
    }))
  {
    for (std::size_t index = 0; index < 4; ++index) {
      corners[index] = cv::Point2f(
        static_cast<float>(fixed_a4_corners_px_[index * 2]),
        static_cast<float>(fixed_a4_corners_px_[index * 2 + 1]));
    }
    return true;
  }

  puzzle_perception_node::GreenA4DetectorParams params;
  params.hsv_range = green_a4_hsv_;
  params.morph_kernel = green_a4_morph_kernel_;
  params.morph_iterations = green_a4_morph_iterations_;
  params.min_area_ratio = a4_min_area_ratio_;
  params.max_area_ratio = a4_max_area_ratio_;
  params.min_short_long_ratio = green_frame_min_short_long_ratio_;
  params.polygon_epsilon_ratio = a4_polygon_epsilon_ratio_;
  params.min_border_coverage = green_a4_min_border_coverage_;
  params.line_fit_band_px = green_a4_line_fit_band_px_;
  params.line_fit_min_points = green_a4_line_fit_min_points_;
  params.max_corner_refine_px = green_a4_max_corner_refine_px_;
  return puzzle_perception_node::detect_green_a4_quad(frame, params, corners, &green_mask);
}

std::vector<vision_interfaces::msg::PuzzlePiece> PuzzlePerceptionNode::extract_pieces(
  const cv::Mat & rectified, cv::Mat & debug_image, const int task_id,
  std::string & failure_status) const
{
  failure_status.clear();
  const int page_margin_px = std::max(
    0, static_cast<int>(std::lround(page_margin_mm_ * pixels_per_mm_)));
  const int divider_y = static_cast<int>(
    std::lround(region_split_ratio_ * kA4HeightMm * pixels_per_mm_));
  const int divider_half = std::max(
    1, static_cast<int>(std::lround(0.5 * divider_exclusion_mm_ * pixels_per_mm_)));
  cv::Mat foreground;
  const bool use_challenge_canny = task_id == 2 && challenge_piece_use_canny_;
  const bool use_plain_background_inversion =
    (task_id == 1 || task_id == 2) && plain_piece_use_a4_background_inversion_ &&
    !use_challenge_canny;
  const std::string plain_status_prefix = task_id == 1 ? "BASIC" : "TASK2";
  if (use_challenge_canny) {
    puzzle_perception_node::CannyPieceForegroundMaskParams canny_params;
    canny_params.page_margin_px = page_margin_px;
    canny_params.divider_center_y_px = divider_y;
    canny_params.divider_half_height_px = divider_half;
    canny_params.low_threshold = canny_low_threshold_;
    canny_params.high_threshold = canny_high_threshold_;
    canny_params.blur_kernel = canny_blur_kernel_;
    canny_params.aperture_size = canny_aperture_size_;
    canny_params.l2_gradient = canny_l2_gradient_;
    canny_params.edge_dilate_iterations = canny_edge_dilate_iterations_;
    canny_params.close_kernel = canny_close_kernel_;
    canny_params.close_iterations = canny_close_iterations_;
    canny_params.flood_fill_background = canny_flood_fill_background_;
    foreground = puzzle_perception_node::make_canny_piece_foreground_mask(
      rectified, canny_params);
    std::ostringstream label;
    label << "TASK2 CANNY " << std::fixed << std::setprecision(0) <<
      canny_low_threshold_ << "/" << canny_high_threshold_;
    if (challenge_piece_use_lab_fill_) {
      cv::Mat page_interior_mask(rectified.size(), CV_8UC1, cv::Scalar(0));
      const int interior_width = rectified.cols - 2 * page_margin_px;
      const int interior_height = rectified.rows - 2 * page_margin_px;
      if (interior_width > 0 && interior_height > 0) {
        cv::rectangle(
          page_interior_mask,
          cv::Rect(page_margin_px, page_margin_px, interior_width, interior_height),
          cv::Scalar(255), cv::FILLED);
        const int divider_start = std::clamp(divider_y - divider_half, 0, rectified.rows);
        const int divider_end = std::clamp(divider_y + divider_half + 1, 0, rectified.rows);
        if (divider_start < divider_end) {
          page_interior_mask.rowRange(divider_start, divider_end).setTo(0);
        }
        puzzle_perception_node::LabBackgroundMaskParams fill_params;
        fill_params.distance_threshold = plain_piece_background_distance_threshold_;
        fill_params.max_background_mad = plain_piece_background_max_mad_;
        fill_params.sample_stride = plain_piece_background_sample_stride_;
        fill_params.minimum_samples = 200;
        fill_params.morph_kernel = morph_kernel_;
        fill_params.morph_iterations = morph_iterations_;
        fill_params.include_lightness = !plain_piece_background_chroma_only_;
        auto fill_result = puzzle_perception_node::make_lab_background_foreground_mask(
          rectified, page_interior_mask, fill_params);
        if (fill_result.valid) {
          if (foreground.empty()) {
            foreground = fill_result.foreground_mask;
          } else {
            cv::bitwise_or(foreground, fill_result.foreground_mask, foreground);
          }
          label << "+LAB mad=" << std::fixed << std::setprecision(1) <<
            fill_result.background_mad;
        } else {
          label << "+LAB:" << fill_result.status;
        }
      } else {
        label << "+LAB:A4_INTERIOR_INVALID";
      }
    }
    cv::putText(
      debug_image, label.str(), cv::Point(12, debug_image.rows - 16),
      cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
  } else if (use_plain_background_inversion) {
    cv::Mat page_interior_mask(rectified.size(), CV_8UC1, cv::Scalar(0));
    const int interior_width = rectified.cols - 2 * page_margin_px;
    const int interior_height = rectified.rows - 2 * page_margin_px;
    if (interior_width <= 0 || interior_height <= 0) {
      failure_status = plain_status_prefix + "_BACKGROUND_SAMPLE_INVALID";
      return {};
    }
    cv::rectangle(
      page_interior_mask,
      cv::Rect(page_margin_px, page_margin_px, interior_width, interior_height),
      cv::Scalar(255), cv::FILLED);
    const int divider_start = std::clamp(divider_y - divider_half, 0, rectified.rows);
    const int divider_end = std::clamp(divider_y + divider_half + 1, 0, rectified.rows);
    if (divider_start < divider_end) {
      page_interior_mask.rowRange(divider_start, divider_end).setTo(0);
    }

    puzzle_perception_node::LabBackgroundMaskParams params;
    params.distance_threshold = plain_piece_background_distance_threshold_;
    params.max_background_mad = plain_piece_background_max_mad_;
    params.sample_stride = plain_piece_background_sample_stride_;
    params.minimum_samples = 200;
    params.morph_kernel = morph_kernel_;
    params.morph_iterations = morph_iterations_;
    params.include_lightness = !plain_piece_background_chroma_only_;
    auto foreground_result = puzzle_perception_node::make_lab_background_foreground_mask(
      rectified, page_interior_mask, params);
    if (!foreground_result.valid) {
      // BUG_POINT:PLAIN_BACKGROUND_INVERSION - Plain pieces should be found by
      // the current green A4 background, not by guessed white-only contours.
      failure_status = plain_status_prefix + "_" + foreground_result.status;
      return {};
    }
    foreground = foreground_result.foreground_mask;
    if (remove_magnet_color_) {
      cv::Mat hsv;
      cv::cvtColor(rectified, hsv, cv::COLOR_BGR2HSV);
      const cv::Mat magnet = puzzle_perception_node::make_hsv_object_mask(hsv, magnet_hsv_);
      foreground.setTo(0, magnet);
    }
    std::ostringstream label;
    label << plain_status_prefix << " BG_INV " <<
      (plain_piece_background_chroma_only_ ? "ab" : "Lab") <<
      " d=" << std::fixed << std::setprecision(1) <<
      plain_piece_background_distance_threshold_ <<
      " mad=" << foreground_result.background_mad;
    cv::putText(
      debug_image, label.str(), cv::Point(12, debug_image.rows - 16),
      cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
  } else {
    foreground = puzzle_perception_node::make_piece_foreground_mask(
      rectified, puzzle_perception_node::PieceForegroundMaskParams{
        white_piece_hsv_, remove_magnet_color_, magnet_hsv_, page_margin_px, divider_y,
        divider_half, morph_kernel_, morph_iterations_});
  }
  if (foreground.empty()) {
    // BUG_POINT:PIECE_FOREGROUND_MASK - Invalid runtime parameters must fail closed.
    failure_status = use_challenge_canny ?
      "TASK2_CANNY_FOREGROUND_MASK_INVALID" : use_plain_background_inversion ?
      plain_status_prefix + "_FOREGROUND_MASK_INVALID" :
      "WHITE_FOREGROUND_MASK_INVALID";
    return {};
  }

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(foreground, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  std::vector<std::vector<cv::Point>> piece_contours;
  std::vector<double> piece_areas_mm2;
  for (const auto & contour : contours) {
    const double area_px = std::abs(cv::contourArea(contour));
    const double area_mm2 = area_px / (pixels_per_mm_ * pixels_per_mm_);
    if (area_mm2 >= min_piece_area_mm2_ && area_mm2 <= max_piece_area_mm2_) {
      piece_contours.push_back(contour);
      piece_areas_mm2.push_back(area_mm2);
    }
  }

  // The fixed basic task has one triangle and three quadrilaterals. Generate
  // both robust vertex-count candidates per contour, then choose the globally
  // consistent area/vertex assignment. This prevents a low-epsilon chamfer
  // from silently turning a four-sided piece into a five-sided one.
  std::vector<puzzle_perception_node::PolygonRefinementResult> basic_refinements;
  if (task_id == 1) {
    if (piece_contours.size() != 4U) {
      return {};
    }
    constexpr std::array<int, 4> kTemplateVertices{3, 4, 4, 4};
    constexpr std::array<double, 4> kTemplateAreasMm2{2400.0, 480.0, 1080.0, 2040.0};
    std::array<std::array<puzzle_perception_node::PolygonRefinementResult, 4>, 4> fits;
    for (std::size_t contour_index = 0; contour_index < piece_contours.size(); ++contour_index) {
      for (std::size_t template_index = 0; template_index < kTemplateVertices.size(); ++template_index) {
        puzzle_perception_node::PolygonRefinementParams params;
        params.epsilon_min_px = contour_epsilon_min_mm_ * pixels_per_mm_;
        params.epsilon_max_px = basic_contour_epsilon_max_mm_ * pixels_per_mm_;
        params.epsilon_steps = contour_epsilon_steps_;
        params.min_vertices = kTemplateVertices[template_index];
        params.max_vertices = kTemplateVertices[template_index];
        params.minimum_points_per_edge = contour_min_points_per_edge_;
        params.maximum_rms_px = contour_max_line_rms_mm_ * pixels_per_mm_;
        params.maximum_residual_px = contour_max_line_residual_mm_ * pixels_per_mm_;
        fits[contour_index][template_index] =
          puzzle_perception_node::refine_polygon_from_contour(
          piece_contours[contour_index], params);
      }
    }

    std::array<std::size_t, 4> assignment{0U, 1U, 2U, 3U};
    std::array<std::size_t, 4> best_assignment{0U, 1U, 2U, 3U};
    double best_score = std::numeric_limits<double>::infinity();
    do {
      bool valid = true;
      double score = 0.0;
      for (std::size_t contour_index = 0; contour_index < piece_contours.size(); ++contour_index) {
        const std::size_t template_index = assignment[contour_index];
        const auto & refined = fits[contour_index][template_index];
        const double area_error = std::abs(
          piece_areas_mm2[contour_index] - kTemplateAreasMm2[template_index]) /
          kTemplateAreasMm2[template_index];
        if (!refined.valid ||
          area_error > basic_template_piece_area_relative_tolerance_)
        {
          valid = false;
          break;
        }
        score += area_error + 0.02 * refined.rms_residual_px / pixels_per_mm_;
      }
      if (valid && score < best_score) {
        best_score = score;
        best_assignment = assignment;
      }
    } while (std::next_permutation(assignment.begin(), assignment.end()));
    if (!std::isfinite(best_score)) {
      // BUG_POINT:BASIC_CONTOUR_REFINEMENT - A four-piece colour mask is not
      // enough: the fixed 3/4/4/4 vertex and area pattern must also be credible.
      return {};
    }
    basic_refinements.reserve(piece_contours.size());
    for (std::size_t contour_index = 0; contour_index < piece_contours.size(); ++contour_index) {
      basic_refinements.push_back(fits[contour_index][best_assignment[contour_index]]);
    }
  }

  struct Candidate
  {
    vision_interfaces::msg::PuzzlePiece message;
    std::vector<cv::Point2f> polygon;
    double line_rms_mm{0.0};
  };
  std::vector<Candidate> candidates;
  for (std::size_t contour_index = 0; contour_index < piece_contours.size(); ++contour_index) {
    const auto & contour = piece_contours[contour_index];
    const double area_px = std::abs(cv::contourArea(contour));
    const double area_mm2 = piece_areas_mm2[contour_index];
    std::vector<cv::Point2f> polygon;
    double line_rms_mm = 0.0;
    if (task_id == 1) {
      polygon = basic_refinements[contour_index].vertices;
      line_rms_mm = basic_refinements[contour_index].rms_residual_px / pixels_per_mm_;
    } else {
      std::vector<cv::Point> initial;
      cv::approxPolyDP(contour, initial, polygon_epsilon_mm_ * pixels_per_mm_, true);
      polygon.reserve(initial.size());
      for (const auto & point : initial) {
        polygon.emplace_back(static_cast<float>(point.x), static_cast<float>(point.y));
      }
    }
    if (polygon.size() < 3 || polygon.size() > static_cast<std::size_t>(max_polygon_vertices_)) {
      continue;
    }
    bool all_edges_long_enough = true;
    const double active_min_piece_edge_mm =
      task_id == 1 ? basic_min_piece_edge_mm_ : min_piece_edge_mm_;
    for (std::size_t index = 0; index < polygon.size(); ++index) {
      const auto & first = polygon[index];
      const auto & second = polygon[(index + 1U) % polygon.size()];
      if (cv::norm(second - first) / pixels_per_mm_ < active_min_piece_edge_mm) {
        all_edges_long_enough = false;
        break;
      }
    }
    if (!all_edges_long_enough) {
      continue;
    }

    const cv::Moments moments = cv::moments(contour);
    if (std::abs(moments.m00) < 1e-6) {
      continue;
    }
    const cv::Point2f center_px(
      static_cast<float>(moments.m10 / moments.m00),
      static_cast<float>(moments.m01 / moments.m00));
    cv::Mat piece_mask(foreground.size(), CV_8UC1, cv::Scalar(0));
    cv::drawContours(piece_mask, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(255), cv::FILLED);
    cv::Mat distance;
    cv::distanceTransform(piece_mask, distance, cv::DIST_L2, 5);
    cv::Point pick_px;
    cv::minMaxLoc(distance, nullptr, nullptr, nullptr, &pick_px);

    Candidate candidate;
    candidate.polygon = polygon;
    candidate.line_rms_mm = line_rms_mm;
    candidate.message.region = center_px.y < divider_y ?
      vision_interfaces::msg::PuzzlePiece::REGION_UPPER :
      vision_interfaces::msg::PuzzlePiece::REGION_LOWER;
    candidate.message.center_a4_mm = to_point32(center_px * static_cast<float>(1.0 / pixels_per_mm_));
    candidate.message.pick_point_a4_mm = to_point32(
      cv::Point2f(static_cast<float>(pick_px.x / pixels_per_mm_), static_cast<float>(pick_px.y / pixels_per_mm_)));
    candidate.message.area_mm2 = static_cast<float>(area_mm2);
    std::vector<puzzle_geometry::Point2d> polygon_mm;
    polygon_mm.reserve(polygon.size());
    for (const auto & point : polygon) {
      polygon_mm.push_back(puzzle_geometry::Point2d{
        point.x / pixels_per_mm_, point.y / pixels_per_mm_});
    }
    candidate.message.orientation_deg = static_cast<float>(
      puzzle_geometry::dominant_edge_angle_deg(polygon_mm));
    const double polygon_area = std::abs(cv::contourArea(polygon));
    candidate.message.confidence = static_cast<float>(
      std::clamp(1.0 - std::abs(area_px - polygon_area) / std::max(1.0, area_px), 0.0, 1.0));
    for (const auto & point : polygon) {
      candidate.message.polygon_a4_mm.push_back(to_point32(
        cv::Point2f(static_cast<float>(point.x / pixels_per_mm_), static_cast<float>(point.y / pixels_per_mm_))));
    }
    candidates.push_back(std::move(candidate));
  }

  std::sort(candidates.begin(), candidates.end(), [](const Candidate & left, const Candidate & right) {
    if (left.message.center_a4_mm.y != right.message.center_a4_mm.y) {
      return left.message.center_a4_mm.y < right.message.center_a4_mm.y;
    }
    return left.message.center_a4_mm.x < right.message.center_a4_mm.x;
  });
  std::vector<vision_interfaces::msg::PuzzlePiece> output;
  output.reserve(candidates.size());
  cv::line(
    debug_image, cv::Point(0, divider_y), cv::Point(debug_image.cols - 1, divider_y),
    cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
  cv::putText(
    debug_image, "UPPER / SOURCE", cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX,
    0.65, cv::Scalar(255, 180, 0), 2, cv::LINE_AA);
  cv::putText(
    debug_image, "LOWER / TARGET", cv::Point(12, std::min(debug_image.rows - 12, divider_y + 30)),
    cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(255, 180, 0), 2, cv::LINE_AA);
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    candidates[index].message.id = static_cast<std::uint32_t>(index + 1);
    const bool upper = candidates[index].message.region ==
      vision_interfaces::msg::PuzzlePiece::REGION_UPPER;
    const cv::Scalar region_color = upper ? cv::Scalar(0, 255, 0) : cv::Scalar(255, 180, 0);
    std::vector<cv::Point> debug_polygon;
    debug_polygon.reserve(candidates[index].polygon.size());
    for (const auto & point : candidates[index].polygon) {
      debug_polygon.emplace_back(cvRound(point.x), cvRound(point.y));
    }
    cv::polylines(debug_image, debug_polygon, true, region_color, 2, cv::LINE_AA);
    const cv::Point center(
      static_cast<int>(std::lround(candidates[index].message.center_a4_mm.x * pixels_per_mm_)),
      static_cast<int>(std::lround(candidates[index].message.center_a4_mm.y * pixels_per_mm_)));
    cv::circle(debug_image, center, 5, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
    cv::putText(
      debug_image,
      "P" + std::to_string(candidates[index].message.id) + (upper ? " U" : " L") +
      (task_id == 1 ? " r=" + std::to_string(candidates[index].line_rms_mm).substr(0, 4) : ""),
      center + cv::Point(8, -8),
      cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    output.push_back(std::move(candidates[index].message));
  }
  return output;
}

std::vector<vision_interfaces::msg::PuzzlePiece>
PuzzlePerceptionNode::extract_challenge_two_pieces(
  const cv::Mat & plane_canvas, const cv::Mat & valid_canvas_mask,
  const cv::Point2d & canvas_origin_a4_mm, cv::Mat & debug_image,
  std::string & failure_status) const
{
  failure_status.clear();
  if (plane_canvas.empty() || plane_canvas.type() != CV_8UC3 ||
    valid_canvas_mask.empty() || valid_canvas_mask.type() != CV_8UC1 ||
    plane_canvas.size() != valid_canvas_mask.size())
  {
    failure_status = "TASK3_CANVAS_INVALID";
    return {};
  }

  cv::Mat page_interior_mask(plane_canvas.size(), CV_8UC1, cv::Scalar(0));
  const int page_margin_px = std::max(
    0, static_cast<int>(std::lround(page_margin_mm_ * pixels_per_mm_)));
  const int interior_width = plane_canvas.cols - 2 * page_margin_px;
  const int interior_height = plane_canvas.rows - 2 * page_margin_px;
  if (interior_width <= 0 || interior_height <= 0) {
    failure_status = "TASK3_A4_INTERIOR_INVALID";
    return {};
  }
  cv::rectangle(
    page_interior_mask,
    cv::Rect(page_margin_px, page_margin_px, interior_width, interior_height),
    cv::Scalar(255), cv::FILLED);
  const int divider_y = static_cast<int>(
    std::lround(region_split_ratio_ * kA4HeightMm * pixels_per_mm_));
  const int divider_top = std::max(
    0, divider_y - std::max(
      1, static_cast<int>(std::lround(0.5 * divider_exclusion_mm_ * pixels_per_mm_))));
  const int divider_bottom = std::min(
    plane_canvas.rows, 2 * divider_y - divider_top + 1);
  cv::rectangle(
    page_interior_mask,
    cv::Rect(0, divider_top, plane_canvas.cols, divider_bottom - divider_top),
    cv::Scalar(0), cv::FILLED);
  cv::Mat sampling_mask;
  cv::bitwise_and(valid_canvas_mask, page_interior_mask, sampling_mask);
  const cv::Mat border_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::erode(sampling_mask, sampling_mask, border_kernel);

  cv::Mat foreground;
  std::string segmentation_label;
  if (challenge_piece_use_canny_) {
    puzzle_perception_node::CannyPieceForegroundMaskParams canny_params;
    canny_params.low_threshold = canny_low_threshold_;
    canny_params.high_threshold = canny_high_threshold_;
    canny_params.blur_kernel = canny_blur_kernel_;
    canny_params.aperture_size = canny_aperture_size_;
    canny_params.l2_gradient = canny_l2_gradient_;
    canny_params.edge_dilate_iterations = canny_edge_dilate_iterations_;
    canny_params.close_kernel = canny_close_kernel_;
    canny_params.close_iterations = canny_close_iterations_;
    canny_params.flood_fill_background = canny_flood_fill_background_;
    foreground = puzzle_perception_node::make_canny_piece_foreground_mask(
      plane_canvas, sampling_mask, canny_params);
    std::ostringstream label;
    label << "Canny " << std::fixed << std::setprecision(0) <<
      canny_low_threshold_ << "/" << canny_high_threshold_;
    if (challenge_piece_use_lab_fill_) {
      puzzle_perception_node::LabBackgroundMaskParams fill_params;
      fill_params.distance_threshold = challenge_two_background_distance_threshold_;
      fill_params.max_background_mad = challenge_two_background_max_mad_;
      fill_params.sample_stride = challenge_two_background_sample_stride_;
      fill_params.minimum_samples = 200;
      fill_params.morph_kernel = morph_kernel_;
      fill_params.morph_iterations = morph_iterations_;
      auto fill_result = puzzle_perception_node::make_lab_background_foreground_mask(
        plane_canvas, sampling_mask, fill_params);
      if (fill_result.valid) {
        if (foreground.empty()) {
          foreground = fill_result.foreground_mask;
        } else {
          cv::bitwise_or(foreground, fill_result.foreground_mask, foreground);
        }
        label << "+Lab MAD=" << std::fixed << std::setprecision(1) <<
          fill_result.background_mad;
      } else {
        label << "+Lab:" << fill_result.status;
      }
    }
    segmentation_label = label.str();
  } else {
    puzzle_perception_node::LabBackgroundMaskParams background_params;
    background_params.distance_threshold = challenge_two_background_distance_threshold_;
    background_params.max_background_mad = challenge_two_background_max_mad_;
    background_params.sample_stride = challenge_two_background_sample_stride_;
    background_params.minimum_samples = 200;
    background_params.morph_kernel = morph_kernel_;
    background_params.morph_iterations = morph_iterations_;
    auto foreground_result = puzzle_perception_node::make_lab_background_foreground_mask(
      plane_canvas, sampling_mask, background_params);
    if (!foreground_result.valid) {
      // BUG_POINT:TASK3_BACKGROUND_SAMPLE - The current light-green A4 background
      // must be sampled successfully instead of inheriting a fixed paper threshold.
      failure_status = foreground_result.status;
      return {};
    }
    foreground = foreground_result.foreground_mask;
    std::ostringstream label;
    label << "Lab BG MAD=" << std::fixed << std::setprecision(1) <<
      foreground_result.background_mad;
    segmentation_label = label.str();
  }
  if (foreground.empty()) {
    failure_status = challenge_piece_use_canny_ ?
      "TASK3_CANNY_FOREGROUND_MASK_INVALID" : "TASK3_FOREGROUND_MASK_INVALID";
    return {};
  }
  if (remove_magnet_color_) {
    cv::Mat hsv;
    cv::cvtColor(plane_canvas, hsv, cv::COLOR_BGR2HSV);
    const cv::Mat magnet = puzzle_perception_node::make_hsv_object_mask(hsv, magnet_hsv_);
    foreground.setTo(0, magnet);
  }

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(foreground.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  struct Candidate
  {
    vision_interfaces::msg::PuzzlePiece message;
    std::vector<cv::Point2f> polygon_px;
    double residual_mm{0.0};
    double rectangularity{0.0};
  };
  std::vector<Candidate> candidates;
  const auto make_candidate = [&](
    const std::vector<cv::Point> & contour,
    const std::vector<cv::Point2f> & polygon_px,
    const double residual_mm,
    const double confidence) -> std::optional<Candidate> {
      const cv::Moments moments = cv::moments(contour);
      if (std::abs(moments.m00) < 1e-6) {
        return std::nullopt;
      }
      const cv::Point2f center_px(
        static_cast<float>(moments.m10 / moments.m00),
        static_cast<float>(moments.m01 / moments.m00));
      cv::Mat piece_mask(foreground.size(), CV_8UC1, cv::Scalar(0));
      cv::drawContours(
        piece_mask, std::vector<std::vector<cv::Point>>{contour}, -1,
        cv::Scalar(255), cv::FILLED);
      cv::Mat distance;
      cv::distanceTransform(piece_mask, distance, cv::DIST_L2, 5);
      cv::Point pick_px;
      cv::minMaxLoc(distance, nullptr, nullptr, nullptr, &pick_px);

      Candidate candidate;
      candidate.polygon_px = polygon_px;
      candidate.residual_mm = residual_mm;
      const auto to_a4_mm = [&](const cv::Point2f & point) {
          return cv::Point2f(
            static_cast<float>(point.x / pixels_per_mm_ + canvas_origin_a4_mm.x),
            static_cast<float>(point.y / pixels_per_mm_ + canvas_origin_a4_mm.y));
        };
      candidate.message.center_a4_mm = to_point32(to_a4_mm(center_px));
      candidate.message.region = candidate.message.center_a4_mm.y <
        region_split_ratio_ * kA4HeightMm ?
        vision_interfaces::msg::PuzzlePiece::REGION_UPPER :
        vision_interfaces::msg::PuzzlePiece::REGION_LOWER;
      candidate.message.pick_point_a4_mm = to_point32(to_a4_mm(cv::Point2f(pick_px)));
      candidate.message.area_mm2 = static_cast<float>(
        std::abs(cv::contourArea(contour)) / (pixels_per_mm_ * pixels_per_mm_));
      std::vector<puzzle_geometry::Point2d> polygon_mm;
      polygon_mm.reserve(polygon_px.size());
      for (const auto & point : polygon_px) {
        const auto point_mm = to_a4_mm(point);
        candidate.message.polygon_a4_mm.push_back(to_point32(point_mm));
        polygon_mm.push_back(puzzle_geometry::Point2d{point_mm.x, point_mm.y});
      }
      candidate.message.orientation_deg = static_cast<float>(
        puzzle_geometry::dominant_edge_angle_deg(polygon_mm));
      candidate.message.confidence = static_cast<float>(
        std::clamp(confidence, 0.0, 1.0));
      return candidate;
  };
  const double active_min_piece_area_mm2 = challenge_two_rectangle_detection_enabled_ ?
    std::max(min_piece_area_mm2_, challenge_two_rectangle_min_area_mm2_) :
    min_piece_area_mm2_;
  for (const auto & contour : contours) {
    const double area_px = std::abs(cv::contourArea(contour));
    const double area_mm2 = area_px / (pixels_per_mm_ * pixels_per_mm_);
    if (area_mm2 < active_min_piece_area_mm2 || area_mm2 > max_piece_area_mm2_) {
      continue;
    }
    puzzle_perception_node::PolygonRefinementParams refinement_params;
    refinement_params.epsilon_min_px = contour_epsilon_min_mm_ * pixels_per_mm_;
    refinement_params.epsilon_max_px = contour_epsilon_max_mm_ * pixels_per_mm_;
    refinement_params.epsilon_steps = contour_epsilon_steps_;
    refinement_params.min_vertices = 3;
    refinement_params.max_vertices = max_polygon_vertices_;
    refinement_params.minimum_points_per_edge = contour_min_points_per_edge_;
    refinement_params.maximum_rms_px = contour_max_line_rms_mm_ * pixels_per_mm_;
    refinement_params.maximum_residual_px = contour_max_line_residual_mm_ * pixels_per_mm_;
    const auto refined = puzzle_perception_node::refine_polygon_from_contour(
      contour, refinement_params);
    if (!refined.valid) {
      continue;
    }
    bool edges_valid = true;
    for (std::size_t index = 0; index < refined.vertices.size(); ++index) {
      if (cv::norm(refined.vertices[(index + 1U) % refined.vertices.size()] -
        refined.vertices[index]) / pixels_per_mm_ < challenge_two_min_piece_edge_mm_)
      {
        edges_valid = false;
        break;
      }
    }
    if (!edges_valid) {
      continue;
    }
    auto candidate = make_candidate(
      contour, refined.vertices, refined.rms_residual_px / pixels_per_mm_,
      1.0 - refined.rms_residual_px /
      std::max(1e-6, contour_max_line_rms_mm_ * pixels_per_mm_));
    if (candidate) {
      candidates.push_back(std::move(*candidate));
    }
  }

  if (challenge_two_rectangle_detection_enabled_ &&
    candidates.size() < static_cast<std::size_t>(max_piece_count_))
  {
    std::vector<Candidate> rectangle_candidates;
    for (const auto & contour : contours) {
      const double contour_area_px = std::abs(cv::contourArea(contour));
      const double contour_area_mm2 = contour_area_px / (pixels_per_mm_ * pixels_per_mm_);
      if (contour_area_mm2 < challenge_two_rectangle_min_area_mm2_ ||
        contour_area_mm2 > challenge_two_rectangle_max_area_mm2_)
      {
        continue;
      }
      const cv::RotatedRect rectangle = cv::minAreaRect(contour);
      const double short_edge_px = std::min(rectangle.size.width, rectangle.size.height);
      const double long_edge_px = std::max(rectangle.size.width, rectangle.size.height);
      if (short_edge_px <= 1.0 || long_edge_px <= 1.0 ||
        short_edge_px / pixels_per_mm_ < challenge_two_min_piece_edge_mm_)
      {
        continue;
      }
      const double aspect_ratio = long_edge_px / short_edge_px;
      if (aspect_ratio > challenge_two_rectangle_max_aspect_ratio_) {
        continue;
      }
      const double rectangle_area_px = short_edge_px * long_edge_px;
      const double rectangularity = contour_area_px / std::max(1.0, rectangle_area_px);
      if (rectangularity < challenge_two_rectangle_min_rectangularity_ ||
        rectangularity > 1.15)
      {
        continue;
      }
      std::array<cv::Point2f, 4> box{};
      rectangle.points(box.data());
      std::vector<cv::Point2f> polygon_px(box.begin(), box.end());
      auto candidate = make_candidate(
        contour, polygon_px, 1.0 - rectangularity,
        0.35 + 0.65 * rectangularity);
      if (candidate) {
        candidate->rectangularity = rectangularity;
        rectangle_candidates.push_back(std::move(*candidate));
      }
    }
    if (rectangle_candidates.size() > candidates.size()) {
      std::sort(
        rectangle_candidates.begin(), rectangle_candidates.end(),
        [](const Candidate & left, const Candidate & right) {
          return left.message.area_mm2 > right.message.area_mm2;
        });
      if (rectangle_candidates.size() > static_cast<std::size_t>(max_piece_count_)) {
        rectangle_candidates.resize(static_cast<std::size_t>(max_piece_count_));
      }
      candidates = std::move(rectangle_candidates);
      segmentation_label += " Rect";
    }
  }

  if (candidates.empty()) {
    failure_status = "TASK3_NO_VALID_PIECES";
    return {};
  }
  std::sort(candidates.begin(), candidates.end(), [](const Candidate & left, const Candidate & right) {
    if (left.message.center_a4_mm.y != right.message.center_a4_mm.y) {
      return left.message.center_a4_mm.y < right.message.center_a4_mm.y;
    }
    return left.message.center_a4_mm.x < right.message.center_a4_mm.x;
  });
  std::vector<vision_interfaces::msg::PuzzlePiece> output;
  output.reserve(candidates.size());
  cv::line(
    debug_image, cv::Point(0, divider_y), cv::Point(debug_image.cols - 1, divider_y),
    cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    candidates[index].message.id = static_cast<std::uint32_t>(index + 1U);
    std::vector<cv::Point> polygon;
    for (const auto & point : candidates[index].polygon_px) {
      polygon.emplace_back(cvRound(point.x), cvRound(point.y));
      cv::circle(debug_image, polygon.back(), 4, cv::Scalar(0, 255, 255), cv::FILLED);
    }
    cv::polylines(debug_image, polygon, true, cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
    const cv::Point center(
      cvRound((candidates[index].message.center_a4_mm.x - canvas_origin_a4_mm.x) * pixels_per_mm_),
      cvRound((candidates[index].message.center_a4_mm.y - canvas_origin_a4_mm.y) * pixels_per_mm_));
    cv::drawMarker(
      debug_image, center, cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 18, 2, cv::LINE_AA);
    std::ostringstream label;
    label << "P" << candidates[index].message.id << " RMS=" <<
      std::fixed << std::setprecision(2) << candidates[index].residual_mm << "mm " <<
      (candidates[index].message.region == vision_interfaces::msg::PuzzlePiece::REGION_UPPER ?
      "U" : "L");
    cv::putText(
      debug_image, label.str(), center + cv::Point(8, -8), cv::FONT_HERSHEY_SIMPLEX,
      0.55, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    output.push_back(std::move(candidates[index].message));
  }
  cv::putText(
    debug_image, segmentation_label, cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX,
    0.65, cv::Scalar(255, 180, 0), 2, cv::LINE_AA);
  return output;
}

bool PuzzlePerceptionNode::undistort_for_challenge_two(
  const cv::Mat & input, cv::Mat & output) const
{
  if (!camera_calibration_valid_ || input.empty() || camera_matrix_values_.size() != 9U ||
    distortion_coefficients_.size() < 4U || calibration_image_width_ <= 0 ||
    calibration_image_height_ <= 0)
  {
    output.release();
    return false;
  }
  cv::Mat camera_matrix(3, 3, CV_64F);
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      camera_matrix.at<double>(row, column) =
        camera_matrix_values_[static_cast<std::size_t>(row * 3 + column)];
    }
  }
  const double scale_x = static_cast<double>(input.cols) / calibration_image_width_;
  const double scale_y = static_cast<double>(input.rows) / calibration_image_height_;
  camera_matrix.at<double>(0, 0) *= scale_x;
  camera_matrix.at<double>(0, 2) *= scale_x;
  camera_matrix.at<double>(1, 1) *= scale_y;
  camera_matrix.at<double>(1, 2) *= scale_y;
  cv::Mat distortion(1, static_cast<int>(distortion_coefficients_.size()), CV_64F);
  for (std::size_t index = 0; index < distortion_coefficients_.size(); ++index) {
    distortion.at<double>(0, static_cast<int>(index)) = distortion_coefficients_[index];
  }
  try {
    cv::undistort(input, output, camera_matrix, distortion);
  } catch (const cv::Exception &) {
    output.release();
    return false;
  }
  return !output.empty();
}

cv::Mat PuzzlePerceptionNode::estimate_a4_to_workspace(
  const std::array<cv::Point2f, 4> & image_corners) const
{
  if (!workspace_mapping_valid_ || workspace_homography_values_.size() != 9) {
    return cv::Mat::eye(3, 3, CV_64F);
  }
  cv::Mat image_to_workspace(3, 3, CV_64F);
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      image_to_workspace.at<double>(row, column) =
        workspace_homography_values_[static_cast<std::size_t>(row * 3 + column)];
    }
  }
  const std::array<cv::Point2f, 4> local_corners = {
    cv::Point2f(0.0F, 0.0F), cv::Point2f(210.0F, 0.0F),
    cv::Point2f(210.0F, 297.0F), cv::Point2f(0.0F, 297.0F)};
  const cv::Mat local_to_image = cv::getPerspectiveTransform(local_corners.data(), image_corners.data());
  // BUG_POINT:ABSOLUTE_MAPPING - Never replace the fixed camera mapping with gantry motion.
  return image_to_workspace * local_to_image;
}

cv::Point2f PuzzlePerceptionNode::transform_point(
  const cv::Mat & homography, const cv::Point2f & point)
{
  std::vector<cv::Point2f> input{point};
  std::vector<cv::Point2f> output;
  cv::perspectiveTransform(input, output, homography);
  return output.front();
}

double PuzzlePerceptionNode::sharpness_score(const cv::Mat & frame)
{
  cv::Mat bgr_frame;
  if (frame.type() == CV_8UC2) {
    cv::cvtColor(frame, bgr_frame, cv::COLOR_YUV2BGR_YUY2);
  } else {
    bgr_frame = frame;
  }
  cv::Mat gray;
  cv::cvtColor(bgr_frame, gray, cv::COLOR_BGR2GRAY);
  cv::Mat laplacian;
  cv::Laplacian(gray, laplacian, CV_64F);
  cv::Scalar mean;
  cv::Scalar stddev;
  cv::meanStdDev(laplacian, mean, stddev);
  return stddev[0] * stddev[0];
}

void PuzzlePerceptionNode::publish_status(const std::string & text)
{
  std_msgs::msg::String message;
  std::ostringstream stream;
  stream << "{\"status\":\"" << text << "\",\"scan_sequence\":" << scan_sequence_ <<
    ",\"active_task\":" << active_task_.load() <<
    ",\"generation\":" << active_generation_.load() << "}";
  message.data = stream.str();
  status_publisher_->publish(message);
}
