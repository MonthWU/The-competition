#ifndef PUZZLE_PERCEPTION_NODE__PUZZLE_PERCEPTION_NODE_HPP_
#define PUZZLE_PERCEPTION_NODE__PUZZLE_PERCEPTION_NODE_HPP_

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <vision_interfaces/msg/puzzle_piece.hpp>
#include <vision_interfaces/msg/puzzle_scene.hpp>
#include <vision_interfaces/msg/scan_request.hpp>
#include <vision_interfaces/msg/task_session.hpp>

#include "puzzle_perception_node/hsv_object_mask.hpp"

class PuzzlePerceptionNode : public rclcpp::Node
{
public:
  PuzzlePerceptionNode();
  ~PuzzlePerceptionNode() override;

private:
  void on_scan_request(const vision_interfaces::msg::ScanRequest::ConstSharedPtr message);
  void on_task_session(const vision_interfaces::msg::TaskSession::ConstSharedPtr message);
  void on_hmi_command(const std_msgs::msg::String::ConstSharedPtr message);
  void on_capture_timer();
  void debug_processing_loop();
  void enqueue_debug_frame(const cv::Mat & frame, std::uint64_t camera_generation);
  void set_latest_debug_frame(const cv::Mat & frame, std::uint64_t camera_generation);
  void publish_debug_frame(const cv::Mat & frame, const rclcpp::Time & stamp);
  bool open_camera_locked();
  void reopen_camera_with_exposure(int exposure_time);
  bool detect_a4_quad(
    const cv::Mat & frame, std::array<cv::Point2f, 4> & corners, cv::Mat & green_mask) const;
  vision_interfaces::msg::PuzzleScene build_scene(
    cv::Mat frame, const rclcpp::Time & stamp, int task_id,
    std::uint32_t generation, std::uint64_t camera_generation,
    bool commit_session_progress);
  std::vector<vision_interfaces::msg::PuzzlePiece> extract_pieces(
    const cv::Mat & rectified, cv::Mat & debug_image, int task_id,
    std::string & failure_status) const;
  std::vector<vision_interfaces::msg::PuzzlePiece> extract_challenge_two_pieces(
    const cv::Mat & plane_canvas, const cv::Mat & valid_canvas_mask,
    const cv::Point2d & canvas_origin_a4_mm, cv::Mat & debug_image,
    std::string & failure_status) const;
  bool undistort_for_challenge_two(const cv::Mat & input, cv::Mat & output) const;
  cv::Mat estimate_a4_to_workspace(const std::array<cv::Point2f, 4> & image_corners) const;
  static cv::Point2f transform_point(const cv::Mat & homography, const cv::Point2f & point);
  static double sharpness_score(const cv::Mat & frame);
  void publish_status(const std::string & text);

  rclcpp::Subscription<vision_interfaces::msg::ScanRequest>::SharedPtr scan_subscription_;
  rclcpp::Subscription<vision_interfaces::msg::TaskSession>::SharedPtr task_session_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr hmi_command_subscription_;
  rclcpp::Publisher<vision_interfaces::msg::PuzzleScene>::SharedPtr scene_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr capture_timer_;
  cv::VideoCapture capture_;

  std::string scan_request_topic_;
  std::string task_session_topic_;
  std::string scene_topic_;
  std::string debug_image_topic_;
  std::string status_topic_;
  std::string hmi_command_topic_;
  std::string camera_pipeline_;
  std::string frame_id_;
  int camera_index_;
  int input_width_;
  int input_height_;
  double input_roi_left_ratio_;
  double input_roi_width_ratio_;
  double capture_fps_;
  double pixels_per_mm_;
  int stable_frames_;
  bool publish_debug_image_;
  bool debug_mode_;
  bool continuous_task_debug_detection_;
  bool workspace_mapping_valid_;
  std::vector<double> fixed_a4_corners_px_;
  std::vector<double> workspace_homography_values_;
  double a4_min_area_ratio_;
  double a4_max_area_ratio_;
  double green_frame_min_short_long_ratio_;
  bool require_green_a4_;
  puzzle_perception_node::HsvRange green_a4_hsv_;
  int green_a4_morph_kernel_;
  int green_a4_morph_iterations_;
  double a4_polygon_epsilon_ratio_;
  double green_a4_min_border_coverage_;
  double green_a4_line_fit_band_px_;
  int green_a4_line_fit_min_points_;
  double green_a4_max_corner_refine_px_;
  puzzle_perception_node::HsvRange white_piece_hsv_;
  bool plain_piece_use_a4_background_inversion_;
  bool basic_piece_use_canny_;
  double plain_piece_background_distance_threshold_;
  double plain_piece_background_max_mad_;
  int plain_piece_background_sample_stride_;
  bool plain_piece_background_chroma_only_;
  bool challenge_piece_use_canny_;
  bool challenge_piece_use_lab_fill_;
  double canny_low_threshold_;
  double canny_high_threshold_;
  int canny_blur_kernel_;
  int canny_aperture_size_;
  bool canny_l2_gradient_;
  int canny_edge_dilate_iterations_;
  int canny_close_kernel_;
  int canny_close_iterations_;
  bool canny_flood_fill_background_;
  double page_margin_mm_;
  double region_split_ratio_;
  double divider_exclusion_mm_;
  int morph_kernel_;
  int morph_iterations_;
  double min_piece_area_mm2_;
  double max_piece_area_mm2_;
  double min_piece_edge_mm_;
  double basic_min_piece_edge_mm_;
  double basic_contour_epsilon_max_mm_;
  double polygon_epsilon_mm_;
  int max_polygon_vertices_;
  int max_piece_count_;
  int challenge_one_expected_piece_count_;
  double basic_template_piece_area_relative_tolerance_;
  bool remove_magnet_color_;
  puzzle_perception_node::HsvRange magnet_hsv_;
  bool camera_calibration_valid_;
  int calibration_image_width_;
  int calibration_image_height_;
  std::vector<double> camera_matrix_values_;
  std::vector<double> distortion_coefficients_;
  double challenge_two_background_distance_threshold_;
  double challenge_two_background_max_mad_;
  int challenge_two_background_sample_stride_;
  double challenge_two_min_piece_edge_mm_;
  bool challenge_two_rectangle_detection_enabled_;
  double challenge_two_rectangle_min_area_mm2_;
  double challenge_two_rectangle_max_area_mm2_;
  double challenge_two_rectangle_min_rectangularity_;
  double challenge_two_rectangle_max_aspect_ratio_;
  double contour_epsilon_min_mm_;
  double contour_epsilon_max_mm_;
  int contour_epsilon_steps_;
  int contour_min_points_per_edge_;
  double contour_max_line_rms_mm_;
  double contour_max_line_residual_mm_;
  std::atomic<int> active_task_{0};
  std::atomic<std::uint32_t> active_generation_{0U};
  std::atomic<std::uint32_t> challenge_initial_region_generation_{0U};
  std::atomic<bool> challenge_initial_region_confirmed_{false};
  std::atomic<std::uint64_t> camera_generation_{0U};

  bool scan_pending_;
  int scan_task_{0};
  std::uint32_t scan_generation_{0U};
  int collected_frames_;
  cv::Mat best_frame_;
  double best_sharpness_;
  std::uint64_t scan_sequence_;

  std::mutex camera_mutex_;
  std::mutex debug_mutex_;
  std::condition_variable debug_condition_;
  cv::Mat pending_debug_frame_;
  cv::Mat latest_debug_frame_;
  std::uint64_t pending_debug_camera_generation_{0U};
  bool debug_frame_ready_{false};
  bool debug_processing_stop_{false};
  std::thread debug_processing_thread_;
};

#endif  // PUZZLE_PERCEPTION_NODE__PUZZLE_PERCEPTION_NODE_HPP_
