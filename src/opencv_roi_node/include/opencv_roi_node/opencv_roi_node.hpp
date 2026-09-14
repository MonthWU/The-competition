#ifndef OPENCV_ROI_NODE__OPENCV_ROI_NODE_HPP_
#define OPENCV_ROI_NODE__OPENCV_ROI_NODE_HPP_

#include <cstdint>
#include <chrono>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <vision_interfaces/msg/roi.hpp>
#include <vision_interfaces/msg/target_point.hpp>

class OpenCvRoiNode : public rclcpp::Node
{
public:
  OpenCvRoiNode();

private:
  void on_roi(const vision_interfaces::msg::Roi::ConstSharedPtr message);
  void publish_status();

  rclcpp::Subscription<vision_interfaces::msg::Roi>::SharedPtr roi_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr processed_roi_publisher_;
  rclcpp::Publisher<vision_interfaces::msg::TargetPoint>::SharedPtr target_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::string roi_topic_;
  std::string processed_roi_topic_;
  std::string target_topic_;
  std::string status_topic_;
  int max_width_;
  int roi_x_;
  int roi_y_;
  int roi_width_;
  int roi_height_;
  int blur_kernel_size_;
  int illumination_kernel_size_;
  double compensation_strength_;
  double gamma_;
  int threshold_;
  int morph_kernel_size_;
  int morph_iterations_;
  int h_min_;
  int h_max_;
  int s_min_;
  int s_max_;
  int v_min_;
  int v_max_;
  double canny_low_threshold_;
  double canny_high_threshold_;
  int canny_aperture_;
  double min_contour_area_;
  double max_contour_area_;
  double contour_epsilon_;
  int contour_thickness_;
  int max_objects_;
  int center_marker_size_;
  int center_marker_thickness_;
  double circularity_threshold_;
  int measurement_decimals_;
  double measurement_font_scale_;
  bool invert_;
  std::string contour_source_;
  std::string center_method_;
  std::string measurement_mode_;
  std::vector<double> perspective_homography_;
  bool perspective_enabled_;
  bool publish_best_only_;
  bool publish_debug_image_;
  bool debug_mode_;
  std::uint64_t processed_roi_count_;
  std::uint64_t measurement_count_;
  double accumulated_processing_ms_;
  double last_processing_ms_;
};

#endif  // OPENCV_ROI_NODE__OPENCV_ROI_NODE_HPP_
