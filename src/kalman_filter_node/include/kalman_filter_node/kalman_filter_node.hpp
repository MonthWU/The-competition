#ifndef KALMAN_FILTER_NODE__KALMAN_FILTER_NODE_HPP_
#define KALMAN_FILTER_NODE__KALMAN_FILTER_NODE_HPP_

#include <chrono>
#include <string>

#include <opencv2/video/tracking.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <vision_interfaces/msg/target_point.hpp>

class KalmanFilterNode : public rclcpp::Node
{
public:
  KalmanFilterNode();

private:
  void on_target(const vision_interfaces::msg::TargetPoint::ConstSharedPtr message);
  void initialize_filter(float x, float y);

  rclcpp::Subscription<vision_interfaces::msg::TargetPoint>::SharedPtr target_subscription_;
  rclcpp::Publisher<vision_interfaces::msg::TargetPoint>::SharedPtr filtered_target_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr filtered_detection_publisher_;
  cv::KalmanFilter filter_;
  std::string raw_target_topic_;
  std::string filtered_target_topic_;
  std::string filtered_detection_topic_;
  int tracked_class_id_;
  int active_class_id_;
  double process_noise_;
  double measurement_noise_;
  double reset_gap_sec_;
  bool debug_mode_;
  bool initialized_;
  std::chrono::steady_clock::time_point last_target_time_;
};

#endif  // KALMAN_FILTER_NODE__KALMAN_FILTER_NODE_HPP_
