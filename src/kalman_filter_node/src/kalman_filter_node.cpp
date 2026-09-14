#include "kalman_filter_node/kalman_filter_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <sstream>

KalmanFilterNode::KalmanFilterNode()
: Node("kalman_filter_node"),
  active_class_id_(-1),
  process_noise_(1.0),
  measurement_noise_(10.0),
  reset_gap_sec_(0.5),
  debug_mode_(false),
  initialized_(false)
{
  raw_target_topic_ = declare_parameter<std::string>("raw_target_topic", "vision/target_raw");
  filtered_target_topic_ = declare_parameter<std::string>(
    "filtered_target_topic", "vision/target_filtered");
  filtered_detection_topic_ = declare_parameter<std::string>("filtered_detection_topic", "vision/detections");
  tracked_class_id_ = declare_parameter<int>("tracked_class_id", -1);
  process_noise_ = std::max(0.0001, declare_parameter<double>("process_noise", process_noise_));
  measurement_noise_ = std::max(0.0001, declare_parameter<double>("measurement_noise", measurement_noise_));
  reset_gap_sec_ = std::max(0.01, declare_parameter<double>("reset_gap_sec", reset_gap_sec_));
  debug_mode_ = declare_parameter<bool>("debug_mode", false);

  auto sensor_qos = rclcpp::SensorDataQoS();
  sensor_qos.keep_last(1);
  target_subscription_ = create_subscription<vision_interfaces::msg::TargetPoint>(
    raw_target_topic_, sensor_qos,
    std::bind(&KalmanFilterNode::on_target, this, std::placeholders::_1));
  filtered_target_publisher_ = create_publisher<vision_interfaces::msg::TargetPoint>(
    filtered_target_topic_, sensor_qos);
  filtered_detection_publisher_ = create_publisher<std_msgs::msg::String>(filtered_detection_topic_, 10);
  RCLCPP_INFO(
    get_logger(), "Kalman filter ready: input=%s typed_output=%s legacy_output=%s tracked_class=%d",
    raw_target_topic_.c_str(), filtered_target_topic_.c_str(),
    filtered_detection_topic_.c_str(), tracked_class_id_);
}

void KalmanFilterNode::initialize_filter(float x, float y)
{
  filter_.init(4, 2, 0, CV_32F);
  filter_.transitionMatrix = (cv::Mat_<float>(4, 4) <<
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1);
  filter_.measurementMatrix = cv::Mat::zeros(2, 4, CV_32F);
  filter_.measurementMatrix.at<float>(0, 0) = 1.0F;
  filter_.measurementMatrix.at<float>(1, 1) = 1.0F;
  cv::setIdentity(filter_.processNoiseCov, cv::Scalar(process_noise_));
  cv::setIdentity(filter_.measurementNoiseCov, cv::Scalar(measurement_noise_));
  cv::setIdentity(filter_.errorCovPost, cv::Scalar(1.0));
  filter_.statePost = (cv::Mat_<float>(4, 1) << x, y, 0, 0);
  initialized_ = true;
}

void KalmanFilterNode::on_target(const vision_interfaces::msg::TargetPoint::ConstSharedPtr message)
{
  if (!std::isfinite(message->x) || !std::isfinite(message->y)) {
    // BUG_POINT:KALMAN_INPUT -- Non-finite geometry must never enter the
    // filter or the downstream serial quality gate.
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "BUG_POINT:KALMAN_INPUT ignored non-finite target x=%.3f y=%.3f",
      message->x, message->y);
    return;
  }
  if (tracked_class_id_ >= 0 && message->class_id != tracked_class_id_) {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  const auto elapsed = initialized_ ? std::chrono::duration<double>(now - last_target_time_).count() : 0.0;
  cv::Mat corrected;
  if (!initialized_ || message->class_id != active_class_id_ || elapsed > reset_gap_sec_) {
    // BUG_POINT:KALMAN_RESET -- Never predict across a target-class change or
    // long observation gap; that would send a stale target point to the MCU.
    initialize_filter(message->x, message->y);
    active_class_id_ = message->class_id;
    // BUG_POINT:KALMAN_INITIALIZATION -- The first point is the known filter
    // state. Calling correct() before a prediction would pull it toward zero.
    corrected = filter_.statePost.clone();
    if (debug_mode_) {
      RCLCPP_DEBUG(
        get_logger(),
        "BUG_POINT:KALMAN_RESET class=%d elapsed=%.3f reset_gap=%.3f raw=(%.2f,%.2f)",
        message->class_id, elapsed, reset_gap_sec_, message->x, message->y);
    }
  } else {
    filter_.transitionMatrix.at<float>(0, 2) = static_cast<float>(elapsed);
    filter_.transitionMatrix.at<float>(1, 3) = static_cast<float>(elapsed);
    filter_.predict();
    const cv::Mat measurement = (cv::Mat_<float>(2, 1) << message->x, message->y);
    corrected = filter_.correct(measurement);
  }
  last_target_time_ = now;

  auto filtered_target = *message;
  filtered_target.x = corrected.at<float>(0);
  filtered_target.y = corrected.at<float>(1);
  filtered_target_publisher_->publish(filtered_target);

  std_msgs::msg::String output;
  std::ostringstream payload;
  payload
    << "{\"source\":\"kalman_filter_node\""
    << ",\"class_id\":" << message->class_id
    << ",\"label\":\"" << message->label << "\""
    << ",\"confidence\":" << message->confidence
    << ",\"raw_x\":" << message->x << ",\"raw_y\":" << message->y
    << ",\"filtered_x\":" << corrected.at<float>(0)
    << ",\"filtered_y\":" << corrected.at<float>(1)
    << ",\"side_length\":" << message->side_length
    << "}";
  output.data = payload.str();
  filtered_detection_publisher_->publish(output);
  if (debug_mode_) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Kalman state class=%d raw=(%.2f,%.2f) filtered=(%.2f,%.2f) elapsed=%.3f",
      message->class_id, message->x, message->y, corrected.at<float>(0),
      corrected.at<float>(1), elapsed);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KalmanFilterNode>());
  rclcpp::shutdown();
  return 0;
}
