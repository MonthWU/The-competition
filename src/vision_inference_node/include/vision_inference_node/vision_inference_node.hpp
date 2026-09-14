#ifndef VISION_INFERENCE_NODE__VISION_INFERENCE_NODE_HPP_
#define VISION_INFERENCE_NODE__VISION_INFERENCE_NODE_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <vision_interfaces/msg/roi.hpp>

#include "vision_inference_node/camera_source.hpp"
#include "vision_inference_node/inference_metrics.hpp"
#include "vision_inference_node/tensorrt_detector.hpp"

class VisionInferenceNode : public rclcpp::Node
{
public:
  VisionInferenceNode();
  ~VisionInferenceNode() override;

private:
  struct LatestFrame
  {
    cv::Mat image;
    std_msgs::msg::Header header;
    std::uint64_t sequence{0U};
  };

  bool open_camera();
  void capture_loop();
  void inference_loop();
  void publish_camera_status();
  void publish_inference_status();
  void publish_results(
    const std_msgs::msg::Header & header,
    const std::vector<vision_inference_node::Detection> & detections,
    const vision_inference_node::InferenceMetrics & metrics);
  void publish_rois(
    const cv::Mat & bgr_frame,
    const std_msgs::msg::Header & header,
    const std::vector<vision_inference_node::Detection> & detections);
  void publish_debug_image(
    const cv::Mat & bgr_frame,
    const std_msgs::msg::Header & header,
    const std::vector<vision_inference_node::Detection> & detections);
  std::string class_label(int class_id) const;

  rclcpp::Publisher<vision_interfaces::msg::Roi>::SharedPtr roi_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr camera_status_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr inference_status_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_publisher_;
  rclcpp::TimerBase::SharedPtr camera_status_timer_;
  rclcpp::TimerBase::SharedPtr inference_status_timer_;

  std::unique_ptr<vision_inference_node::CameraSource> camera_source_;
  std::unique_ptr<vision_inference_node::TensorRtDetector> detector_;
  std::thread capture_thread_;
  std::thread inference_thread_;
  std::atomic<bool> stopping_{false};
  mutable std::mutex latest_frame_mutex_;
  std::condition_variable latest_frame_condition_;
  LatestFrame latest_frame_;

  std::string camera_type_;
  std::string camera_device_;
  std::string camera_pipeline_;
  std::string camera_fourcc_;
  std::string frame_id_;
  std::string engine_path_;
  std::string roi_topic_;
  std::string result_topic_;
  std::string camera_status_topic_;
  std::string inference_status_topic_;
  std::string debug_image_topic_;
  std::vector<std::string> class_names_;
  int sensor_id_{0};
  int input_width_{1280};
  int input_height_{720};
  double frame_rate_{120.0};
  int capture_buffer_size_{1};
  int reconnect_interval_ms_{2000};
  double status_interval_sec_{1.0};
  int network_input_size_{640};
  float confidence_threshold_{0.25F};
  float iou_threshold_{0.45F};
  int max_detections_{10};
  bool publish_debug_image_{false};
  double debug_publish_fps_{5.0};
  bool debug_mode_{false};

  std::atomic<bool> camera_open_{false};
  std::atomic<double> actual_width_{0.0};
  std::atomic<double> actual_height_{0.0};
  std::atomic<double> actual_fps_{0.0};
  std::atomic<std::uint64_t> captured_frames_{0U};
  std::atomic<std::uint64_t> capture_errors_{0U};
  std::atomic<double> measured_capture_fps_{0.0};
  std::atomic<double> last_capture_ms_{0.0};
  std::atomic<std::uint64_t> inferred_frames_{0U};
  std::atomic<std::uint64_t> dropped_frames_{0U};
  std::atomic<std::uint64_t> inference_errors_{0U};
  std::atomic<double> measured_inference_fps_{0.0};
  std::atomic<double> last_upload_preprocess_ms_{0.0};
  std::atomic<double> last_tensorrt_ms_{0.0};
  std::atomic<double> last_download_decode_ms_{0.0};
  std::atomic<double> last_total_ms_{0.0};
  std::atomic<int> last_detection_count_{0};
  vision_inference_node::LatencyWindow latency_window_{256U};
  std::chrono::steady_clock::time_point last_debug_publish_;
};

#endif  // VISION_INFERENCE_NODE__VISION_INFERENCE_NODE_HPP_
