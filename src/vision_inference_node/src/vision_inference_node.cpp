#include "vision_inference_node/vision_inference_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <opencv2/imgproc.hpp>

namespace
{
constexpr const char * kBugPointCameraOpen = "BUG_POINT:CAMERA_OPEN";
constexpr const char * kBugPointFrameRead = "BUG_POINT:FRAME_READ";

std::string expand_home(std::string path)
{
  if (!path.empty() && path.front() == '~') {
    const char * home = std::getenv("HOME");
    if (home == nullptr) {
      throw std::runtime_error("BUG_POINT:ENGINE_LOAD HOME is unset");
    }
    path.replace(0, 1, home);
  }
  return path;
}

std::string json_escape(const std::string & value)
{
  std::ostringstream output;
  for (const char character : value) {
    switch (character) {
      case '\\': output << "\\\\"; break;
      case '"': output << "\\\""; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default: output << character; break;
    }
  }
  return output.str();
}

sensor_msgs::msg::Image bgr_message(
  const cv::Mat & frame, const std_msgs::msg::Header & header, const std::string & frame_id)
{
  const cv::Mat contiguous = frame.isContinuous() ? frame : frame.clone();
  sensor_msgs::msg::Image image;
  image.header = header;
  image.header.frame_id = frame_id;
  image.height = static_cast<std::uint32_t>(contiguous.rows);
  image.width = static_cast<std::uint32_t>(contiguous.cols);
  image.encoding = "bgr8";
  image.is_bigendian = false;
  image.step = static_cast<sensor_msgs::msg::Image::_step_type>(contiguous.cols * 3);
  image.data.resize(static_cast<std::size_t>(image.step) * image.height);
  std::memcpy(image.data.data(), contiguous.data, image.data.size());
  return image;
}
}  // namespace

VisionInferenceNode::VisionInferenceNode()
: Node("vision_inference_node"),
  last_debug_publish_(std::chrono::steady_clock::now())
{
  camera_type_ = declare_parameter<std::string>("camera_type", "usb");
  camera_device_ = declare_parameter<std::string>("camera_device", "/dev/video0");
  camera_pipeline_ = declare_parameter<std::string>("camera_pipeline", "");
  camera_fourcc_ = declare_parameter<std::string>("camera_fourcc", "YUYV");
  sensor_id_ = declare_parameter<int>("sensor_id", 0);
  frame_id_ = declare_parameter<std::string>("frame_id", "camera_optical_frame");
  input_width_ = declare_parameter<int>("input_width", 1280);
  input_height_ = declare_parameter<int>("input_height", 720);
  frame_rate_ = declare_parameter<double>("frame_rate", 120.0);
  capture_buffer_size_ = declare_parameter<int>("capture_buffer_size", 1);
  reconnect_interval_ms_ = declare_parameter<int>("reconnect_interval_ms", 2000);
  status_interval_sec_ = declare_parameter<double>("status_interval_sec", 1.0);
  engine_path_ = expand_home(declare_parameter<std::string>(
      "engine_path",
      "~/ProjectsByMonthWU/VisionJetson/Copyfiles/models/e_project/2025e-yolo26n_fp16.engine"));
  network_input_size_ = declare_parameter<int>("network_input_size", 640);
  confidence_threshold_ = static_cast<float>(
    declare_parameter<double>("confidence_threshold", 0.25));
  iou_threshold_ = static_cast<float>(declare_parameter<double>("iou_threshold", 0.45));
  max_detections_ = declare_parameter<int>("max_detections", 10);
  class_names_ = declare_parameter<std::vector<std::string>>(
    "class_names", std::vector<std::string>{"blank"});
  roi_topic_ = declare_parameter<std::string>("roi_topic", "vision/roi");
  result_topic_ = declare_parameter<std::string>("result_topic", "vision/detections");
  camera_status_topic_ = declare_parameter<std::string>(
    "camera_status_topic", "vision/camera_status");
  inference_status_topic_ = declare_parameter<std::string>(
    "inference_status_topic", "vision/yolo_status");
  debug_image_topic_ = declare_parameter<std::string>(
    "debug_image_topic", "vision/image_debug");
  publish_debug_image_ = declare_parameter<bool>("publish_debug_image", false);
  debug_publish_fps_ = declare_parameter<double>("debug_publish_fps", 5.0);
  debug_mode_ = declare_parameter<bool>("debug_mode", false);

  input_width_ = std::max(2, input_width_);
  input_height_ = std::max(1, input_height_);
  frame_rate_ = std::max(0.1, frame_rate_);
  capture_buffer_size_ = std::max(1, capture_buffer_size_);
  reconnect_interval_ms_ = std::max(100, reconnect_interval_ms_);
  status_interval_sec_ = std::max(0.2, status_interval_sec_);
  network_input_size_ = std::max(32, network_input_size_);
  confidence_threshold_ = std::clamp(confidence_threshold_, 0.0F, 1.0F);
  iou_threshold_ = std::clamp(iou_threshold_, 0.0F, 1.0F);
  max_detections_ = std::max(1, max_detections_);
  debug_publish_fps_ = std::max(0.1, debug_publish_fps_);
  if (camera_fourcc_.size() != 4U) {
    throw std::runtime_error("camera_fourcc must contain exactly four characters");
  }
  if (input_width_ % 2 != 0) {
    throw std::runtime_error("YUYV input_width must be even");
  }

  vision_inference_node::CameraConfig camera_config;
  camera_config.type = camera_type_;
  camera_config.device = camera_device_;
  camera_config.pipeline = camera_pipeline_;
  camera_config.fourcc = camera_fourcc_;
  camera_config.sensor_id = sensor_id_;
  camera_config.width = input_width_;
  camera_config.height = input_height_;
  camera_config.frame_rate = frame_rate_;
  camera_config.buffer_size = capture_buffer_size_;
  camera_source_ = std::make_unique<vision_inference_node::CameraSource>(
    std::move(camera_config));

  detector_ = std::make_unique<vision_inference_node::TensorRtDetector>(
    engine_path_, network_input_size_, confidence_threshold_, iou_threshold_, max_detections_);

  auto sensor_qos = rclcpp::SensorDataQoS();
  sensor_qos.keep_last(1);
  roi_publisher_ = create_publisher<vision_interfaces::msg::Roi>(roi_topic_, sensor_qos);
  result_publisher_ = create_publisher<std_msgs::msg::String>(result_topic_, 10);
  camera_status_publisher_ = create_publisher<std_msgs::msg::String>(camera_status_topic_, 10);
  inference_status_publisher_ = create_publisher<std_msgs::msg::String>(
    inference_status_topic_, 10);
  debug_image_publisher_ = create_publisher<sensor_msgs::msg::Image>(
    debug_image_topic_, sensor_qos);
  camera_status_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(status_interval_sec_)),
    std::bind(&VisionInferenceNode::publish_camera_status, this));
  inference_status_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(status_interval_sec_)),
    std::bind(&VisionInferenceNode::publish_inference_status, this));

  capture_thread_ = std::thread(&VisionInferenceNode::capture_loop, this);
  inference_thread_ = std::thread(&VisionInferenceNode::inference_loop, this);
  RCLCPP_INFO(
    get_logger(),
    "C++ same-process camera/TensorRT ready: engine=%s %s camera=%dx%d@%.1f",
    engine_path_.c_str(), detector_->binding_summary().c_str(),
    input_width_, input_height_, frame_rate_);
}

VisionInferenceNode::~VisionInferenceNode()
{
  stopping_.store(true);
  latest_frame_condition_.notify_all();
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
  if (inference_thread_.joinable()) {
    inference_thread_.join();
  }
}

bool VisionInferenceNode::open_camera()
{
  camera_source_->close();
  if (!camera_source_->open()) {
    camera_open_.store(false);
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "%s type=%s input=%s",
      kBugPointCameraOpen, camera_type_.c_str(), camera_source_->input_description().c_str());
    return false;
  }
  actual_width_.store(camera_source_->actual_width());
  actual_height_.store(camera_source_->actual_height());
  actual_fps_.store(camera_source_->actual_fps());
  camera_open_.store(true);
  RCLCPP_INFO(get_logger(), "Camera opened for same-process inference");
  return true;
}

void VisionInferenceNode::capture_loop()
{
  std::uint64_t fps_frames = 0U;
  auto fps_started = std::chrono::steady_clock::now();
  while (!stopping_.load()) {
    if (!camera_source_->is_opened() && !open_camera()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_interval_ms_));
      continue;
    }
    const auto started = std::chrono::steady_clock::now();
    cv::Mat frame;
    if (!camera_source_->read(frame) || frame.empty()) {
      capture_errors_.fetch_add(1U);
      camera_open_.store(false);
      camera_source_->close();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "%s camera returned an empty frame",
        kBugPointFrameRead);
      continue;
    }
    last_capture_ms_.store(std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count());
    if (frame.type() != CV_8UC2) {
      capture_errors_.fetch_add(1U);
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "BUG_POINT:YUYV_FRAME_FORMAT expected CV_8UC2, received type=%d", frame.type());
      continue;
    }
    if (!frame.isContinuous()) {
      frame = frame.clone();
    }
    LatestFrame newest;
    newest.image = frame.clone();
    newest.header.stamp = now();
    newest.header.frame_id = frame_id_;
    newest.sequence = captured_frames_.fetch_add(1U) + 1U;
    {
      std::lock_guard<std::mutex> lock(latest_frame_mutex_);
      latest_frame_ = std::move(newest);
    }
    latest_frame_condition_.notify_one();

    ++fps_frames;
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - fps_started).count();
    if (elapsed >= 1.0) {
      measured_capture_fps_.store(static_cast<double>(fps_frames) / elapsed);
      fps_frames = 0U;
      fps_started = std::chrono::steady_clock::now();
    }
  }
  camera_source_->close();
  camera_open_.store(false);
}

void VisionInferenceNode::inference_loop()
{
  std::uint64_t consumed_sequence = 0U;
  std::uint64_t fps_frames = 0U;
  auto fps_started = std::chrono::steady_clock::now();
  while (!stopping_.load()) {
    LatestFrame frame;
    {
      std::unique_lock<std::mutex> lock(latest_frame_mutex_);
      latest_frame_condition_.wait_for(
        lock, std::chrono::milliseconds(500), [&]() {
          return stopping_.load() || latest_frame_.sequence > consumed_sequence;
        });
      if (stopping_.load()) {
        break;
      }
      if (latest_frame_.sequence <= consumed_sequence) {
        continue;
      }
      frame = latest_frame_;
    }
    if (consumed_sequence != 0U && frame.sequence > consumed_sequence + 1U) {
      dropped_frames_.fetch_add(frame.sequence - consumed_sequence - 1U);
    }
    consumed_sequence = frame.sequence;

    try {
      vision_inference_node::InferenceMetrics metrics;
      const auto detections = detector_->infer(frame.image, metrics);
      last_upload_preprocess_ms_.store(metrics.upload_preprocess_ms);
      last_tensorrt_ms_.store(metrics.tensorrt_ms);
      last_download_decode_ms_.store(metrics.download_decode_ms);
      last_total_ms_.store(metrics.total_ms);
      last_detection_count_.store(static_cast<int>(detections.size()));
      inferred_frames_.fetch_add(1U);
      latency_window_.record(metrics.total_ms);
      publish_results(frame.header, detections, metrics);

      const bool debug_due = publish_debug_image_ &&
        std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_debug_publish_).count() >=
        1.0 / debug_publish_fps_;
      if (!detections.empty() || debug_due) {
        cv::Mat bgr;
        cv::cvtColor(frame.image, bgr, cv::COLOR_YUV2BGR_YUY2);
        if (!detections.empty()) {
          publish_rois(bgr, frame.header, detections);
        }
        if (debug_due) {
          publish_debug_image(bgr, frame.header, detections);
          last_debug_publish_ = std::chrono::steady_clock::now();
        }
      }
    } catch (const std::exception & error) {
      inference_errors_.fetch_add(1U);
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "BUG_POINT:INFERENCE_RUNTIME %s", error.what());
    }

    ++fps_frames;
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - fps_started).count();
    if (elapsed >= 1.0) {
      measured_inference_fps_.store(static_cast<double>(fps_frames) / elapsed);
      fps_frames = 0U;
      fps_started = std::chrono::steady_clock::now();
    }
  }
}

void VisionInferenceNode::publish_results(
  const std_msgs::msg::Header & header,
  const std::vector<vision_inference_node::Detection> & detections,
  const vision_inference_node::InferenceMetrics & metrics)
{
  std_msgs::msg::String message;
  std::ostringstream json;
  json << std::fixed << std::setprecision(3)
       << "{\"source\":\"vision_inference_node\",\"backend\":\"TensorRT_CPP\""
       << ",\"stamp_sec\":" << header.stamp.sec
       << ",\"stamp_nanosec\":" << header.stamp.nanosec
       << ",\"preprocess_ms\":" << metrics.upload_preprocess_ms
       << ",\"tensorrt_ms\":" << metrics.tensorrt_ms
       << ",\"total_ms\":" << metrics.total_ms
       << ",\"detections\":[";
  for (std::size_t index = 0; index < detections.size(); ++index) {
    const auto & detection = detections[index];
    if (index != 0U) {
      json << ",";
    }
    json << "{\"class_id\":" << detection.class_id
         << ",\"label\":\"" << json_escape(class_label(detection.class_id)) << "\""
         << ",\"confidence\":" << detection.confidence
         << ",\"bbox_xyxy\":[" << detection.x_min << "," << detection.y_min
         << "," << detection.x_max << "," << detection.y_max << "]}";
  }
  json << "]}";
  message.data = json.str();
  result_publisher_->publish(message);
}

void VisionInferenceNode::publish_rois(
  const cv::Mat & bgr_frame,
  const std_msgs::msg::Header & header,
  const std::vector<vision_inference_node::Detection> & detections)
{
  const cv::Rect bounds(0, 0, bgr_frame.cols, bgr_frame.rows);
  for (std::size_t index = 0; index < detections.size(); ++index) {
    const auto & detection = detections[index];
    const int x_min = static_cast<int>(std::floor(detection.x_min));
    const int y_min = static_cast<int>(std::floor(detection.y_min));
    const int x_max = static_cast<int>(std::ceil(detection.x_max));
    const int y_max = static_cast<int>(std::ceil(detection.y_max));
    const cv::Rect roi = cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min) & bounds;
    if (roi.width <= 1 || roi.height <= 1) {
      continue;
    }
    vision_interfaces::msg::Roi message;
    message.image = bgr_message(
      bgr_frame(roi), header,
      header.frame_id + "/roi_" + std::to_string(index) + "_class_" +
      std::to_string(detection.class_id));
    message.class_id = detection.class_id;
    message.label = class_label(detection.class_id);
    message.confidence = detection.confidence;
    message.x_min = roi.x;
    message.y_min = roi.y;
    message.x_max = roi.x + roi.width;
    message.y_max = roi.y + roi.height;
    message.center_x = static_cast<float>(roi.x) + static_cast<float>(roi.width) * 0.5F;
    message.center_y = static_cast<float>(roi.y) + static_cast<float>(roi.height) * 0.5F;
    roi_publisher_->publish(message);
  }
}

void VisionInferenceNode::publish_debug_image(
  const cv::Mat & bgr_frame,
  const std_msgs::msg::Header & header,
  const std::vector<vision_inference_node::Detection> & detections)
{
  cv::Mat annotated = bgr_frame.clone();
  for (const auto & detection : detections) {
    const cv::Point top_left(
      static_cast<int>(std::lround(detection.x_min)),
      static_cast<int>(std::lround(detection.y_min)));
    const cv::Point bottom_right(
      static_cast<int>(std::lround(detection.x_max)),
      static_cast<int>(std::lround(detection.y_max)));
    cv::rectangle(annotated, top_left, bottom_right, cv::Scalar(0, 255, 0), 2);
    std::ostringstream label;
    label << class_label(detection.class_id) << " " << std::fixed << std::setprecision(2)
          << detection.confidence;
    cv::putText(
      annotated, label.str(), top_left + cv::Point(0, -4), cv::FONT_HERSHEY_SIMPLEX,
      0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
  }
  debug_image_publisher_->publish(
    bgr_message(annotated, header, header.frame_id + "/debug"));
}

std::string VisionInferenceNode::class_label(const int class_id) const
{
  if (class_id >= 0 && class_id < static_cast<int>(class_names_.size())) {
    return class_names_[static_cast<std::size_t>(class_id)];
  }
  return "class_" + std::to_string(class_id);
}

void VisionInferenceNode::publish_camera_status()
{
  std_msgs::msg::String message;
  std::ostringstream json;
  json << "{\"node\":\"vision_inference_node\",\"component\":\"camera\""
       << ",\"status\":\"" << (camera_open_.load() ? "open" : "closed") << "\""
       << ",\"frames\":" << captured_frames_.load()
       << ",\"capture_errors\":" << capture_errors_.load()
       << ",\"configured_width\":" << input_width_
       << ",\"configured_height\":" << input_height_
       << ",\"configured_fps\":" << frame_rate_
       << ",\"configured_fourcc\":\"" << camera_fourcc_ << "\""
       << ",\"actual_width\":" << actual_width_.load()
       << ",\"actual_height\":" << actual_height_.load()
       << ",\"actual_fps\":" << actual_fps_.load()
       << ",\"measured_fps\":" << measured_capture_fps_.load()
       << ",\"last_capture_ms\":" << last_capture_ms_.load()
       << ",\"output_encoding\":\"internal_yuyv\"}";
  message.data = json.str();
  camera_status_publisher_->publish(message);
}

void VisionInferenceNode::publish_inference_status()
{
  const auto percentiles = latency_window_.percentiles();
  std_msgs::msg::String message;
  std::ostringstream json;
  json << "{\"node\":\"vision_inference_node\",\"component\":\"yolo\""
       << ",\"backend\":\"TensorRT_CPP\""
       << ",\"binding\":\"" << json_escape(detector_->binding_summary()) << "\""
       << ",\"inference_frames\":" << inferred_frames_.load()
       << ",\"latest_frame_drops\":" << dropped_frames_.load()
       << ",\"errors\":" << inference_errors_.load()
       << ",\"measured_fps\":" << measured_inference_fps_.load()
       << ",\"last_preprocess_ms\":" << last_upload_preprocess_ms_.load()
       << ",\"last_tensorrt_ms\":" << last_tensorrt_ms_.load()
       << ",\"last_download_decode_ms\":" << last_download_decode_ms_.load()
       << ",\"last_total_ms\":" << last_total_ms_.load()
       << ",\"total_p50_ms\":" << percentiles.p50_ms
       << ",\"total_p95_ms\":" << percentiles.p95_ms
       << ",\"last_detections\":" << last_detection_count_.load() << "}";
  message.data = json.str();
  inference_status_publisher_->publish(message);
  if (debug_mode_) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 1000, "BUG_POINT:INFERENCE_STATUS %s",
      message.data.c_str());
  }
}
