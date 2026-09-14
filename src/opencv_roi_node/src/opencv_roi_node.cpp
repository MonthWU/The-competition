#include "opencv_roi_node/opencv_roi_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace
{
struct ShapeMeasurement
{
  std::vector<cv::Point> contour;
  cv::Point2f center;
  float side_length;
  double area;
  double circularity;
  bool is_circle;
};

int normalize_odd_kernel(const int value)
{
  const auto positive = std::max(1, value);
  return positive % 2 == 0 ? positive + 1 : positive;
}

int normalize_aperture(const int value)
{
  return value <= 3 ? 3 : value <= 5 ? 5 : 7;
}

cv::Mat gamma_correct(const cv::Mat & input, const double gamma)
{
  cv::Mat lookup(1, 256, CV_8U);
  for (int index = 0; index < 256; ++index) {
    lookup.at<unsigned char>(index) = static_cast<unsigned char>(std::clamp(
      std::lround(std::pow(static_cast<double>(index) / 255.0, gamma) * 255.0), 0L, 255L));
  }
  cv::Mat output;
  cv::LUT(input, lookup, output);
  return output;
}

std::string measurement_text(const float side_length, const int decimals)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(decimals) << side_length << "px";
  return stream.str();
}
}  // namespace

OpenCvRoiNode::OpenCvRoiNode()
: Node("opencv_roi_node"),
  max_width_(960),
  roi_x_(0),
  roi_y_(0),
  roi_width_(100),
  roi_height_(100),
  blur_kernel_size_(3),
  illumination_kernel_size_(81),
  compensation_strength_(100.0),
  gamma_(0.8),
  threshold_(150),
  morph_kernel_size_(3),
  morph_iterations_(1),
  h_min_(35),
  h_max_(95),
  s_min_(50),
  s_max_(255),
  v_min_(30),
  v_max_(255),
  canny_low_threshold_(50.0),
  canny_high_threshold_(150.0),
  canny_aperture_(3),
  min_contour_area_(100.0),
  max_contour_area_(200000.0),
  contour_epsilon_(1.0),
  contour_thickness_(2),
  max_objects_(20),
  center_marker_size_(11),
  center_marker_thickness_(2),
  circularity_threshold_(0.82),
  measurement_decimals_(1),
  measurement_font_scale_(0.55),
  invert_(false),
  contour_source_("canny"),
  center_method_("pixel"),
  measurement_mode_("auto"),
  perspective_enabled_(false),
  publish_best_only_(true),
  publish_debug_image_(false),
  processed_roi_count_(0U),
  measurement_count_(0U),
  accumulated_processing_ms_(0.0),
  last_processing_ms_(0.0)
{
  roi_topic_ = declare_parameter<std::string>("roi_topic", "vision/roi");
  processed_roi_topic_ = declare_parameter<std::string>("processed_roi_topic", "vision/roi_opencv");
  target_topic_ = declare_parameter<std::string>("target_topic", "vision/target_raw");
  status_topic_ = declare_parameter<std::string>("status_topic", "vision/opencv_status");
  max_width_ = declare_parameter<int>("max_width", max_width_);
  roi_x_ = declare_parameter<int>("roi_x", roi_x_);
  roi_y_ = declare_parameter<int>("roi_y", roi_y_);
  roi_width_ = declare_parameter<int>("roi_width", roi_width_);
  roi_height_ = declare_parameter<int>("roi_height", roi_height_);
  blur_kernel_size_ = normalize_odd_kernel(declare_parameter<int>("denoise_kernel", blur_kernel_size_));
  illumination_kernel_size_ = normalize_odd_kernel(
    declare_parameter<int>("illumination_kernel", illumination_kernel_size_));
  compensation_strength_ = declare_parameter<double>("compensation_strength", compensation_strength_);
  gamma_ = declare_parameter<double>("gamma", gamma_);
  threshold_ = declare_parameter<int>("threshold", threshold_);
  morph_kernel_size_ = normalize_odd_kernel(declare_parameter<int>("morph_kernel", morph_kernel_size_));
  morph_iterations_ = declare_parameter<int>("morph_iterations", morph_iterations_);
  h_min_ = declare_parameter<int>("h_min", h_min_);
  h_max_ = declare_parameter<int>("h_max", h_max_);
  s_min_ = declare_parameter<int>("s_min", s_min_);
  s_max_ = declare_parameter<int>("s_max", s_max_);
  v_min_ = declare_parameter<int>("v_min", v_min_);
  v_max_ = declare_parameter<int>("v_max", v_max_);
  canny_low_threshold_ = declare_parameter<double>("canny_low", canny_low_threshold_);
  canny_high_threshold_ = declare_parameter<double>("canny_high", canny_high_threshold_);
  canny_aperture_ = normalize_aperture(declare_parameter<int>("canny_aperture", canny_aperture_));
  min_contour_area_ = declare_parameter<double>("contour_min_area", min_contour_area_);
  max_contour_area_ = declare_parameter<double>("contour_max_area", max_contour_area_);
  contour_epsilon_ = declare_parameter<double>("contour_epsilon", contour_epsilon_);
  contour_thickness_ = declare_parameter<int>("contour_thickness", contour_thickness_);
  max_objects_ = declare_parameter<int>("max_objects", max_objects_);
  center_marker_size_ = declare_parameter<int>("center_marker_size", center_marker_size_);
  center_marker_thickness_ = declare_parameter<int>("center_marker_thickness", center_marker_thickness_);
  circularity_threshold_ = declare_parameter<double>("circularity_threshold", circularity_threshold_);
  measurement_decimals_ = declare_parameter<int>("measurement_decimals", measurement_decimals_);
  measurement_font_scale_ = declare_parameter<double>("measurement_font_scale", measurement_font_scale_);
  invert_ = declare_parameter<bool>("invert", invert_);
  contour_source_ = declare_parameter<std::string>("contour_source", contour_source_);
  center_method_ = declare_parameter<std::string>("center_method", center_method_);
  measurement_mode_ = declare_parameter<std::string>("measurement_mode", measurement_mode_);
  perspective_enabled_ = declare_parameter<bool>("perspective_enabled", perspective_enabled_);
  perspective_homography_ = declare_parameter<std::vector<double>>(
    "perspective_homography",
    std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
  publish_best_only_ = declare_parameter<bool>("publish_best_only", publish_best_only_);
  publish_debug_image_ = declare_parameter<bool>("publish_debug_image", publish_debug_image_);
  debug_mode_ = declare_parameter<bool>("debug_mode", false);

  max_width_ = std::max(1, max_width_);
  roi_x_ = std::max(0, roi_x_);
  roi_y_ = std::max(0, roi_y_);
  compensation_strength_ = std::clamp(compensation_strength_, 0.0, 100.0);
  gamma_ = std::max(0.01, gamma_);
  threshold_ = std::clamp(threshold_, 0, 255);
  morph_iterations_ = std::max(0, morph_iterations_);
  h_min_ = std::clamp(h_min_, 0, 179);
  h_max_ = std::clamp(h_max_, h_min_, 179);
  s_min_ = std::clamp(s_min_, 0, 255);
  s_max_ = std::clamp(s_max_, s_min_, 255);
  v_min_ = std::clamp(v_min_, 0, 255);
  v_max_ = std::clamp(v_max_, v_min_, 255);
  canny_low_threshold_ = std::max(0.0, canny_low_threshold_);
  canny_high_threshold_ = std::max(canny_low_threshold_ + 1.0, canny_high_threshold_);
  min_contour_area_ = std::max(1.0, min_contour_area_);
  max_contour_area_ = std::max(min_contour_area_, max_contour_area_);
  contour_epsilon_ = std::max(0.1, contour_epsilon_);
  contour_thickness_ = std::max(1, contour_thickness_);
  max_objects_ = std::max(1, max_objects_);
  center_marker_size_ = std::max(1, center_marker_size_);
  center_marker_thickness_ = std::max(1, center_marker_thickness_);
  circularity_threshold_ = std::clamp(circularity_threshold_, 0.0, 1.0);
  measurement_decimals_ = std::clamp(measurement_decimals_, 0, 6);
  measurement_font_scale_ = std::max(0.1, measurement_font_scale_);
  if (perspective_enabled_ &&
    (perspective_homography_.size() != 9U ||
    !std::all_of(
      perspective_homography_.begin(), perspective_homography_.end(),
      [](const double value) {return std::isfinite(value);})))
  {
    throw std::runtime_error("perspective_homography must contain exactly 9 finite values");
  }

  auto qos = rclcpp::SensorDataQoS();
  qos.keep_last(1);
  roi_subscription_ = create_subscription<vision_interfaces::msg::Roi>(
    roi_topic_, qos, std::bind(&OpenCvRoiNode::on_roi, this, std::placeholders::_1));
  processed_roi_publisher_ = create_publisher<sensor_msgs::msg::Image>(processed_roi_topic_, qos);
  target_publisher_ = create_publisher<vision_interfaces::msg::TargetPoint>(target_topic_, 10);
  status_publisher_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);
  status_timer_ = create_wall_timer(
    std::chrono::seconds(1), std::bind(&OpenCvRoiNode::publish_status, this));
  RCLCPP_INFO(
    get_logger(), "OpenCV shape node ready: source=%s ROI=%d,%d,%d,%d canny=%.0f/%.0f",
    contour_source_.c_str(), roi_x_, roi_y_, roi_width_, roi_height_,
    canny_low_threshold_, canny_high_threshold_);
}

void OpenCvRoiNode::on_roi(const vision_interfaces::msg::Roi::ConstSharedPtr message)
{
  const auto processing_start = std::chrono::steady_clock::now();
  const auto & image = message->image;
  // BUG_POINT:ROI_FORMAT -- Reject malformed input before constructing cv::Mat.
  if (image.encoding != "bgr8" || image.step < image.width * 3U ||
    image.data.size() < static_cast<std::size_t>(image.step) * image.height)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "BUG_POINT:ROI_FORMAT ignored encoding=%s width=%u height=%u step=%u data=%zu",
      image.encoding.c_str(), image.width, image.height, image.step, image.data.size());
    return;
  }

  const cv::Mat input(
    static_cast<int>(image.height), static_cast<int>(image.width), CV_8UC3,
    const_cast<unsigned char *>(image.data.data()), image.step);
  const auto scale = input.cols > max_width_ ? static_cast<double>(max_width_) / input.cols : 1.0;
  cv::Mat analysis_frame;
  if (scale < 1.0) {
    cv::resize(input, analysis_frame, cv::Size(), scale, scale, cv::INTER_AREA);
  } else {
    // The YOLO node already copied this small ROI into its message. Keep a
    // zero-copy cv::Mat view in work mode and clone only for debug annotation.
    analysis_frame = input;
  }
  const auto crop_width = roi_width_ > 0 ? roi_width_ : analysis_frame.cols - roi_x_;
  const auto crop_height = roi_height_ > 0 ? roi_height_ : analysis_frame.rows - roi_y_;
  const cv::Rect requested_roi(roi_x_, roi_y_, crop_width, crop_height);
  const cv::Rect image_bounds(0, 0, analysis_frame.cols, analysis_frame.rows);
  const cv::Rect analysis_roi = requested_roi & image_bounds;
  if (analysis_roi.width <= 1 || analysis_roi.height <= 1) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "BUG_POINT:TUNER_ROI invalid ROI=%d,%d,%d,%d for image=%dx%d",
      roi_x_, roi_y_, crop_width, crop_height, analysis_frame.cols, analysis_frame.rows);
    return;
  }

  const cv::Mat analysis_bgr = analysis_frame(analysis_roi);
  cv::Mat grayscale;
  cv::Mat denoised;
  cv::Mat illumination;
  cv::Mat compensated;
  cv::Mat corrected;
  cv::cvtColor(analysis_bgr, grayscale, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(grayscale, denoised, cv::Size(blur_kernel_size_, blur_kernel_size_), 0.0);
  cv::morphologyEx(
    denoised, illumination, cv::MORPH_OPEN,
    cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(illumination_kernel_size_, illumination_kernel_size_)));
  cv::addWeighted(
    denoised, 1.0, illumination, -compensation_strength_ / 100.0,
    128.0 * compensation_strength_ / 100.0, compensated);
  corrected = gamma_correct(compensated, gamma_);

  cv::Mat threshold_mask;
  cv::threshold(
    corrected, threshold_mask, threshold_, 255, invert_ ? cv::THRESH_BINARY_INV : cv::THRESH_BINARY);
  cv::Mat hsv;
  cv::Mat hsv_mask;
  cv::cvtColor(analysis_bgr, hsv, cv::COLOR_BGR2HSV);
  cv::inRange(hsv, cv::Scalar(h_min_, s_min_, v_min_), cv::Scalar(h_max_, s_max_, v_max_), hsv_mask);
  cv::Mat canny_mask;
  cv::Canny(corrected, canny_mask, canny_low_threshold_, canny_high_threshold_, canny_aperture_);

  cv::Mat contour_mask;
  if (contour_source_ == "threshold") {
    contour_mask = threshold_mask;
  } else if (contour_source_ == "hsv") {
    contour_mask = hsv_mask;
  } else {
    contour_mask = canny_mask;
  }
  if (morph_iterations_ > 0) {
    const auto morph_kernel = cv::getStructuringElement(
      cv::MORPH_ELLIPSE, cv::Size(morph_kernel_size_, morph_kernel_size_));
    cv::morphologyEx(contour_mask, contour_mask, cv::MORPH_CLOSE, morph_kernel, cv::Point(-1, -1), morph_iterations_);
  }

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(contour_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  std::vector<ShapeMeasurement> measurements;
  for (const auto & contour : contours) {
    const auto area = std::abs(cv::contourArea(contour));
    if (area < min_contour_area_ || area > max_contour_area_) {
      continue;
    }
    const auto perimeter = cv::arcLength(contour, true);
    if (perimeter <= std::numeric_limits<double>::epsilon()) {
      continue;
    }
    std::vector<cv::Point> polygon;
    cv::approxPolyDP(contour, polygon, contour_epsilon_, true);
    const auto circularity = 4.0 * CV_PI * area / (perimeter * perimeter);
    const auto use_circle = measurement_mode_ == "circle" ||
      (measurement_mode_ == "auto" && circularity >= circularity_threshold_);
    ShapeMeasurement measurement;
    measurement.contour = contour;
    measurement.area = area;
    measurement.circularity = circularity;
    measurement.is_circle = use_circle;
    if (use_circle) {
      float radius = 0.0F;
      cv::minEnclosingCircle(contour, measurement.center, radius);
      measurement.side_length = 2.0F * radius;
    } else {
      const auto rectangle = cv::minAreaRect(contour);
      if (center_method_ == "pixel") {
        const auto moments = cv::moments(contour);
        measurement.center = std::abs(moments.m00) > std::numeric_limits<double>::epsilon() ?
          cv::Point2f(static_cast<float>(moments.m10 / moments.m00), static_cast<float>(moments.m01 / moments.m00)) :
          rectangle.center;
      } else {
        measurement.center = rectangle.center;
      }
      measurement.side_length = (rectangle.size.width + rectangle.size.height) * 0.5F;
    }
    if (measurement.side_length > 0.0F) {
      measurements.push_back(std::move(measurement));
    }
  }
  std::sort(measurements.begin(), measurements.end(), [](const auto & left, const auto & right) {
    return left.area > right.area;
  });
  if (measurements.size() > static_cast<std::size_t>(max_objects_)) {
    measurements.resize(static_cast<std::size_t>(max_objects_));
  }

  const auto publish_count = publish_best_only_ ? std::min<std::size_t>(1U, measurements.size()) :
    measurements.size();
  cv::Mat debug_frame;
  if (publish_debug_image_) {
    debug_frame = analysis_frame.clone();
  }
  for (std::size_t index = 0; index < publish_count; ++index) {
    const auto & measurement = measurements[index];
    const cv::Point2f display_center = measurement.center + cv::Point2f(
      static_cast<float>(analysis_roi.x), static_cast<float>(analysis_roi.y));
    if (publish_debug_image_) {
      cv::drawContours(debug_frame, std::vector<std::vector<cv::Point>>{measurement.contour}, -1,
        cv::Scalar(0, 255, 0), contour_thickness_, cv::LINE_AA, cv::noArray(),
        std::numeric_limits<int>::max(), analysis_roi.tl());
      cv::drawMarker(debug_frame, display_center, cv::Scalar(0, 0, 255), cv::MARKER_CROSS,
        center_marker_size_, center_marker_thickness_, cv::LINE_AA);
      cv::putText(
        debug_frame,
        measurement_text(measurement.side_length / static_cast<float>(scale), measurement_decimals_),
        display_center + cv::Point2f(6.0F, -6.0F), cv::FONT_HERSHEY_SIMPLEX,
        measurement_font_scale_, cv::Scalar(0, 255, 0), contour_thickness_, cv::LINE_AA);
    }

    cv::Point2f absolute_center(
      static_cast<float>(message->x_min) + display_center.x / static_cast<float>(scale),
      static_cast<float>(message->y_min) + display_center.y / static_cast<float>(scale));
    float corrected_side_length = measurement.side_length / static_cast<float>(scale);
    if (perspective_enabled_) {
      const cv::Mat homography(3, 3, CV_64F, perspective_homography_.data());
      const auto half_side = corrected_side_length * 0.5F;
      const std::vector<cv::Point2f> source_points{
        absolute_center,
        absolute_center + cv::Point2f(half_side, 0.0F),
        absolute_center + cv::Point2f(0.0F, half_side)};
      std::vector<cv::Point2f> transformed_points;
      cv::perspectiveTransform(source_points, transformed_points, homography);
      const auto transformed_points_valid = transformed_points.size() == 3U &&
        std::all_of(
        transformed_points.begin(), transformed_points.end(), [](const cv::Point2f & point) {
          return std::isfinite(point.x) && std::isfinite(point.y);
        });
      if (!transformed_points_valid)
      {
        // BUG_POINT:PERSPECTIVE_TRANSFORM -- A singular or mismatched matrix
        // must not publish invalid geometry into the control chain.
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "BUG_POINT:PERSPECTIVE_TRANSFORM rejected non-finite transformed target");
        continue;
      }
      corrected_side_length = cv::norm(transformed_points[1] - transformed_points[0]) +
        cv::norm(transformed_points[2] - transformed_points[0]);
      absolute_center = transformed_points[0];
    }

    vision_interfaces::msg::TargetPoint target;
    target.header = image.header;
    target.class_id = message->class_id;
    target.label = message->label;
    target.confidence = message->confidence;
    target.x = absolute_center.x;
    target.y = absolute_center.y;
    target.side_length = corrected_side_length;
    target_publisher_->publish(target);
  }
  if (measurements.empty() && debug_mode_) {
    // BUG_POINT:SHAPE_CONTOUR -- A YOLO ROI may not contain a valid target shape.
    RCLCPP_DEBUG(
      get_logger(), "BUG_POINT:SHAPE_CONTOUR no contour source=%s ROI=%dx%d",
      contour_source_.c_str(), analysis_roi.width, analysis_roi.height);
  }

  if (publish_debug_image_) {
    cv::Mat output_frame;
    if (scale < 1.0) {
      cv::resize(debug_frame, output_frame, input.size(), 0.0, 0.0, cv::INTER_LINEAR);
    } else {
      output_frame = debug_frame;
    }
    sensor_msgs::msg::Image output = image;
    output.step = static_cast<sensor_msgs::msg::Image::_step_type>(
      output_frame.cols * output_frame.elemSize());
    output.data.resize(static_cast<std::size_t>(output.step) * output.height);
    std::memcpy(output.data.data(), output_frame.data, output.data.size());
    processed_roi_publisher_->publish(output);
  }

  ++processed_roi_count_;
  measurement_count_ += publish_count;
  last_processing_ms_ = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - processing_start).count();
  accumulated_processing_ms_ += last_processing_ms_;
  if (debug_mode_ && !measurements.empty()) {
    const auto & first = measurements.front();
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Shape measurements=%zu center=(%.1f,%.1f) side=%.1fpx circularity=%.3f",
      measurements.size(), first.center.x / scale, first.center.y / scale,
      first.side_length / scale, first.circularity);
  }
}

void OpenCvRoiNode::publish_status()
{
  std_msgs::msg::String status;
  std::ostringstream payload;
  const auto average_ms = processed_roi_count_ > 0U ?
    accumulated_processing_ms_ / static_cast<double>(processed_roi_count_) : 0.0;
  payload << "{\"node\":\"opencv_roi_node\""
          << ",\"processed_rois\":" << processed_roi_count_
          << ",\"measurements\":" << measurement_count_
          << ",\"last_processing_ms\":" << last_processing_ms_
          << ",\"average_processing_ms\":" << average_ms
          << "}";
  status.data = payload.str();
  status_publisher_->publish(status);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OpenCvRoiNode>());
  rclcpp::shutdown();
  return 0;
}
