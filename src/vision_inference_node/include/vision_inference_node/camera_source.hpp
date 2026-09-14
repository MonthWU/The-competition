#ifndef VISION_INFERENCE_NODE__CAMERA_SOURCE_HPP_
#define VISION_INFERENCE_NODE__CAMERA_SOURCE_HPP_

#include <string>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

namespace vision_inference_node
{

struct CameraConfig
{
  std::string type{"usb"};
  std::string device{"/dev/video0"};
  std::string pipeline;
  std::string fourcc{"YUYV"};
  int sensor_id{0};
  int width{1280};
  int height{720};
  double frame_rate{120.0};
  int buffer_size{1};
};

class CameraSource
{
public:
  explicit CameraSource(CameraConfig config);

  bool open();
  bool read(cv::Mat & frame);
  void close();
  bool is_opened() const;
  std::string input_description() const;
  double actual_width() const;
  double actual_height() const;
  double actual_fps() const;

private:
  std::string camera_input() const;

  CameraConfig config_;
  cv::VideoCapture capture_;
};

}  // namespace vision_inference_node

#endif  // VISION_INFERENCE_NODE__CAMERA_SOURCE_HPP_
