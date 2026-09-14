#include "vision_inference_node/camera_source.hpp"

#include <sstream>
#include <stdexcept>
#include <utility>

namespace vision_inference_node
{

CameraSource::CameraSource(CameraConfig config)
: config_(std::move(config))
{
  if (config_.fourcc.size() != 4U) {
    throw std::invalid_argument("camera fourcc must contain exactly four characters");
  }
}

std::string CameraSource::camera_input() const
{
  if (!config_.pipeline.empty()) {
    return config_.pipeline;
  }
  if (config_.type == "csi") {
    std::ostringstream pipeline;
    pipeline << "nvarguscamerasrc sensor-id=" << config_.sensor_id
             << " ! video/x-raw(memory:NVMM),width=(int)" << config_.width
             << ",height=(int)" << config_.height
             << ",framerate=(fraction)" << static_cast<int>(config_.frame_rate) << "/1"
             << ",format=(string)NV12 ! nvvidconv ! video/x-raw,format=(string)YUY2"
             << " ! appsink max-buffers=1 drop=true sync=false";
    return pipeline.str();
  }
  return config_.device;
}

bool CameraSource::open()
{
  close();
  const auto backend = config_.type == "csi" || !config_.pipeline.empty() ?
    cv::CAP_GSTREAMER : cv::CAP_V4L2;
  if (!capture_.open(camera_input(), backend)) {
    return false;
  }
  if (backend == cv::CAP_V4L2) {
    capture_.set(cv::CAP_PROP_BUFFERSIZE, config_.buffer_size);
    capture_.set(
      cv::CAP_PROP_FOURCC,
      cv::VideoWriter::fourcc(
        config_.fourcc[0], config_.fourcc[1], config_.fourcc[2], config_.fourcc[3]));
    capture_.set(cv::CAP_PROP_FRAME_WIDTH, config_.width);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, config_.height);
    capture_.set(cv::CAP_PROP_FPS, config_.frame_rate);
    capture_.set(cv::CAP_PROP_CONVERT_RGB, 0.0);
  }
  return true;
}

bool CameraSource::read(cv::Mat & frame)
{
  return capture_.read(frame);
}

void CameraSource::close()
{
  capture_.release();
}

bool CameraSource::is_opened() const
{
  return capture_.isOpened();
}

std::string CameraSource::input_description() const
{
  return camera_input();
}

double CameraSource::actual_width() const
{
  return capture_.get(cv::CAP_PROP_FRAME_WIDTH);
}

double CameraSource::actual_height() const
{
  return capture_.get(cv::CAP_PROP_FRAME_HEIGHT);
}

double CameraSource::actual_fps() const
{
  return capture_.get(cv::CAP_PROP_FPS);
}

}  // namespace vision_inference_node
