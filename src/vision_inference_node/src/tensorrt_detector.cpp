#include "vision_inference_node/tensorrt_detector.hpp"

#include "vision_inference_node/cuda_preprocess.hpp"
#include "vision_inference_node/yolo_postprocess.hpp"

#include <NvInfer.h>
#include <cuda_fp16.h>
#include <cuda_runtime_api.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace vision_inference_node
{
namespace
{

class TensorRtLogger final : public nvinfer1::ILogger
{
public:
  void log(const Severity severity, const char * message) noexcept override
  {
    if (severity <= Severity::kWARNING) {
      last_message = message == nullptr ? "" : message;
    }
  }

  std::string last_message;
};

template<typename T>
struct TensorRtDeleter
{
  void operator()(T * object) const
  {
    delete object;
  }
};

template<typename T>
using TensorRtPtr = std::unique_ptr<T, TensorRtDeleter<T>>;

void check_cuda(const cudaError_t status, const char * operation)
{
  if (status != cudaSuccess) {
    throw std::runtime_error(
            std::string("BUG_POINT:CUDA_RUNTIME ") + operation + ": " +
            cudaGetErrorString(status));
  }
}

std::vector<std::uint8_t> read_binary_file(const std::string & path)
{
  std::ifstream stream(path, std::ios::binary | std::ios::ate);
  if (!stream) {
    throw std::runtime_error("BUG_POINT:ENGINE_LOAD cannot open engine: " + path);
  }
  const auto size = stream.tellg();
  if (size <= 0) {
    throw std::runtime_error("BUG_POINT:ENGINE_LOAD engine is empty: " + path);
  }
  std::vector<std::uint8_t> data(static_cast<std::size_t>(size));
  stream.seekg(0, std::ios::beg);
  if (!stream.read(
      reinterpret_cast<char *>(data.data()), static_cast<std::streamsize>(size)))
  {
    throw std::runtime_error("BUG_POINT:ENGINE_LOAD failed to read engine: " + path);
  }
  return data;
}

std::size_t data_type_size(const nvinfer1::DataType type)
{
  switch (type) {
    case nvinfer1::DataType::kFLOAT:
      return sizeof(float);
    case nvinfer1::DataType::kHALF:
      return sizeof(__half);
    case nvinfer1::DataType::kINT32:
      return sizeof(std::int32_t);
    case nvinfer1::DataType::kINT8:
      return sizeof(std::int8_t);
    case nvinfer1::DataType::kBOOL:
      return sizeof(bool);
#if NV_TENSORRT_MAJOR >= 10
    case nvinfer1::DataType::kUINT8:
      return sizeof(std::uint8_t);
    case nvinfer1::DataType::kFP8:
    case nvinfer1::DataType::kBF16:
    case nvinfer1::DataType::kINT64:
    case nvinfer1::DataType::kINT4:
      break;
#endif
  }
  throw std::runtime_error("BUG_POINT:ENGINE_BINDING unsupported TensorRT data type");
}

std::string data_type_name(const nvinfer1::DataType type)
{
  switch (type) {
    case nvinfer1::DataType::kFLOAT: return "fp32";
    case nvinfer1::DataType::kHALF: return "fp16";
    case nvinfer1::DataType::kINT32: return "int32";
    case nvinfer1::DataType::kINT8: return "int8";
    case nvinfer1::DataType::kBOOL: return "bool";
#if NV_TENSORRT_MAJOR >= 10
    case nvinfer1::DataType::kUINT8: return "uint8";
    case nvinfer1::DataType::kFP8: return "fp8";
    case nvinfer1::DataType::kBF16: return "bf16";
    case nvinfer1::DataType::kINT64: return "int64";
    case nvinfer1::DataType::kINT4: return "int4";
#endif
  }
  return "unknown";
}

std::size_t tensor_volume(const nvinfer1::Dims & dimensions)
{
  std::size_t volume = 1U;
  for (int index = 0; index < dimensions.nbDims; ++index) {
    if (dimensions.d[index] <= 0) {
      throw std::runtime_error("BUG_POINT:ENGINE_BINDING unresolved or invalid tensor shape");
    }
    volume *= static_cast<std::size_t>(dimensions.d[index]);
  }
  return volume;
}

std::string dimensions_text(const nvinfer1::Dims & dimensions)
{
  std::ostringstream stream;
  stream << "[";
  for (int index = 0; index < dimensions.nbDims; ++index) {
    if (index != 0) {
      stream << ",";
    }
    stream << dimensions.d[index];
  }
  stream << "]";
  return stream.str();
}

}  // namespace

class TensorRtDetector::Impl
{
public:
  Impl(
    const std::string & engine_path,
    const int configured_input_size,
    const float confidence_threshold,
    const float iou_threshold,
    const int max_detections)
  : confidence_threshold_(confidence_threshold),
    iou_threshold_(iou_threshold),
    max_detections_(max_detections)
  {
    const auto serialized_engine = read_binary_file(engine_path);
    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    if (!runtime_) {
      throw std::runtime_error("BUG_POINT:ENGINE_LOAD createInferRuntime failed");
    }
    engine_.reset(runtime_->deserializeCudaEngine(
        serialized_engine.data(), serialized_engine.size()));
    if (!engine_) {
      throw std::runtime_error(
              "BUG_POINT:ENGINE_LOAD deserializeCudaEngine failed: " + logger_.last_message);
    }
    context_.reset(engine_->createExecutionContext());
    if (!context_) {
      throw std::runtime_error("BUG_POINT:ENGINE_LOAD createExecutionContext failed");
    }
    inspect_bindings(configured_input_size);
    allocate_buffers();
  }

  ~Impl()
  {
    if (preprocess_start_ != nullptr) {cudaEventDestroy(preprocess_start_);}
    if (preprocess_end_ != nullptr) {cudaEventDestroy(preprocess_end_);}
    if (inference_end_ != nullptr) {cudaEventDestroy(inference_end_);}
    if (download_end_ != nullptr) {cudaEventDestroy(download_end_);}
    if (stream_ != nullptr) {cudaStreamDestroy(stream_);}
    if (host_output_ != nullptr) {cudaFreeHost(host_output_);}
    if (device_output_ != nullptr) {cudaFree(device_output_);}
    if (device_input_ != nullptr) {cudaFree(device_input_);}
    if (device_yuyv_ != nullptr) {cudaFree(device_yuyv_);}
  }

  std::vector<Detection> infer(const cv::Mat & frame, InferenceMetrics & metrics)
  {
    if (frame.empty() || frame.type() != CV_8UC2 || !frame.isContinuous()) {
      throw std::runtime_error(
              "BUG_POINT:INFERENCE_INPUT expected a continuous CV_8UC2 YUYV frame");
    }
    const auto started = std::chrono::steady_clock::now();
    const std::size_t source_bytes = frame.total() * frame.elemSize();
    ensure_source_capacity(source_bytes);
    check_cuda(cudaEventRecord(preprocess_start_, stream_), "record preprocess start");
    check_cuda(
      cudaMemcpyAsync(
        device_yuyv_, frame.data, source_bytes, cudaMemcpyHostToDevice, stream_),
      "upload YUYV frame");
    check_cuda(
      launch_yuyv_letterbox(
        static_cast<const unsigned char *>(device_yuyv_), frame.cols, frame.rows,
        device_input_, input_width_, input_height_, input_type_, stream_),
      "launch YUYV letterbox kernel");
    check_cuda(cudaEventRecord(preprocess_end_, stream_), "record preprocess end");
    if (!context_->enqueueV3(stream_)) {
      throw std::runtime_error("BUG_POINT:INFERENCE_RUNTIME TensorRT enqueueV3 failed");
    }
    check_cuda(cudaEventRecord(inference_end_, stream_), "record inference end");
    check_cuda(
      cudaMemcpyAsync(
        host_output_, device_output_, output_bytes_, cudaMemcpyDeviceToHost, stream_),
      "download TensorRT output");
    check_cuda(cudaEventRecord(download_end_, stream_), "record download end");
    check_cuda(cudaEventSynchronize(download_end_), "synchronize inference stream");

    float elapsed = 0.0F;
    check_cuda(
      cudaEventElapsedTime(&elapsed, preprocess_start_, preprocess_end_),
      "measure preprocess");
    metrics.upload_preprocess_ms = elapsed;
    check_cuda(
      cudaEventElapsedTime(&elapsed, preprocess_end_, inference_end_),
      "measure TensorRT");
    metrics.tensorrt_ms = elapsed;
    check_cuda(
      cudaEventElapsedTime(&elapsed, inference_end_, download_end_),
      "measure download");
    metrics.download_decode_ms = elapsed;

    std::vector<int> output_dimensions;
    output_dimensions.reserve(static_cast<std::size_t>(output_dimensions_.nbDims));
    for (int index = 0; index < output_dimensions_.nbDims; ++index) {
      output_dimensions.push_back(output_dimensions_.d[index]);
    }
    const TensorView output(
      host_output_, output_elements_,
      output_data_type_ == nvinfer1::DataType::kFLOAT ?
      TensorValueType::kFloat32 : TensorValueType::kFloat16);
    auto detections = decode_yolo_output(
      output, output_dimensions, frame.cols, frame.rows, input_width_, input_height_,
      confidence_threshold_, iou_threshold_, max_detections_);
    metrics.total_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    metrics.download_decode_ms = std::max(
      0.0, metrics.total_ms - metrics.upload_preprocess_ms - metrics.tensorrt_ms);
    return detections;
  }

  int input_width() const {return input_width_;}
  int input_height() const {return input_height_;}
  std::string binding_summary() const {return binding_summary_;}

private:
  void inspect_bindings(const int configured_input_size)
  {
    for (int index = 0; index < engine_->getNbIOTensors(); ++index) {
      const char * name = engine_->getIOTensorName(index);
      if (name == nullptr) {
        continue;
      }
      if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
        if (!input_name_.empty()) {
          throw std::runtime_error("BUG_POINT:ENGINE_BINDING multiple inputs are unsupported");
        }
        input_name_ = name;
      } else {
        if (!output_name_.empty()) {
          throw std::runtime_error("BUG_POINT:ENGINE_BINDING multiple outputs are unsupported");
        }
        output_name_ = name;
      }
    }
    if (input_name_.empty() || output_name_.empty()) {
      throw std::runtime_error("BUG_POINT:ENGINE_BINDING expected exactly one input and one output");
    }

    auto input_dimensions = engine_->getTensorShape(input_name_.c_str());
    if (input_dimensions.nbDims != 4) {
      throw std::runtime_error("BUG_POINT:ENGINE_BINDING expected NCHW rank-4 input");
    }
    if (input_dimensions.d[0] < 0) {input_dimensions.d[0] = 1;}
    if (input_dimensions.d[1] < 0) {input_dimensions.d[1] = 3;}
    if (input_dimensions.d[2] < 0) {input_dimensions.d[2] = configured_input_size;}
    if (input_dimensions.d[3] < 0) {input_dimensions.d[3] = configured_input_size;}
    if (input_dimensions.d[0] != 1 || input_dimensions.d[1] != 3) {
      throw std::runtime_error("BUG_POINT:ENGINE_BINDING only batch-1 RGB NCHW input is supported");
    }
    if (!context_->setInputShape(input_name_.c_str(), input_dimensions)) {
      throw std::runtime_error("BUG_POINT:ENGINE_BINDING setInputShape failed");
    }
    input_width_ = input_dimensions.d[3];
    input_height_ = input_dimensions.d[2];
    if (input_width_ != configured_input_size || input_height_ != configured_input_size) {
      throw std::runtime_error(
              "BUG_POINT:ENGINE_BINDING engine input shape does not match input_size parameter");
    }
    input_data_type_ = engine_->getTensorDataType(input_name_.c_str());
    if (input_data_type_ == nvinfer1::DataType::kFLOAT) {
      input_type_ = NetworkInputType::kFloat32;
    } else if (input_data_type_ == nvinfer1::DataType::kHALF) {
      input_type_ = NetworkInputType::kFloat16;
    } else {
      throw std::runtime_error("BUG_POINT:ENGINE_BINDING input must be fp32 or fp16");
    }
    output_data_type_ = engine_->getTensorDataType(output_name_.c_str());
    if (output_data_type_ != nvinfer1::DataType::kFLOAT &&
      output_data_type_ != nvinfer1::DataType::kHALF)
    {
      throw std::runtime_error("BUG_POINT:ENGINE_BINDING output must be fp32 or fp16");
    }
    output_dimensions_ = context_->getTensorShape(output_name_.c_str());
    output_elements_ = tensor_volume(output_dimensions_);
    output_bytes_ = output_elements_ * data_type_size(output_data_type_);

    std::ostringstream summary;
    summary << "input=" << input_name_ << dimensions_text(input_dimensions) << "/" <<
      data_type_name(input_data_type_) << " output=" << output_name_ <<
      dimensions_text(output_dimensions_) << "/" << data_type_name(output_data_type_);
    binding_summary_ = summary.str();
  }

  void allocate_buffers()
  {
    check_cuda(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking), "create CUDA stream");
    check_cuda(cudaEventCreate(&preprocess_start_), "create preprocess start event");
    check_cuda(cudaEventCreate(&preprocess_end_), "create preprocess end event");
    check_cuda(cudaEventCreate(&inference_end_), "create inference end event");
    check_cuda(cudaEventCreate(&download_end_), "create download end event");
    const std::size_t input_elements =
      static_cast<std::size_t>(3 * input_width_ * input_height_);
    check_cuda(
      cudaMalloc(&device_input_, input_elements * data_type_size(input_data_type_)),
      "allocate TensorRT input");
    check_cuda(cudaMalloc(&device_output_, output_bytes_), "allocate TensorRT output");
    check_cuda(cudaMallocHost(&host_output_, output_bytes_), "allocate pinned output");
    if (!context_->setTensorAddress(input_name_.c_str(), device_input_) ||
      !context_->setTensorAddress(output_name_.c_str(), device_output_))
    {
      throw std::runtime_error("BUG_POINT:ENGINE_BINDING setTensorAddress failed");
    }
  }

  void ensure_source_capacity(const std::size_t bytes)
  {
    if (bytes <= source_capacity_) {
      return;
    }
    if (device_yuyv_ != nullptr) {
      check_cuda(cudaFree(device_yuyv_), "free previous YUYV buffer");
      device_yuyv_ = nullptr;
    }
    check_cuda(cudaMalloc(&device_yuyv_, bytes), "allocate YUYV buffer");
    source_capacity_ = bytes;
  }

  TensorRtLogger logger_;
  TensorRtPtr<nvinfer1::IRuntime> runtime_;
  TensorRtPtr<nvinfer1::ICudaEngine> engine_;
  TensorRtPtr<nvinfer1::IExecutionContext> context_;
  std::string input_name_;
  std::string output_name_;
  std::string binding_summary_;
  nvinfer1::DataType input_data_type_{nvinfer1::DataType::kFLOAT};
  nvinfer1::DataType output_data_type_{nvinfer1::DataType::kFLOAT};
  NetworkInputType input_type_{NetworkInputType::kFloat32};
  nvinfer1::Dims output_dimensions_{};
  int input_width_{0};
  int input_height_{0};
  std::size_t output_elements_{0U};
  std::size_t output_bytes_{0U};
  std::size_t source_capacity_{0U};
  float confidence_threshold_{0.25F};
  float iou_threshold_{0.45F};
  int max_detections_{10};
  void * device_yuyv_{nullptr};
  void * device_input_{nullptr};
  void * device_output_{nullptr};
  void * host_output_{nullptr};
  cudaStream_t stream_{nullptr};
  cudaEvent_t preprocess_start_{nullptr};
  cudaEvent_t preprocess_end_{nullptr};
  cudaEvent_t inference_end_{nullptr};
  cudaEvent_t download_end_{nullptr};
};

TensorRtDetector::TensorRtDetector(
  const std::string & engine_path,
  const int input_size,
  const float confidence_threshold,
  const float iou_threshold,
  const int max_detections)
: impl_(std::make_unique<Impl>(
      engine_path, input_size, confidence_threshold, iou_threshold, max_detections))
{
}

TensorRtDetector::~TensorRtDetector() = default;

std::vector<Detection> TensorRtDetector::infer(
  const cv::Mat & yuyv_frame, InferenceMetrics & metrics)
{
  return impl_->infer(yuyv_frame, metrics);
}

int TensorRtDetector::input_width() const {return impl_->input_width();}
int TensorRtDetector::input_height() const {return impl_->input_height();}
std::string TensorRtDetector::binding_summary() const {return impl_->binding_summary();}

}  // namespace vision_inference_node
