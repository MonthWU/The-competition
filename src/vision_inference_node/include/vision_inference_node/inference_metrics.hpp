#ifndef VISION_INFERENCE_NODE__INFERENCE_METRICS_HPP_
#define VISION_INFERENCE_NODE__INFERENCE_METRICS_HPP_

#include <cstddef>
#include <deque>
#include <mutex>

namespace vision_inference_node
{

struct InferenceMetrics
{
  double upload_preprocess_ms{0.0};
  double tensorrt_ms{0.0};
  double download_decode_ms{0.0};
  double total_ms{0.0};
};

struct LatencyPercentiles
{
  double p50_ms{0.0};
  double p95_ms{0.0};
};

class LatencyWindow
{
public:
  explicit LatencyWindow(std::size_t maximum_samples = 256U);

  void record(double milliseconds);
  LatencyPercentiles percentiles() const;

private:
  std::size_t maximum_samples_;
  mutable std::mutex mutex_;
  std::deque<double> samples_;
};

}  // namespace vision_inference_node

#endif  // VISION_INFERENCE_NODE__INFERENCE_METRICS_HPP_
