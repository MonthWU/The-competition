#include "vision_inference_node/inference_metrics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace vision_inference_node
{

LatencyWindow::LatencyWindow(const std::size_t maximum_samples)
: maximum_samples_(maximum_samples)
{
  if (maximum_samples_ == 0U) {
    throw std::invalid_argument("LatencyWindow requires at least one sample");
  }
}

void LatencyWindow::record(const double milliseconds)
{
  std::lock_guard<std::mutex> lock(mutex_);
  samples_.push_back(milliseconds);
  if (samples_.size() > maximum_samples_) {
    samples_.pop_front();
  }
}

LatencyPercentiles LatencyWindow::percentiles() const
{
  std::vector<double> samples;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    samples.assign(samples_.begin(), samples_.end());
  }
  if (samples.empty()) {
    return {0.0, 0.0};
  }
  std::sort(samples.begin(), samples.end());
  const auto percentile = [&](const double fraction) {
      const auto index = static_cast<std::size_t>(std::ceil(
          fraction * static_cast<double>(samples.size()))) - 1U;
      return samples[std::min(index, samples.size() - 1U)];
    };
  return {percentile(0.50), percentile(0.95)};
}

}  // namespace vision_inference_node
