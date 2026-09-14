#ifndef PUZZLE_PERCEPTION_NODE__HMI_EXPOSURE_HPP_
#define PUZZLE_PERCEPTION_NODE__HMI_EXPOSURE_HPP_

#include <optional>
#include <regex>
#include <string>

namespace puzzle_perception_node
{

constexpr int kShortExposureTime = 78;
constexpr int kLongExposureTime = 220;

inline std::optional<int> parse_hmi_flash_command(const std::string & command)
{
  static const std::regex flash_command_pattern(R"("cmd"\s*:\s*"flash")");
  static const std::regex flash_value_pattern(R"("value"\s*:\s*([01])\s*[,}])");
  if (!std::regex_search(command, flash_command_pattern)) {
    return std::nullopt;
  }
  std::smatch value_match;
  if (!std::regex_search(command, value_match, flash_value_pattern)) {
    return std::nullopt;
  }
  return value_match[1].str() == "0" ? kShortExposureTime : kLongExposureTime;
}

inline std::optional<std::string> camera_pipeline_with_exposure(
  const std::string & pipeline, const int exposure_time)
{
  if (exposure_time != kShortExposureTime && exposure_time != kLongExposureTime) {
    return std::nullopt;
  }
  constexpr const char * marker = "exposure_time_absolute=";
  const auto marker_position = pipeline.find(marker);
  if (marker_position == std::string::npos ||
    pipeline.find(marker, marker_position + 1U) != std::string::npos)
  {
    return std::nullopt;
  }
  const auto value_begin = marker_position + std::string(marker).size();
  auto value_end = value_begin;
  while (value_end < pipeline.size() && pipeline[value_end] >= '0' && pipeline[value_end] <= '9') {
    ++value_end;
  }
  if (value_end == value_begin) {
    return std::nullopt;
  }
  return pipeline.substr(0, value_begin) + std::to_string(exposure_time) +
         pipeline.substr(value_end);
}

}  // namespace puzzle_perception_node

#endif  // PUZZLE_PERCEPTION_NODE__HMI_EXPOSURE_HPP_
