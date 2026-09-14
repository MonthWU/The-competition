#ifndef SERIAL_BRIDGE_NODE__VISION_FRAME_FORMATTER_HPP_
#define SERIAL_BRIDGE_NODE__VISION_FRAME_FORMATTER_HPP_

#include <cmath>
#include <cstdint>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>

namespace serial_bridge_node
{

inline bool is_valid_no_target_token(const std::string & token)
{
  if (token.empty()) {
    return false;
  }
  for (const unsigned char value : token) {
    if (value < 0x21U || value > 0x7EU || value == '[' || value == ']' || value == ',') {
      return false;
    }
  }
  return true;
}

inline std::string format_vision_frame(
  const bool detected,
  const float dx,
  const float dy,
  const std::uint64_t timestamp_ms,
  const int decimals,
  const std::string & no_target_token)
{
  if (detected && (!std::isfinite(dx) || !std::isfinite(dy))) {
    throw std::invalid_argument("detected visual offsets must be finite");
  }
  if (!is_valid_no_target_token(no_target_token)) {
    throw std::invalid_argument("invalid ASCII no-target token");
  }
  std::ostringstream payload;
  payload.imbue(std::locale::classic());
  payload << '[';
  if (detected) {
    payload << std::fixed << std::setprecision(decimals) << dx << ',' << dy;
  } else {
    payload << no_target_token << ',' << no_target_token;
  }
  payload << ',' << std::setfill('0') << std::setw(3) << (timestamp_ms % 1000U) << ']';
  return payload.str();
}

inline std::string format_placement_frame(
  const double source_x, const double source_y,
  const double target_x, const double target_y,
  const double angle_deg, const int decimals)
{
  if (!std::isfinite(source_x) || !std::isfinite(source_y) ||
    !std::isfinite(target_x) || !std::isfinite(target_y) || !std::isfinite(angle_deg))
  {
    throw std::invalid_argument("placement fields must be finite");
  }
  if (decimals < 0 || decimals > 6) {
    throw std::invalid_argument("placement decimals must be in [0, 6]");
  }
  std::ostringstream payload;
  payload.imbue(std::locale::classic());
  payload << '[' << std::fixed << std::setprecision(decimals) <<
    source_x << ',' << source_y << ',' << target_x << ',' << target_y << ',' << angle_deg << ']';
  return payload.str();
}

}  // namespace serial_bridge_node

#endif  // SERIAL_BRIDGE_NODE__VISION_FRAME_FORMATTER_HPP_
