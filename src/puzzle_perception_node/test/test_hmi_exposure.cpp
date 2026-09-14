#include "puzzle_perception_node/hmi_exposure.hpp"

#include <iostream>
#include <string>

int main()
{
  const auto short_exposure = puzzle_perception_node::parse_hmi_flash_command(
    R"({"cmd":"flash","value":0,"persistent_parameter":"hmi_value_flash"})");
  const auto long_exposure = puzzle_perception_node::parse_hmi_flash_command(
    R"({ "value" : 1, "cmd" : "flash" })");
  if (!short_exposure || *short_exposure != 78 || !long_exposure || *long_exposure != 220) {
    std::cerr << "flash command mapping mismatch\n";
    return 1;
  }
  if (puzzle_perception_node::parse_hmi_flash_command(R"({"cmd":"debug","value":1})") ||
    puzzle_perception_node::parse_hmi_flash_command(R"({"cmd":"flash","value":2})"))
  {
    std::cerr << "invalid flash command accepted\n";
    return 1;
  }

  const std::string pipeline =
    "v4l2src extra-controls=c,exposure_time_absolute=78,gain=48 ! appsink";
  const auto updated = puzzle_perception_node::camera_pipeline_with_exposure(pipeline, 220);
  if (!updated || updated->find("exposure_time_absolute=220") == std::string::npos ||
    updated->find("gain=48") == std::string::npos)
  {
    std::cerr << "camera pipeline exposure replacement failed\n";
    return 1;
  }
  if (puzzle_perception_node::camera_pipeline_with_exposure(pipeline, 100) ||
    puzzle_perception_node::camera_pipeline_with_exposure("v4l2src ! appsink", 78))
  {
    std::cerr << "invalid camera pipeline accepted\n";
    return 1;
  }
  return 0;
}
