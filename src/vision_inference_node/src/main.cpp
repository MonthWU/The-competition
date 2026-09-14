#include "vision_inference_node/vision_inference_node.hpp"

#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<VisionInferenceNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("vision_inference_node"), "%s", error.what());
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
