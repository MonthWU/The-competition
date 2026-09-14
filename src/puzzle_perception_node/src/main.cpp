#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "puzzle_perception_node/puzzle_perception_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PuzzlePerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
