#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "puzzle_coordinator_node/puzzle_coordinator_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PuzzleCoordinatorNode>());
  rclcpp::shutdown();
  return 0;
}
