#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "puzzle_solver_node/puzzle_solver_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PuzzleSolverNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2U);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
