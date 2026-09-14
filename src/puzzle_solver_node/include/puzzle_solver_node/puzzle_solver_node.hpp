#ifndef PUZZLE_SOLVER_NODE__PUZZLE_SOLVER_NODE_HPP_
#define PUZZLE_SOLVER_NODE__PUZZLE_SOLVER_NODE_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <vision_interfaces/msg/puzzle_plan.hpp>
#include <vision_interfaces/msg/puzzle_scene.hpp>
#include <vision_interfaces/msg/task_session.hpp>

#include "puzzle_solver_node/puzzle_solver_core.hpp"

class PuzzleSolverNode : public rclcpp::Node
{
public:
  PuzzleSolverNode();

private:
  void on_task_session(const vision_interfaces::msg::TaskSession::ConstSharedPtr message);
  void on_scene(const vision_interfaces::msg::PuzzleScene::ConstSharedPtr message);
  void publish_status(
    const std::string & status, double solve_ms, std::size_t piece_count,
    int task_id, std::uint32_t generation);
  void publish_debug_plan(
    const vision_interfaces::msg::PuzzleScene & scene,
    const vision_interfaces::msg::PuzzlePlan & plan,
    const cv::Mat & rectified) const;

  rclcpp::Subscription<vision_interfaces::msg::PuzzleScene>::SharedPtr scene_subscription_;
  rclcpp::Subscription<vision_interfaces::msg::TaskSession>::SharedPtr task_session_subscription_;
  rclcpp::CallbackGroup::SharedPtr scene_callback_group_;
  rclcpp::CallbackGroup::SharedPtr task_session_callback_group_;
  rclcpp::Publisher<vision_interfaces::msg::PuzzlePlan>::SharedPtr plan_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_publisher_;
  std::unique_ptr<puzzle_solver::PuzzleSolverCore> basic_solver_;
  std::unique_ptr<puzzle_solver::PuzzleSolverCore> challenge_one_solver_;
  std::unique_ptr<puzzle_solver::PuzzleSolverCore> challenge_two_solver_;
  std::string scene_topic_;
  std::string plan_topic_;
  std::string status_topic_;
  std::string task_session_topic_;
  std::string debug_image_topic_;
  double pixels_per_mm_;
  double placed_position_tolerance_mm_;
  double placed_angle_tolerance_deg_;
  double target_piece_gap_mm_;
  double minimum_score_margin_;
  double challenge_minimum_score_margin_;
  double challenge_two_minimum_score_margin_;
  bool require_workspace_mapping_;
  bool camera_y_axis_up_;
  bool debug_mode_;
  std::atomic<int> active_task_{0};
  std::atomic<std::uint32_t> active_generation_{0U};
};

#endif  // PUZZLE_SOLVER_NODE__PUZZLE_SOLVER_NODE_HPP_
