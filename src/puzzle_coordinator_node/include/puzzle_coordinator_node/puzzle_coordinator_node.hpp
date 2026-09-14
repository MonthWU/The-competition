#ifndef PUZZLE_COORDINATOR_NODE__PUZZLE_COORDINATOR_NODE_HPP_
#define PUZZLE_COORDINATOR_NODE__PUZZLE_COORDINATOR_NODE_HPP_

#include <chrono>
#include <cstdint>
#include <deque>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <vision_interfaces/msg/placement_done.hpp>
#include <vision_interfaces/msg/puzzle_placement.hpp>
#include <vision_interfaces/msg/puzzle_plan.hpp>
#include <vision_interfaces/msg/puzzle_scene.hpp>
#include <vision_interfaces/msg/scan_request.hpp>
#include <vision_interfaces/msg/task_session.hpp>

class PuzzleCoordinatorNode : public rclcpp::Node
{
public:
  PuzzleCoordinatorNode();

private:
  enum class State
  {
    IDLE,
    WAIT_GREEN_A4,
    WAIT_PLAN,
    WAIT_PLACEMENT_DONE,
    TASK_SELECTED,
    COMPLETE,
    FAILED
  };

  void on_start(const std_msgs::msg::Bool::ConstSharedPtr message);
  void on_task_command(const std_msgs::msg::String::ConstSharedPtr message);
  void on_scene(const vision_interfaces::msg::PuzzleScene::ConstSharedPtr message);
  void on_plan(const vision_interfaces::msg::PuzzlePlan::ConstSharedPtr message);
  void on_placement_done(const vision_interfaces::msg::PlacementDone::ConstSharedPtr message);
  void on_watchdog();
  void request_scan();
  void request_green_a4_scan();
  void dispatch_task(int task_number);
  void start_selected_task();
  void dispatch_next_placement(const std::string & reason);
  void publish_task_session(const std::string & reason);
  void publish_map_ok();
  void set_state(State state, const std::string & reason);
  static const char * state_name(State state);

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_command_subscription_;
  rclcpp::Subscription<vision_interfaces::msg::PuzzleScene>::SharedPtr scene_subscription_;
  rclcpp::Subscription<vision_interfaces::msg::PuzzlePlan>::SharedPtr plan_subscription_;
  rclcpp::Subscription<vision_interfaces::msg::PlacementDone>::SharedPtr
    placement_done_subscription_;
  rclcpp::Publisher<vision_interfaces::msg::ScanRequest>::SharedPtr scan_request_publisher_;
  rclcpp::Publisher<vision_interfaces::msg::PuzzlePlacement>::SharedPtr placement_publisher_;
  rclcpp::Publisher<vision_interfaces::msg::TaskSession>::SharedPtr task_session_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr map_result_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  std::string start_topic_;
  std::string task_command_topic_;
  std::string scan_request_topic_;
  std::string scene_topic_;
  std::string plan_topic_;
  std::string placement_request_topic_;
  std::string placement_done_topic_;
  std::string task_session_topic_;
  std::string map_result_topic_;
  std::string status_topic_;
  double plan_timeout_sec_;
  double challenge_plan_timeout_sec_;
  double placement_timeout_sec_;
  double green_a4_retry_sec_;
  double result_linger_sec_;
  int max_scan_retries_;
  bool debug_mode_;

  State state_;
  int scan_retries_;
  std::uint32_t active_piece_id_;
  std::uint32_t task_generation_;
  int active_task_;
  bool green_a4_detected_;
  std::deque<vision_interfaces::msg::PuzzlePlacement> pending_placements_;
  std::chrono::steady_clock::time_point state_started_;
};

#endif  // PUZZLE_COORDINATOR_NODE__PUZZLE_COORDINATOR_NODE_HPP_
