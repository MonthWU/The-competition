#include "puzzle_coordinator_node/puzzle_coordinator_node.hpp"

#include <functional>
#include <limits>
#include <sstream>
#include <utility>

PuzzleCoordinatorNode::PuzzleCoordinatorNode()
: Node("puzzle_coordinator_node"),
  state_(State::IDLE),
  scan_retries_(0),
  active_piece_id_(0),
  task_generation_(0),
  active_task_(0),
  green_a4_detected_(false),
  state_started_(std::chrono::steady_clock::now())
{
  start_topic_ = declare_parameter<std::string>("start_topic", "puzzle/start");
  task_command_topic_ = declare_parameter<std::string>(
    "task_command_topic", "puzzle/task_command");
  scan_request_topic_ = declare_parameter<std::string>("scan_request_topic", "puzzle/scan_request");
  scene_topic_ = declare_parameter<std::string>("scene_topic", "puzzle/scene");
  plan_topic_ = declare_parameter<std::string>("plan_topic", "puzzle/plan");
  placement_request_topic_ = declare_parameter<std::string>(
    "placement_request_topic", "puzzle/placement_request");
  placement_done_topic_ = declare_parameter<std::string>(
    "placement_done_topic", "puzzle/placement_done");
  task_session_topic_ = declare_parameter<std::string>(
    "task_session_topic", "puzzle/task_session");
  map_result_topic_ = declare_parameter<std::string>("map_result_topic", "puzzle/map_result");
  status_topic_ = declare_parameter<std::string>("status_topic", "puzzle/coordinator_status");
  plan_timeout_sec_ = declare_parameter<double>("plan_timeout_sec", 5.0);
  challenge_plan_timeout_sec_ = declare_parameter<double>("challenge_plan_timeout_sec", 30.0);
  placement_timeout_sec_ = declare_parameter<double>("placement_timeout_sec", 30.0);
  green_a4_retry_sec_ = declare_parameter<double>("green_a4_retry_sec", 0.5);
  result_linger_sec_ = declare_parameter<double>("result_linger_sec", 1.0);
  max_scan_retries_ = declare_parameter<int>("max_scan_retries", 2);
  debug_mode_ = declare_parameter<bool>("debug_mode", false);

  scan_request_publisher_ = create_publisher<vision_interfaces::msg::ScanRequest>(
    scan_request_topic_, 10);
  placement_publisher_ = create_publisher<vision_interfaces::msg::PuzzlePlacement>(
    placement_request_topic_, 10);
  task_session_publisher_ = create_publisher<vision_interfaces::msg::TaskSession>(
    task_session_topic_, 10);
  map_result_publisher_ = create_publisher<std_msgs::msg::String>(map_result_topic_, 10);
  status_publisher_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);
  start_subscription_ = create_subscription<std_msgs::msg::Bool>(
    start_topic_, 10, std::bind(&PuzzleCoordinatorNode::on_start, this, std::placeholders::_1));
  task_command_subscription_ = create_subscription<std_msgs::msg::String>(
    task_command_topic_, 10,
    std::bind(&PuzzleCoordinatorNode::on_task_command, this, std::placeholders::_1));
  scene_subscription_ = create_subscription<vision_interfaces::msg::PuzzleScene>(
    scene_topic_, 1, std::bind(&PuzzleCoordinatorNode::on_scene, this, std::placeholders::_1));
  plan_subscription_ = create_subscription<vision_interfaces::msg::PuzzlePlan>(
    plan_topic_, 1, std::bind(&PuzzleCoordinatorNode::on_plan, this, std::placeholders::_1));
  placement_done_subscription_ = create_subscription<vision_interfaces::msg::PlacementDone>(
    placement_done_topic_, 10,
    std::bind(&PuzzleCoordinatorNode::on_placement_done, this, std::placeholders::_1));
  watchdog_timer_ = create_wall_timer(
    std::chrono::milliseconds(200), std::bind(&PuzzleCoordinatorNode::on_watchdog, this));
  set_state(State::IDLE, "READY");
}

void PuzzleCoordinatorNode::on_task_command(const std_msgs::msg::String::ConstSharedPtr message)
{
  const std::string & frame = message->data;
  if (frame.size() != 8U || frame.compare(0U, 6U, "[task,") != 0 || frame[7] != ']' ||
    frame[6] < '1' || frame[6] > '3')
  {
    // BUG_POINT:TASK_COMMAND_FORMAT - Only the exact protocol [task,1|2|3] is accepted.
    RCLCPP_WARN(get_logger(), "BUG_POINT:TASK_COMMAND_FORMAT rejected frame=%s", frame.c_str());
    return;
  }
  dispatch_task(frame[6] - '0');
}

void PuzzleCoordinatorNode::dispatch_task(const int task_number)
{
  const auto previous_state = state_;
  const int previous_task = active_task_;
  const std::uint32_t previous_generation = task_generation_;
  if (task_generation_ == std::numeric_limits<std::uint32_t>::max()) {
    task_generation_ = 1U;
  } else {
    ++task_generation_;
  }
  active_task_ = task_number;
  active_piece_id_ = 0;
  scan_retries_ = 0;
  green_a4_detected_ = false;
  pending_placements_.clear();
  publish_task_session(
    "SWITCH_FROM_TASK_" + std::to_string(previous_task) + "_GEN_" +
    std::to_string(previous_generation));
  RCLCPP_INFO(
    get_logger(), "Task switched: old_task=%d old_generation=%u old_state=%s "
    "new_task=%d new_generation=%u",
    previous_task, previous_generation, state_name(previous_state), active_task_, task_generation_);
  request_green_a4_scan();
}

void PuzzleCoordinatorNode::start_selected_task()
{
  if (active_task_ < 1 || active_task_ > 3) {
    set_state(
      State::FAILED,
      "TASK_" + std::to_string(active_task_) + "_NOT_IMPLEMENTED");
    return;
  }
  set_state(
    State::TASK_SELECTED,
    "TASK_" + std::to_string(active_task_) + "_GREEN_A4_READY");
  request_scan();
}

void PuzzleCoordinatorNode::on_start(const std_msgs::msg::Bool::ConstSharedPtr message)
{
  if (!message->data) {
    return;
  }
  dispatch_task(1);
}

void PuzzleCoordinatorNode::on_scene(
  const vision_interfaces::msg::PuzzleScene::ConstSharedPtr message)
{
  if (state_ != State::WAIT_GREEN_A4 || active_task_ == 0) {
    return;
  }
  if (message->task_id != static_cast<std::uint8_t>(active_task_) ||
    message->generation != task_generation_)
  {
    // BUG_POINT:TASK_SESSION_SCENE - A delayed scan from an interrupted task is inert.
    RCLCPP_WARN(
      get_logger(), "BUG_POINT:TASK_SESSION_SCENE ignored task=%u generation=%u; "
      "active_task=%d active_generation=%u",
      message->task_id, message->generation, active_task_, task_generation_);
    return;
  }
  if (message->a4_detected) {
    green_a4_detected_ = true;
    if (active_task_ == 2 || active_task_ == 3) {
      // BUG_POINT:CHALLENGE_DUPLICATE_SCAN - The green-A4 gate already contains the
      // complete supplied-piece scene. Requesting a second frame makes the solver
      // publish two results for one generation, so reuse this frame and enter
      // WAIT_PLAN before its bounded search completes.
      set_state(
        State::WAIT_PLAN,
        "TASK_" + std::to_string(active_task_) + "_GREEN_A4_READY_USING_CURRENT_SCAN");
      return;
    }
    start_selected_task();
    return;
  }
  set_state(
    State::WAIT_GREEN_A4,
    "TASK_" + std::to_string(active_task_) + "_GREEN_A4_NOT_FOUND_RETRY");
}

void PuzzleCoordinatorNode::on_plan(
  const vision_interfaces::msg::PuzzlePlan::ConstSharedPtr message)
{
  if (state_ != State::WAIT_PLAN) {
    const bool challenge_gate_plan =
      (active_task_ == 2 || active_task_ == 3) && state_ == State::WAIT_GREEN_A4 &&
      message->task_id == static_cast<std::uint8_t>(active_task_) &&
      message->generation == task_generation_;
    if (!challenge_gate_plan) {
      return;
    }
    // BUG_POINT:CHALLENGE_PLAN_BEFORE_WAIT_PLAN - The solver and coordinator both
    // consume the green-A4 scan. If the solved plan arrives before the state
    // callback flips to WAIT_PLAN, keep the same-generation plan instead of
    // waiting for a timeout.
    set_state(
      State::WAIT_PLAN,
      "TASK_" + std::to_string(active_task_) + "_PLAN_ARRIVED_DURING_GREEN_A4_GATE");
  }
  if (active_task_ < 1 || active_task_ > 3) {
    set_state(State::FAILED, "UNSUPPORTED_TASK_PLAN");
    return;
  }
  if (message->task_id != static_cast<std::uint8_t>(active_task_) ||
    message->generation != task_generation_)
  {
    // BUG_POINT:TASK_PLAN_RACE - Never execute a plan solved under another task session.
    RCLCPP_WARN(
      get_logger(), "BUG_POINT:TASK_PLAN_RACE ignored plan_task=%u generation=%u; "
      "active_task=%d active_generation=%u",
      message->task_id, message->generation, active_task_, task_generation_);
    return;
  }
  if (!message->solved) {
    if (scan_retries_ < max_scan_retries_) {
      ++scan_retries_;
      request_scan();
    } else {
      set_state(State::FAILED, "PLAN_FAILED:" + message->status);
    }
    return;
  }

  pending_placements_.clear();
  for (const auto & placement : message->placements) {
    if (placement.already_placed) {
      continue;
    }
    auto session_placement = placement;
    session_placement.task_id = static_cast<std::uint8_t>(active_task_);
    session_placement.generation = task_generation_;
    pending_placements_.push_back(std::move(session_placement));
  }
  if (pending_placements_.empty()) {
    active_piece_id_ = 0;
    publish_map_ok();
    set_state(State::COMPLETE, "ALL_PIECES_ALREADY_PLACED");
    return;
  }

  RCLCPP_INFO(
    get_logger(), "Cached placement queue: task=%d generation=%u count=%zu",
    active_task_, task_generation_, pending_placements_.size());
  dispatch_next_placement("PLACEMENT_QUEUE_STARTED");
}

void PuzzleCoordinatorNode::on_placement_done(
  const vision_interfaces::msg::PlacementDone::ConstSharedPtr message)
{
  if (message->task_id != static_cast<std::uint8_t>(active_task_) ||
    message->generation != task_generation_)
  {
    // BUG_POINT:TASK_SESSION_ACK - A late acknowledgement belongs to its original task.
    RCLCPP_WARN(
      get_logger(), "BUG_POINT:TASK_SESSION_ACK ignored task=%u generation=%u piece=%u; "
      "active_task=%d active_generation=%u",
      message->task_id, message->generation, message->piece_id,
      active_task_, task_generation_);
    return;
  }
  if (state_ != State::WAIT_PLACEMENT_DONE || message->piece_id != active_piece_id_) {
    return;
  }
  scan_retries_ = 0;
  if (pending_placements_.empty()) {
    active_piece_id_ = 0;
    publish_map_ok();
    set_state(State::COMPLETE, "ALL_PLACEMENTS_ACKED");
    return;
  }
  dispatch_next_placement("NEXT_PLACEMENT_AFTER_ACK");
}

void PuzzleCoordinatorNode::on_watchdog()
{
  const double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - state_started_).count();
  const double active_plan_timeout = (active_task_ == 2 || active_task_ == 3) ?
    challenge_plan_timeout_sec_ : plan_timeout_sec_;
  if (state_ == State::WAIT_PLAN && elapsed > active_plan_timeout) {
    set_state(State::FAILED, "PLAN_TIMEOUT");
  } else if (state_ == State::WAIT_GREEN_A4 && elapsed > green_a4_retry_sec_) {
    request_green_a4_scan();
  } else if (state_ == State::WAIT_PLACEMENT_DONE && elapsed > placement_timeout_sec_) {
    set_state(State::FAILED, "PLACEMENT_TIMEOUT");
  } else if ((state_ == State::COMPLETE || state_ == State::FAILED) &&
    elapsed > result_linger_sec_)
  {
    active_task_ = 0;
    active_piece_id_ = 0;
    green_a4_detected_ = false;
    pending_placements_.clear();
    publish_task_session("NO_ACTIVE_TASK");
    set_state(State::IDLE, "NO_TASK");
  }
}

void PuzzleCoordinatorNode::request_green_a4_scan()
{
  vision_interfaces::msg::ScanRequest request;
  request.task_id = static_cast<std::uint8_t>(active_task_);
  request.generation = task_generation_;
  scan_request_publisher_->publish(request);
  set_state(
    State::WAIT_GREEN_A4,
    "TASK_" + std::to_string(active_task_) + "_SEARCHING_GREEN_A4");
}

void PuzzleCoordinatorNode::request_scan()
{
  vision_interfaces::msg::ScanRequest request;
  request.task_id = static_cast<std::uint8_t>(active_task_);
  request.generation = task_generation_;
  scan_request_publisher_->publish(request);
  set_state(State::WAIT_PLAN, "SCAN_REQUESTED");
}

void PuzzleCoordinatorNode::dispatch_next_placement(const std::string & reason)
{
  if (pending_placements_.empty()) {
    active_piece_id_ = 0;
    publish_map_ok();
    set_state(State::COMPLETE, "ALL_PLACEMENTS_ACKED");
    return;
  }
  // BUG_POINT:PLACEMENT_BATCH_QUEUE - Field hardware can still occlude the A4
  // frame when [move,ok] arrives, so do not rescan between queued placements.
  auto placement = pending_placements_.front();
  pending_placements_.pop_front();
  active_piece_id_ = placement.piece_id;
  placement_publisher_->publish(placement);
  set_state(State::WAIT_PLACEMENT_DONE, reason);
}

void PuzzleCoordinatorNode::set_state(const State state, const std::string & reason)
{
  state_ = state;
  state_started_ = std::chrono::steady_clock::now();
  std_msgs::msg::String message;
  std::ostringstream stream;
  stream << "{\"state\":\"" << state_name(state_) << "\",\"reason\":\"" << reason <<
    "\",\"active_piece_id\":" << active_piece_id_ <<
    ",\"active_task\":" << active_task_ <<
    ",\"generation\":" << task_generation_ <<
    ",\"queued_placements\":" << pending_placements_.size() <<
    ",\"green_a4_detected\":" << (green_a4_detected_ ? "true" : "false") << "}";
  message.data = stream.str();
  status_publisher_->publish(message);
}

void PuzzleCoordinatorNode::publish_task_session(const std::string & reason)
{
  vision_interfaces::msg::TaskSession message;
  message.task_id = static_cast<std::uint8_t>(active_task_);
  message.generation = task_generation_;
  message.reason = reason;
  task_session_publisher_->publish(message);
}

void PuzzleCoordinatorNode::publish_map_ok()
{
  std_msgs::msg::String message;
  message.data = "[map,ok]";
  map_result_publisher_->publish(message);
}

const char * PuzzleCoordinatorNode::state_name(const State state)
{
  switch (state) {
    case State::IDLE: return "IDLE";
    case State::WAIT_GREEN_A4: return "WAIT_GREEN_A4";
    case State::WAIT_PLAN: return "WAIT_PLAN";
    case State::WAIT_PLACEMENT_DONE: return "WAIT_PLACEMENT_DONE";
    case State::TASK_SELECTED: return "TASK_SELECTED";
    case State::COMPLETE: return "COMPLETE";
    case State::FAILED: return "FAILED";
  }
  return "UNKNOWN";
}
