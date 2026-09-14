#include "serial_bridge_node/move_ack_tracker.hpp"
#include "serial_bridge_node/task_command_parser.hpp"
#include "serial_bridge_node/vision_frame_formatter.hpp"

#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{
bool expect_equal(const std::string & actual, const std::string & expected)
{
  if (actual == expected) {
    return true;
  }
  std::cerr << "expected '" << expected << "' but got '" << actual << "'\n";
  return false;
}
}  // namespace

int main()
{
  using serial_bridge_node::parse_task_command;
  using serial_bridge_node::format_placement_frame;
  using serial_bridge_node::format_vision_frame;
  bool success = true;
  success &= expect_equal(
    format_vision_frame(true, 12.345F, -6.5F, 7U, 2, "N"), "[12.35,-6.50,007]");
  success &= expect_equal(
    format_vision_frame(false, 0.0F, 0.0F, 999U, 2, "N"), "[N,N,999]");
  success &= expect_equal(
    format_vision_frame(false, 0.0F, 0.0F, 1000U, 2, "N"), "[N,N,000]");
  success &= expect_equal(
    format_placement_frame(12.345, -6.5, 30.0, 40.25, -15.0, 2),
    "[12.35,-6.50,30.00,40.25,-15.00]");
  bool rejected_non_finite = false;
  try {
    (void)format_placement_frame(
      0.0, 0.0, 1.0, 2.0, std::numeric_limits<double>::quiet_NaN(), 2);
  } catch (const std::invalid_argument &) {
    rejected_non_finite = true;
  }
  success &= rejected_non_finite;
  int task_number = 0;
  success &= parse_task_command("[task,1]", task_number) && task_number == 1;
  success &= parse_task_command("[task,2]", task_number) && task_number == 2;
  success &= parse_task_command("[task,3]", task_number) && task_number == 3;
  success &= !parse_task_command("[task,0]", task_number);
  success &= !parse_task_command("[task,4]", task_number);
  success &= !parse_task_command("[task,1", task_number);
  success &= !parse_task_command("[TASK,1]", task_number);

  serial_bridge_node::MoveAckTracker tracker;
  std::uint32_t completed_piece_id = 0U;
  std::uint8_t completed_task_id = 0U;
  std::uint32_t completed_generation = 0U;
  success &= tracker.begin_placement(7U, 1U, 10U);
  success &= !tracker.begin_placement(8U, 2U, 11U);
  success &= !tracker.accept_frame(
    "[move,ok]", completed_piece_id, completed_task_id, completed_generation);
  success &= !tracker.mark_transmitted(7U, 1U, 11U);
  success &= tracker.mark_transmitted(7U, 1U, 10U);
  success &= tracker.accept_frame(
    "[move,ok]", completed_piece_id, completed_task_id, completed_generation) &&
    completed_piece_id == 7U && completed_task_id == 1U && completed_generation == 10U;
  success &= !tracker.accept_frame(
    "[move,ok]", completed_piece_id, completed_task_id, completed_generation);
  success &= !tracker.begin_placement(8U, 2U, 0U);
  success &= tracker.begin_placement(8U, 2U, 11U);
  tracker.cancel();
  success &= !tracker.busy();
  return success ? 0 : 1;
}
