#ifndef SERIAL_BRIDGE_NODE__TASK_COMMAND_PARSER_HPP_
#define SERIAL_BRIDGE_NODE__TASK_COMMAND_PARSER_HPP_

#include <string>

namespace serial_bridge_node
{

inline bool parse_task_command(const std::string & frame, int & task_number)
{
  if (frame.size() != 8U || frame.compare(0U, 6U, "[task,") != 0 || frame[7] != ']') {
    return false;
  }
  if (frame[6] < '1' || frame[6] > '3') {
    return false;
  }
  task_number = frame[6] - '0';
  return true;
}

}  // namespace serial_bridge_node

#endif  // SERIAL_BRIDGE_NODE__TASK_COMMAND_PARSER_HPP_
