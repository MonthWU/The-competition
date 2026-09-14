#ifndef SERIAL_BRIDGE_NODE__MOVE_ACK_TRACKER_HPP_
#define SERIAL_BRIDGE_NODE__MOVE_ACK_TRACKER_HPP_

#include <cstdint>
#include <string>

namespace serial_bridge_node
{

class MoveAckTracker
{
public:
  bool begin_placement(
    const std::uint32_t piece_id, const std::uint8_t task_id,
    const std::uint32_t generation)
  {
    if (state_ != State::IDLE || piece_id == 0U || task_id == 0U || generation == 0U) {
      return false;
    }
    piece_id_ = piece_id;
    task_id_ = task_id;
    generation_ = generation;
    state_ = State::QUEUED;
    return true;
  }

  bool mark_transmitted(
    const std::uint32_t piece_id, const std::uint8_t task_id,
    const std::uint32_t generation)
  {
    if (state_ != State::QUEUED || piece_id != piece_id_ || task_id != task_id_ ||
      generation != generation_)
    {
      return false;
    }
    state_ = State::WAITING_ACK;
    return true;
  }

  bool accept_frame(
    const std::string & frame, std::uint32_t & completed_piece_id,
    std::uint8_t & completed_task_id, std::uint32_t & completed_generation)
  {
    if (frame != "[move,ok]" || state_ != State::WAITING_ACK) {
      return false;
    }
    completed_piece_id = piece_id_;
    completed_task_id = task_id_;
    completed_generation = generation_;
    piece_id_ = 0U;
    task_id_ = 0U;
    generation_ = 0U;
    state_ = State::IDLE;
    return true;
  }

  void cancel()
  {
    piece_id_ = 0U;
    task_id_ = 0U;
    generation_ = 0U;
    state_ = State::IDLE;
  }

  bool busy() const {return state_ != State::IDLE;}
  bool queued() const {return state_ == State::QUEUED;}
  bool waiting_ack() const {return state_ == State::WAITING_ACK;}
  std::uint32_t piece_id() const {return piece_id_;}
  std::uint8_t task_id() const {return task_id_;}
  std::uint32_t generation() const {return generation_;}

private:
  enum class State
  {
    IDLE,
    QUEUED,
    WAITING_ACK
  };

  State state_{State::IDLE};
  std::uint32_t piece_id_{0U};
  std::uint8_t task_id_{0U};
  std::uint32_t generation_{0U};
};

}  // namespace serial_bridge_node

#endif  // SERIAL_BRIDGE_NODE__MOVE_ACK_TRACKER_HPP_
