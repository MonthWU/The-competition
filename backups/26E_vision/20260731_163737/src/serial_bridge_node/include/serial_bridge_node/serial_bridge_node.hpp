#ifndef SERIAL_BRIDGE_NODE__SERIAL_BRIDGE_NODE_HPP_
#define SERIAL_BRIDGE_NODE__SERIAL_BRIDGE_NODE_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <vision_interfaces/msg/placement_done.hpp>
#include <vision_interfaces/msg/puzzle_placement.hpp>
#include <vision_interfaces/msg/task_session.hpp>
#include <vision_interfaces/msg/target_point.hpp>

#include "serial_bridge_node/move_ack_tracker.hpp"

class SerialBridgeNode : public rclcpp::Node
{
public:
  SerialBridgeNode();
  ~SerialBridgeNode() override;

private:
  struct PendingFrame
  {
    std::vector<std::uint8_t> bytes;
    std::size_t written_bytes{0U};
    std::string source;
    std::uint32_t piece_id{0U};
    std::uint8_t task_id{0U};
    std::uint32_t generation{0U};
  };

  void bridge_tick();
  void vision_tick();
  void on_target(const vision_interfaces::msg::TargetPoint::ConstSharedPtr message);
  void on_placement(const vision_interfaces::msg::PuzzlePlacement::ConstSharedPtr message);
  void on_task_session(const vision_interfaces::msg::TaskSession::ConstSharedPtr message);
  void process_placement(const vision_interfaces::msg::PuzzlePlacement & message);
  void maybe_send_deferred_placement();
  void discard_unsent_stale_placements();
  void on_hmi_tx(const std_msgs::msg::UInt8MultiArray::ConstSharedPtr message);
  bool open_ports();
  bool try_open_hmi();
  bool try_open_gyro();
  void close_ports();
  void extract_gyro_frames();
  void consume_mcu_bytes(const char * data, std::size_t size);
  void consume_hmi_bytes(const char * data, std::size_t size);
  void publish_received_frame(const std::string & frame);
  void publish_hmi_frame(const std::string & frame);
  void publish_status();
  void record_sent_vision_frame(const std::string & frame);
  void record_mcu_raw_bytes(const char * data, std::size_t size);
  void record_received_mcu_frame(const std::string & frame);
  bool enqueue_frame(
    std::vector<std::uint8_t> bytes, const std::string & source,
    std::uint32_t piece_id = 0U, std::uint8_t task_id = 0U,
    std::uint32_t generation = 0U);
  bool enqueue_hmi_frame(std::vector<std::uint8_t> bytes);
  bool flush_transmit_queue();
  bool flush_hmi_transmit_queue();

  rclcpp::TimerBase::SharedPtr bridge_timer_;
  rclcpp::TimerBase::SharedPtr vision_timer_;
  rclcpp::Subscription<vision_interfaces::msg::TargetPoint>::SharedPtr target_subscription_;
  rclcpp::Subscription<vision_interfaces::msg::PuzzlePlacement>::SharedPtr placement_subscription_;
  rclcpp::Subscription<vision_interfaces::msg::TaskSession>::SharedPtr task_session_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr hmi_tx_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr raw_frame_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_command_publisher_;
  rclcpp::Publisher<vision_interfaces::msg::PlacementDone>::SharedPtr placement_done_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hmi_rx_publisher_;

  std::string gyro_input_port_;
  std::string mcu_port_;
  std::string hmi_port_;
  std::string target_topic_;
  std::string placement_topic_;
  std::string placement_coordinate_frame_;
  std::string placement_angle_mode_;
  std::string status_topic_;
  std::string raw_frame_topic_;
  std::string task_command_topic_;
  std::string task_session_topic_;
  std::string placement_done_topic_;
  std::string hmi_rx_topic_;
  std::string hmi_tx_topic_;
  int baud_rate_;
  int hmi_baud_rate_;
  int bridge_rate_hz_;
  int vision_rate_hz_;
  int target_timeout_ms_;
  int vision_decimals_;
  int placement_decimals_;
  int max_read_bytes_;
  int max_queue_bytes_;
  int max_receive_frame_length_;
  int gyro_frame_length_;
  double coordinate_center_x_;
  double coordinate_center_y_;
  double width_normalizer_px_;
  double minimum_confidence_;
  double frequency_tolerance_hz_;
  std::string no_target_token_;
  std::vector<std::uint8_t> gyro_frame_header_;
  bool serial_enabled_;
  bool vision_tx_enabled_;
  bool placement_tx_enabled_;
  bool gyro_forward_enabled_;
  bool hmi_enabled_;
  bool hmi_over_mcu_;
  bool apply_width_weight_;
  bool debug_mode_;

  bool target_received_;
  bool latest_target_valid_;
  float latest_dx_;
  float latest_dy_;
  int gyro_input_fd_;
  int mcu_fd_;
  int hmi_fd_;
  std::vector<std::uint8_t> gyro_receive_buffer_;
  std::string mcu_receive_buffer_;
  std::string hmi_receive_buffer_;
  std::deque<PendingFrame> transmit_queue_;
  std::deque<PendingFrame> hmi_transmit_queue_;
  std::size_t queued_bytes_;
  std::size_t hmi_queued_bytes_;
  std::chrono::steady_clock::time_point next_open_attempt_;
  std::chrono::steady_clock::time_point status_window_start_;
  std::chrono::steady_clock::time_point last_target_update_;
  std::size_t bridge_tick_count_;
  std::size_t vision_transmitted_count_;
  std::size_t gyro_transmitted_count_;
  std::size_t hmi_transmitted_count_;
  std::size_t hmi_received_count_;
  std::size_t mcu_received_bytes_count_{0U};
  std::size_t mcu_received_frame_count_{0U};
  std::size_t mcu_discarded_bytes_count_{0U};
  std::size_t queue_drop_count_;
  std::size_t hmi_queue_drop_count_;
  std::size_t placement_ack_count_{0U};
  std::uint8_t active_task_id_{0U};
  std::uint32_t active_generation_{0U};
  std::string last_vision_frame_;
  std::deque<std::string> recent_sent_vision_frames_;
  std::deque<std::string> recent_mcu_raw_chunks_;
  std::deque<std::string> recent_received_mcu_frames_;
  serial_bridge_node::MoveAckTracker move_ack_tracker_;
  std::optional<vision_interfaces::msg::PuzzlePlacement> deferred_placement_;
};

#endif  // SERIAL_BRIDGE_NODE__SERIAL_BRIDGE_NODE_HPP_
