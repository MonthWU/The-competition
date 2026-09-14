#include "serial_bridge_node/serial_bridge_node.hpp"
#include "serial_bridge_node/task_command_parser.hpp"
#include "serial_bridge_node/vision_frame_formatter.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <geometry_msgs/msg/point32.hpp>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>

namespace
{
bool configure_baud_rate(const int baud_rate, speed_t & speed)
{
  switch (baud_rate) {
    case 9600: speed = B9600; return true;
    case 19200: speed = B19200; return true;
    case 38400: speed = B38400; return true;
    case 57600: speed = B57600; return true;
    case 115200: speed = B115200; return true;
    case 230400: speed = B230400; return true;
    default: return false;
  }
}

bool open_raw_serial(const std::string & path, const speed_t speed, int & descriptor)
{
  descriptor = open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (descriptor < 0) {
    return false;
  }
  termios options{};
  if (tcgetattr(descriptor, &options) != 0) {
    close(descriptor);
    descriptor = -1;
    return false;
  }
  cfmakeraw(&options);
  options.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
  options.c_cflag &= static_cast<tcflag_t>(~(PARENB | CSTOPB | CRTSCTS | CSIZE));
  options.c_cflag |= CS8;
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;
  if (cfsetispeed(&options, speed) != 0 || cfsetospeed(&options, speed) != 0 ||
    tcsetattr(descriptor, TCSANOW, &options) != 0)
  {
    close(descriptor);
    descriptor = -1;
    return false;
  }
  tcflush(descriptor, TCIOFLUSH);
  return true;
}

void append_json_string(std::ostringstream & payload, const std::string & value)
{
  payload << "\"";
  for (const auto character : value) {
    switch (character) {
      case '\\': payload << "\\\\"; break;
      case '"': payload << "\\\""; break;
      case '\b': payload << "\\b"; break;
      case '\f': payload << "\\f"; break;
      case '\n': payload << "\\n"; break;
      case '\r': payload << "\\r"; break;
      case '\t': payload << "\\t"; break;
      default: payload << character; break;
    }
  }
  payload << "\"";
}

std::string summarize_raw_bytes(const char * data, const std::size_t size)
{
  constexpr std::size_t kMaxPreviewBytes = 64U;
  std::ostringstream preview;
  preview.imbue(std::locale::classic());
  preview << "len=" << size << " ascii=";
  for (std::size_t index = 0U; index < std::min(size, kMaxPreviewBytes); ++index) {
    const auto value = static_cast<unsigned char>(data[index]);
    if (value >= 0x20U && value <= 0x7EU) {
      preview << static_cast<char>(value);
    } else if (value == '\r') {
      preview << "\\r";
    } else if (value == '\n') {
      preview << "\\n";
    } else {
      preview << "\\x" << std::uppercase << std::hex << std::setw(2) <<
        std::setfill('0') << static_cast<unsigned int>(value) << std::dec <<
        std::setfill(' ');
    }
  }
  if (size > kMaxPreviewBytes) {
    preview << "...";
  }
  return preview.str();
}
}  // namespace

SerialBridgeNode::SerialBridgeNode()
: Node("serial_bridge_node"),
  baud_rate_(115200), hmi_baud_rate_(9600), bridge_rate_hz_(200), vision_rate_hz_(60),
  target_timeout_ms_(100), vision_decimals_(2), placement_decimals_(2),
  max_read_bytes_(256), max_queue_bytes_(4096),
  max_receive_frame_length_(256), gyro_frame_length_(5), coordinate_center_x_(640.0),
  coordinate_center_y_(360.0), width_normalizer_px_(720.0), minimum_confidence_(0.0),
  frequency_tolerance_hz_(5.0), serial_enabled_(false), vision_tx_enabled_(false),
  placement_tx_enabled_(false), gyro_forward_enabled_(false), hmi_enabled_(false),
  hmi_over_mcu_(false),
  apply_width_weight_(true), debug_mode_(false),
  target_received_(false), latest_target_valid_(false), latest_dx_(0.0F), latest_dy_(0.0F),
  gyro_input_fd_(-1), mcu_fd_(-1), hmi_fd_(-1), queued_bytes_(0U), hmi_queued_bytes_(0U),
  next_open_attempt_(std::chrono::steady_clock::now()),
  status_window_start_(std::chrono::steady_clock::now()),
  last_target_update_(std::chrono::steady_clock::now()), bridge_tick_count_(0U),
  vision_transmitted_count_(0U), gyro_transmitted_count_(0U), hmi_transmitted_count_(0U),
  hmi_received_count_(0U), queue_drop_count_(0U), hmi_queue_drop_count_(0U)
{
  gyro_input_port_ = declare_parameter<std::string>("gyro_input_port", "/dev/ttyTHS3");
  mcu_port_ = declare_parameter<std::string>("mcu_port", "/dev/ttyTHS1");
  hmi_port_ = declare_parameter<std::string>("hmi_port", "/dev/ttyTHS3");
  target_topic_ = declare_parameter<std::string>("target_topic", "vision/target_filtered");
  placement_topic_ = declare_parameter<std::string>(
    "placement_topic", "puzzle/placement_request");
  placement_coordinate_frame_ = declare_parameter<std::string>(
    "placement_coordinate_frame", "camera_center_px");
  placement_angle_mode_ = declare_parameter<std::string>(
    "placement_angle_mode", "delta");
  status_topic_ = declare_parameter<std::string>("status_topic", "vision/serial_bridge_status");
  raw_frame_topic_ = declare_parameter<std::string>("raw_frame_topic", "vision/tuning_frames");
  task_command_topic_ = declare_parameter<std::string>(
    "task_command_topic", "puzzle/task_command");
  task_session_topic_ = declare_parameter<std::string>(
    "task_session_topic", "puzzle/task_session");
  placement_done_topic_ = declare_parameter<std::string>(
    "placement_done_topic", "puzzle/placement_done");
  hmi_rx_topic_ = declare_parameter<std::string>("hmi_rx_topic", "vision/hmi/rx_frame");
  hmi_tx_topic_ = declare_parameter<std::string>("hmi_tx_topic", "vision/hmi/tx_bytes");
  baud_rate_ = declare_parameter<int>("baud_rate", baud_rate_);
  hmi_baud_rate_ = declare_parameter<int>("hmi_baud_rate", hmi_baud_rate_);
  bridge_rate_hz_ = declare_parameter<int>("bridge_rate_hz", bridge_rate_hz_);
  vision_rate_hz_ = declare_parameter<int>("vision_rate_hz", vision_rate_hz_);
  target_timeout_ms_ = declare_parameter<int>("target_timeout_ms", target_timeout_ms_);
  vision_decimals_ = declare_parameter<int>("vision_decimals", vision_decimals_);
  placement_decimals_ = declare_parameter<int>("placement_decimals", 2);
  max_read_bytes_ = declare_parameter<int>("max_read_bytes", max_read_bytes_);
  max_queue_bytes_ = declare_parameter<int>("max_queue_bytes", max_queue_bytes_);
  max_receive_frame_length_ = declare_parameter<int>(
    "max_receive_frame_length", max_receive_frame_length_);
  gyro_frame_length_ = declare_parameter<int>("gyro_frame_length", gyro_frame_length_);
  coordinate_center_x_ = declare_parameter<double>("coordinate_center_x", coordinate_center_x_);
  coordinate_center_y_ = declare_parameter<double>("coordinate_center_y", coordinate_center_y_);
  width_normalizer_px_ = declare_parameter<double>("width_normalizer_px", width_normalizer_px_);
  minimum_confidence_ = declare_parameter<double>("minimum_confidence", minimum_confidence_);
  frequency_tolerance_hz_ = declare_parameter<double>(
    "frequency_tolerance_hz", frequency_tolerance_hz_);
  no_target_token_ = declare_parameter<std::string>("no_target_token", "N");
  serial_enabled_ = declare_parameter<bool>("serial_enabled", serial_enabled_);
  vision_tx_enabled_ = declare_parameter<bool>("vision_tx_enabled", vision_tx_enabled_);
  placement_tx_enabled_ = declare_parameter<bool>(
    "placement_tx_enabled", placement_tx_enabled_);
  gyro_forward_enabled_ = declare_parameter<bool>("gyro_forward_enabled", gyro_forward_enabled_);
  hmi_enabled_ = declare_parameter<bool>("hmi_enabled", hmi_enabled_);
  hmi_over_mcu_ = declare_parameter<bool>("hmi_over_mcu", hmi_over_mcu_);
  apply_width_weight_ = declare_parameter<bool>("apply_width_weight", apply_width_weight_);
  debug_mode_ = declare_parameter<bool>("debug_mode", debug_mode_);
  const auto header_values = declare_parameter<std::vector<std::int64_t>>(
    "gyro_frame_header", std::vector<std::int64_t>{0x5A});

  for (const auto value : header_values) {
    if (value >= 0 && value <= 255) {
      gyro_frame_header_.push_back(static_cast<std::uint8_t>(value));
    }
  }
  if (gyro_frame_header_.empty()) {
    gyro_frame_header_.push_back(0x5A);
  }
  bridge_rate_hz_ = std::clamp(bridge_rate_hz_, 1, 1000);
  vision_rate_hz_ = std::clamp(vision_rate_hz_, 1, 120);
  target_timeout_ms_ = std::clamp(target_timeout_ms_, 1, 2000);
  vision_decimals_ = std::clamp(vision_decimals_, 0, 6);
  placement_decimals_ = std::clamp(placement_decimals_, 0, 6);
  max_read_bytes_ = std::clamp(max_read_bytes_, 1, 4096);
  max_queue_bytes_ = std::max(max_read_bytes_, max_queue_bytes_);
  max_receive_frame_length_ = std::clamp(max_receive_frame_length_, 8, 4096);
  gyro_frame_length_ = std::max(static_cast<int>(gyro_frame_header_.size()), gyro_frame_length_);
  width_normalizer_px_ = std::max(1.0, width_normalizer_px_);
  minimum_confidence_ = std::clamp(minimum_confidence_, 0.0, 1.0);
  frequency_tolerance_hz_ = std::max(0.0, frequency_tolerance_hz_);
  if (!serial_bridge_node::is_valid_no_target_token(no_target_token_)) {
    throw std::runtime_error("no_target_token must be printable ASCII without frame delimiters");
  }
  if (placement_coordinate_frame_ != "camera_center_px" &&
    placement_coordinate_frame_ != "a4_mm" && placement_coordinate_frame_ != "workspace_mm")
  {
    throw std::runtime_error(
            "placement_coordinate_frame must be camera_center_px, a4_mm, or workspace_mm");
  }
  if (placement_angle_mode_ != "source" && placement_angle_mode_ != "target" &&
    placement_angle_mode_ != "delta")
  {
    throw std::runtime_error("placement_angle_mode must be source, target, or delta");
  }
  speed_t hmi_speed{};
  if (hmi_enabled_ && !configure_baud_rate(hmi_baud_rate_, hmi_speed)) {
    throw std::runtime_error("hmi_baud_rate is unsupported");
  }
  if (hmi_enabled_ && gyro_forward_enabled_ && hmi_port_ == gyro_input_port_) {
    throw std::runtime_error(
            "hmi_port and gyro_input_port share one UART; disable gyro_forward_enabled "
            "when the HMI screen uses the old gyroscope port");
  }
  if (hmi_enabled_ && !hmi_over_mcu_ && hmi_port_ == mcu_port_) {
    throw std::runtime_error(
            "hmi_port and mcu_port share one UART; set hmi_over_mcu=true or choose "
            "separate UART ports");
  }

  auto qos = rclcpp::SensorDataQoS();
  qos.keep_last(1);
  target_subscription_ = create_subscription<vision_interfaces::msg::TargetPoint>(
    target_topic_, qos, std::bind(&SerialBridgeNode::on_target, this, std::placeholders::_1));
  placement_subscription_ = create_subscription<vision_interfaces::msg::PuzzlePlacement>(
    placement_topic_, 10,
    std::bind(&SerialBridgeNode::on_placement, this, std::placeholders::_1));
  task_session_subscription_ = create_subscription<vision_interfaces::msg::TaskSession>(
    task_session_topic_, 10,
    std::bind(&SerialBridgeNode::on_task_session, this, std::placeholders::_1));
  status_publisher_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);
  raw_frame_publisher_ = create_publisher<std_msgs::msg::String>(raw_frame_topic_, 10);
  task_command_publisher_ = create_publisher<std_msgs::msg::String>(task_command_topic_, 10);
  placement_done_publisher_ = create_publisher<vision_interfaces::msg::PlacementDone>(
    placement_done_topic_, 10);
  hmi_rx_publisher_ = create_publisher<std_msgs::msg::String>(hmi_rx_topic_, 10);
  hmi_tx_subscription_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
    hmi_tx_topic_, 10, std::bind(&SerialBridgeNode::on_hmi_tx, this, std::placeholders::_1));
  bridge_timer_ = create_wall_timer(
    std::chrono::microseconds(1000000 / bridge_rate_hz_),
    std::bind(&SerialBridgeNode::bridge_tick, this));
  vision_timer_ = create_wall_timer(
    std::chrono::microseconds(1000000 / vision_rate_hz_),
    std::bind(&SerialBridgeNode::vision_tick, this));
  RCLCPP_INFO(
    get_logger(),
    "Serial bridge ready: enabled=%s target=%s placement=%s gyro=%s mcu=%s baud=%d "
    "vision_tx=%s placement_tx=%s gyro_forward=%s frame=%s angle=%s hmi=%s hmi_over_mcu=%s "
    "hmi_port=%s hmi_baud=%d",
    serial_enabled_ ? "true" : "false", target_topic_.c_str(), placement_topic_.c_str(),
    gyro_input_port_.c_str(), mcu_port_.c_str(), baud_rate_,
    vision_tx_enabled_ ? "true" : "false", placement_tx_enabled_ ? "true" : "false",
    gyro_forward_enabled_ ? "true" : "false",
    placement_coordinate_frame_.c_str(), placement_angle_mode_.c_str(),
    hmi_enabled_ ? "true" : "false", hmi_over_mcu_ ? "true" : "false",
    hmi_port_.c_str(), hmi_baud_rate_);
}

SerialBridgeNode::~SerialBridgeNode()
{
  close_ports();
}

void SerialBridgeNode::on_target(const vision_interfaces::msg::TargetPoint::ConstSharedPtr message)
{
  const bool finite = std::isfinite(message->x) && std::isfinite(message->y) &&
    std::isfinite(message->side_length);
  latest_target_valid_ = finite && message->confidence >= minimum_confidence_ &&
    (!apply_width_weight_ || message->side_length > 0.0F);
  if (latest_target_valid_) {
    const auto weight = apply_width_weight_ ?
      static_cast<double>(message->side_length) / width_normalizer_px_ : 1.0;
    latest_dx_ = static_cast<float>((message->x - coordinate_center_x_) * weight);
    latest_dy_ = static_cast<float>((message->y - coordinate_center_y_) * weight);
  }
  target_received_ = true;
  last_target_update_ = std::chrono::steady_clock::now();
  if (!latest_target_valid_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "BUG_POINT:SERIAL_QUALITY_GATE rejected confidence=%.3f x=%.2f y=%.2f size=%.2f",
      message->confidence, message->x, message->y, message->side_length);
  }
}

void SerialBridgeNode::on_placement(
  const vision_interfaces::msg::PuzzlePlacement::ConstSharedPtr message)
{
  process_placement(*message);
}

void SerialBridgeNode::on_task_session(
  const vision_interfaces::msg::TaskSession::ConstSharedPtr message)
{
  if (message->generation < active_generation_) {
    return;
  }
  active_task_id_ = message->task_id;
  active_generation_ = message->generation;
  discard_unsent_stale_placements();
  if (deferred_placement_ &&
    (deferred_placement_->task_id != active_task_id_ ||
    deferred_placement_->generation != active_generation_))
  {
    deferred_placement_.reset();
  }
  maybe_send_deferred_placement();
}

void SerialBridgeNode::process_placement(
  const vision_interfaces::msg::PuzzlePlacement & message)
{
  if (!serial_enabled_ || !placement_tx_enabled_) {
    return;
  }
  if (message.task_id == 0U || message.generation == 0U ||
    message.generation < active_generation_ ||
    (message.generation == active_generation_ && message.task_id != active_task_id_))
  {
    RCLCPP_WARN(
      get_logger(), "BUG_POINT:PLACEMENT_SESSION ignored task=%u generation=%u piece=%u; "
      "active_task=%u active_generation=%u",
      message.task_id, message.generation, message.piece_id,
      active_task_id_, active_generation_);
    return;
  }
  if (message.generation > active_generation_) {
    // Placement is authoritative if DDS delivers it before TaskSession on another topic.
    active_task_id_ = message.task_id;
    active_generation_ = message.generation;
    discard_unsent_stale_placements();
  }
  if (move_ack_tracker_.busy()) {
    if (move_ack_tracker_.task_id() == message.task_id &&
      move_ack_tracker_.generation() == message.generation)
    {
      RCLCPP_WARN(
        get_logger(), "BUG_POINT:PLACEMENT_ACK_INFLIGHT duplicate current-session piece=%u "
        "pending=%u",
        message.piece_id, move_ack_tracker_.piece_id());
      return;
    }
    // BUG_POINT:TASK_SWITCH_SERIAL_BARRIER - Do not reuse an untagged [move,ok]
    // while an older task command is still in flight. Keep only the newest task's request.
    deferred_placement_ = message;
    RCLCPP_WARN(
      get_logger(), "BUG_POINT:TASK_SWITCH_SERIAL_BARRIER deferred task=%u generation=%u "
      "piece=%u behind task=%u generation=%u piece=%u",
      message.task_id, message.generation, message.piece_id,
      move_ack_tracker_.task_id(), move_ack_tracker_.generation(),
      move_ack_tracker_.piece_id());
    return;
  }
  const geometry_msgs::msg::Point32 * source = nullptr;
  const geometry_msgs::msg::Point32 * target = nullptr;
  if (placement_coordinate_frame_ == "camera_center_px") {
    source = &message.source_center_camera_px;
    target = &message.target_center_camera_px;
  } else if (placement_coordinate_frame_ == "a4_mm") {
    source = &message.source_center_a4_mm;
    target = &message.target_center_a4_mm;
  } else {
    if (!message.workspace_mapping_valid) {
      // BUG_POINT:PLACEMENT_MAPPING - Never serialize local coordinates as absolute millimetres.
      RCLCPP_ERROR(
        get_logger(), "BUG_POINT:PLACEMENT_MAPPING piece=%u workspace mapping invalid",
        message.piece_id);
      return;
    }
    source = &message.source_center_workspace_mm;
    target = &message.target_center_workspace_mm;
  }

  const double angle = placement_angle_mode_ == "source" ? message.source_angle_deg :
    (placement_angle_mode_ == "target" ? message.target_angle_deg : message.rotation_delta_deg);
  try {
    const auto frame = serial_bridge_node::format_placement_frame(
      source->x, source->y, target->x, target->y, angle, placement_decimals_);
    if (!move_ack_tracker_.begin_placement(
        message.piece_id, message.task_id, message.generation))
    {
      RCLCPP_ERROR(
        get_logger(), "BUG_POINT:PLACEMENT_ACK_STATE unable to begin task=%u generation=%u "
        "piece=%u",
        message.task_id, message.generation, message.piece_id);
      return;
    }
    if (enqueue_frame(
        std::vector<std::uint8_t>(frame.begin(), frame.end()), "placement",
        message.piece_id, message.task_id, message.generation))
    {
      last_vision_frame_ = frame;
      if (debug_mode_) {
        RCLCPP_INFO(
          get_logger(), "PLACEMENT_ASCII piece=%u frame=%s frame_id=%s angle_mode=%s",
          message.piece_id, frame.c_str(), placement_coordinate_frame_.c_str(),
          placement_angle_mode_.c_str());
      }
    } else {
      move_ack_tracker_.cancel();
    }
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      get_logger(), "BUG_POINT:PLACEMENT_FRAME piece=%u %s",
      message.piece_id, exception.what());
  }
}

void SerialBridgeNode::discard_unsent_stale_placements()
{
  for (auto iterator = transmit_queue_.begin(); iterator != transmit_queue_.end(); ) {
    const bool stale_placement = iterator->source == "placement" &&
      (iterator->task_id != active_task_id_ || iterator->generation != active_generation_);
    if (!stale_placement || iterator->written_bytes != 0U) {
      ++iterator;
      continue;
    }
    queued_bytes_ -= iterator->bytes.size();
    if (move_ack_tracker_.queued() &&
      move_ack_tracker_.task_id() == iterator->task_id &&
      move_ack_tracker_.generation() == iterator->generation)
    {
      move_ack_tracker_.cancel();
    }
    iterator = transmit_queue_.erase(iterator);
  }
}

void SerialBridgeNode::maybe_send_deferred_placement()
{
  if (!deferred_placement_ || move_ack_tracker_.busy() ||
    deferred_placement_->task_id != active_task_id_ ||
    deferred_placement_->generation != active_generation_ || active_task_id_ == 0U)
  {
    return;
  }
  const auto placement = *deferred_placement_;
  deferred_placement_.reset();
  process_placement(placement);
}

void SerialBridgeNode::on_hmi_tx(const std_msgs::msg::UInt8MultiArray::ConstSharedPtr message)
{
  if (!hmi_enabled_) {
    return;
  }
  // BUG_POINT:HMI_TX_PATH -- HMI logic must publish here; only this bridge touches UART bytes.
  auto bytes = std::vector<std::uint8_t>(message->data.begin(), message->data.end());
  if (hmi_over_mcu_) {
    // BUG_POINT:HMI_OVER_MCU - Screen replies share the MCU UART only in forwarded topology.
    enqueue_frame(std::move(bytes), "hmi");
    return;
  }
  enqueue_hmi_frame(std::move(bytes));
}

void SerialBridgeNode::bridge_tick()
{
  ++bridge_tick_count_;
  publish_status();
  if (!serial_enabled_) {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  if ((mcu_fd_ < 0 || (hmi_enabled_ && !hmi_over_mcu_ && hmi_fd_ < 0)) &&
    now >= next_open_attempt_)
  {
    if (!open_ports()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "BUG_POINT:SERIAL_OPEN unable to open one or more enabled UART ports");
    }
    next_open_attempt_ = now + std::chrono::seconds(2);
  }
  if (gyro_forward_enabled_ && gyro_input_fd_ < 0 && now >= next_open_attempt_) {
    try_open_gyro();
    next_open_attempt_ = now + std::chrono::seconds(2);
  }

  if (mcu_fd_ >= 0 && gyro_input_fd_ >= 0) {
    std::array<std::uint8_t, 4096> incoming{};
    const auto received = read(gyro_input_fd_, incoming.data(), static_cast<std::size_t>(max_read_bytes_));
    if (received > 0) {
      gyro_receive_buffer_.insert(
        gyro_receive_buffer_.end(), incoming.begin(), incoming.begin() + received);
      extract_gyro_frames();
    } else if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      RCLCPP_WARN(get_logger(), "BUG_POINT:SERIAL_GYRO_READ %s", std::strerror(errno));
      close(gyro_input_fd_);
      gyro_input_fd_ = -1;
    }
  }

  if (mcu_fd_ >= 0) {
    std::array<char, 512> mcu_input{};
    const auto mcu_received = read(mcu_fd_, mcu_input.data(), mcu_input.size());
    if (mcu_received > 0) {
      record_mcu_raw_bytes(mcu_input.data(), static_cast<std::size_t>(mcu_received));
      consume_mcu_bytes(mcu_input.data(), static_cast<std::size_t>(mcu_received));
    } else if (mcu_received < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      RCLCPP_WARN(get_logger(), "BUG_POINT:SERIAL_MCU_READ %s", std::strerror(errno));
      close_ports();
      return;
    }
    flush_transmit_queue();
  }

  if (hmi_enabled_ && !hmi_over_mcu_ && hmi_fd_ >= 0) {
    std::array<char, 512> hmi_input{};
    const auto hmi_received = read(hmi_fd_, hmi_input.data(), hmi_input.size());
    if (hmi_received > 0) {
      consume_hmi_bytes(hmi_input.data(), static_cast<std::size_t>(hmi_received));
    } else if (hmi_received < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      RCLCPP_WARN(get_logger(), "BUG_POINT:HMI_READ %s", std::strerror(errno));
      close(hmi_fd_);
      hmi_fd_ = -1;
      hmi_receive_buffer_.clear();
    }
    flush_hmi_transmit_queue();
  }
}

void SerialBridgeNode::vision_tick()
{
  if (!serial_enabled_ || !vision_tx_enabled_ || mcu_fd_ < 0) {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  const bool fresh = target_received_ &&
    std::chrono::duration_cast<std::chrono::milliseconds>(now - last_target_update_).count() <=
    target_timeout_ms_;
  const bool detected = fresh && latest_target_valid_;
  const auto timestamp_ms = static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count());
  try {
    const auto frame = serial_bridge_node::format_vision_frame(
      detected, latest_dx_, latest_dy_, timestamp_ms, vision_decimals_, no_target_token_);
    if (enqueue_frame(std::vector<std::uint8_t>(frame.begin(), frame.end()), "vision")) {
      last_vision_frame_ = frame;
    }
    if (debug_mode_) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 1000, "VISION_ASCII frame=%s detected=%s fresh=%s",
        frame.c_str(), detected ? "true" : "false", fresh ? "true" : "false");
    }
  } catch (const std::exception & exception) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "BUG_POINT:SERIAL_FRAME_FORMAT %s", exception.what());
  }
}

bool SerialBridgeNode::open_ports()
{
  bool any_open = false;
  speed_t speed = B115200;
  if (!configure_baud_rate(baud_rate_, speed)) {
    RCLCPP_ERROR(get_logger(), "BUG_POINT:SERIAL_BAUD unsupported baud=%d", baud_rate_);
    return false;
  }
  if (mcu_fd_ < 0 && !open_raw_serial(mcu_port_, speed, mcu_fd_)) {
    mcu_fd_ = -1;
  } else if (mcu_fd_ >= 0) {
    any_open = true;
  }
  if (mcu_fd_ >= 0 && gyro_forward_enabled_ && gyro_input_fd_ < 0 &&
    !open_raw_serial(gyro_input_port_, speed, gyro_input_fd_))
  {
    gyro_input_fd_ = -1;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "BUG_POINT:SERIAL_GYRO_OPEN unavailable: %s", gyro_input_port_.c_str());
  }
  if (mcu_fd_ >= 0) {
    RCLCPP_INFO(get_logger(), "Serial MCU port connected: %s", mcu_port_.c_str());
  }
  if (hmi_enabled_ && !hmi_over_mcu_ && hmi_fd_ < 0) {
    any_open = try_open_hmi() || any_open;
  }
  return any_open;
}

bool SerialBridgeNode::try_open_hmi()
{
  speed_t speed = B9600;
  if (!configure_baud_rate(hmi_baud_rate_, speed) ||
    !open_raw_serial(hmi_port_, speed, hmi_fd_))
  {
    hmi_fd_ = -1;
    return false;
  }
  RCLCPP_INFO(get_logger(), "HMI serial port connected: %s", hmi_port_.c_str());
  return true;
}

bool SerialBridgeNode::try_open_gyro()
{
  speed_t speed = B115200;
  if (!configure_baud_rate(baud_rate_, speed) ||
    !open_raw_serial(gyro_input_port_, speed, gyro_input_fd_))
  {
    gyro_input_fd_ = -1;
    return false;
  }
  RCLCPP_INFO(get_logger(), "Gyroscope input connected: %s", gyro_input_port_.c_str());
  return true;
}

void SerialBridgeNode::close_ports()
{
  if (gyro_input_fd_ >= 0) {
    close(gyro_input_fd_);
    gyro_input_fd_ = -1;
  }
  if (mcu_fd_ >= 0) {
    close(mcu_fd_);
    mcu_fd_ = -1;
  }
  if (hmi_fd_ >= 0) {
    close(hmi_fd_);
    hmi_fd_ = -1;
  }
  gyro_receive_buffer_.clear();
  mcu_receive_buffer_.clear();
  hmi_receive_buffer_.clear();
  transmit_queue_.clear();
  hmi_transmit_queue_.clear();
  queued_bytes_ = 0U;
  hmi_queued_bytes_ = 0U;
}

void SerialBridgeNode::extract_gyro_frames()
{
  while (gyro_receive_buffer_.size() >= gyro_frame_header_.size()) {
    const auto header = std::search(
      gyro_receive_buffer_.begin(), gyro_receive_buffer_.end(),
      gyro_frame_header_.begin(), gyro_frame_header_.end());
    if (header == gyro_receive_buffer_.end()) {
      const auto preserve = gyro_frame_header_.size() - 1U;
      if (gyro_receive_buffer_.size() > preserve) {
        gyro_receive_buffer_.erase(
          gyro_receive_buffer_.begin(),
          gyro_receive_buffer_.end() - static_cast<std::ptrdiff_t>(preserve));
      }
      return;
    }
    if (header != gyro_receive_buffer_.begin()) {
      gyro_receive_buffer_.erase(gyro_receive_buffer_.begin(), header);
    }
    if (gyro_receive_buffer_.size() < static_cast<std::size_t>(gyro_frame_length_)) {
      return;
    }
    std::vector<std::uint8_t> frame(
      gyro_receive_buffer_.begin(), gyro_receive_buffer_.begin() + gyro_frame_length_);
    gyro_receive_buffer_.erase(
      gyro_receive_buffer_.begin(), gyro_receive_buffer_.begin() + gyro_frame_length_);
    // BUG_POINT:SERIAL_GYRO_TRANSPARENT -- The payload bytes are never parsed or modified.
    enqueue_frame(std::move(frame), "gyro");
  }
}

void SerialBridgeNode::consume_mcu_bytes(const char * data, const std::size_t size)
{
  mcu_receive_buffer_.append(data, size);
  while (true) {
    const auto begin = mcu_receive_buffer_.find('[');
    if (begin == std::string::npos) {
      if (mcu_receive_buffer_.size() > static_cast<std::size_t>(max_receive_frame_length_)) {
        mcu_discarded_bytes_count_ += mcu_receive_buffer_.size();
        RCLCPP_WARN(
          get_logger(),
          "BUG_POINT:SERIAL_MCU_FRAME dropped %zu bytes without '['; last_raw=%s",
          mcu_receive_buffer_.size(),
          recent_mcu_raw_chunks_.empty() ? "" : recent_mcu_raw_chunks_.back().c_str());
        mcu_receive_buffer_.clear();
      }
      return;
    }
    if (begin > 0U) {
      mcu_discarded_bytes_count_ += begin;
      mcu_receive_buffer_.erase(0U, begin);
    }
    const auto end = mcu_receive_buffer_.find(']', 1U);
    if (end == std::string::npos) {
      if (mcu_receive_buffer_.size() > static_cast<std::size_t>(max_receive_frame_length_)) {
        RCLCPP_WARN(get_logger(), "BUG_POINT:SERIAL_MCU_FRAME oversized incomplete frame");
        mcu_receive_buffer_.clear();
      }
      return;
    }
    publish_received_frame(mcu_receive_buffer_.substr(0U, end + 1U));
    mcu_receive_buffer_.erase(0U, end + 1U);
  }
}

void SerialBridgeNode::publish_received_frame(const std::string & frame)
{
  record_received_mcu_frame(frame);
  std_msgs::msg::String message;
  message.data = frame;
  raw_frame_publisher_->publish(message);
  if (hmi_enabled_ && hmi_over_mcu_ && frame != "[move,ok]") {
    // BUG_POINT:HMI_OVER_MCU - Forwarded screen frames need HMI ACK/status handling.
    publish_hmi_frame(frame);
    return;
  }
  int task_number = 0;
  if (serial_bridge_node::parse_task_command(frame, task_number)) {
    task_command_publisher_->publish(message);
    RCLCPP_INFO(get_logger(), "MCU task command accepted: task=%d", task_number);
  } else if (frame.compare(0U, 6U, "[task,") == 0U) {
    // BUG_POINT:TASK_COMMAND_FORMAT - Reject malformed or out-of-range task frames.
    RCLCPP_WARN(get_logger(), "BUG_POINT:TASK_COMMAND_FORMAT rejected frame=%s", frame.c_str());
  } else if (frame == "[move,ok]") {
    std::uint32_t completed_piece_id = 0U;
    std::uint8_t completed_task_id = 0U;
    std::uint32_t completed_generation = 0U;
    if (move_ack_tracker_.accept_frame(
        frame, completed_piece_id, completed_task_id, completed_generation))
    {
      vision_interfaces::msg::PlacementDone completed;
      completed.task_id = completed_task_id;
      completed.generation = completed_generation;
      completed.piece_id = completed_piece_id;
      placement_done_publisher_->publish(completed);
      ++placement_ack_count_;
      RCLCPP_INFO(
        get_logger(), "MCU move acknowledgement accepted: task=%u generation=%u piece=%u",
        completed_task_id, completed_generation, completed_piece_id);
      maybe_send_deferred_placement();
    } else {
      // BUG_POINT:MOVE_ACK_STATE - Ignore duplicate, early, or stale acknowledgements.
      RCLCPP_WARN(
        get_logger(), "BUG_POINT:MOVE_ACK_STATE ignored frame=%s pending=%u waiting=%s",
        frame.c_str(), move_ack_tracker_.piece_id(),
        move_ack_tracker_.waiting_ack() ? "true" : "false");
    }
  }
}

void SerialBridgeNode::consume_hmi_bytes(const char * data, const std::size_t size)
{
  hmi_receive_buffer_.append(data, size);
  while (true) {
    const auto begin = hmi_receive_buffer_.find('[');
    if (begin == std::string::npos) {
      if (hmi_receive_buffer_.size() > static_cast<std::size_t>(max_receive_frame_length_)) {
        hmi_receive_buffer_.clear();
      }
      return;
    }
    if (begin > 0U) {
      hmi_receive_buffer_.erase(0U, begin);
    }
    const auto end = hmi_receive_buffer_.find(']', 1U);
    if (end == std::string::npos) {
      if (hmi_receive_buffer_.size() > static_cast<std::size_t>(max_receive_frame_length_)) {
        RCLCPP_WARN(get_logger(), "BUG_POINT:HMI_FRAME oversized incomplete frame");
        hmi_receive_buffer_.clear();
      }
      return;
    }
    publish_hmi_frame(hmi_receive_buffer_.substr(0U, end + 1U));
    hmi_receive_buffer_.erase(0U, end + 1U);
  }
}

void SerialBridgeNode::publish_hmi_frame(const std::string & frame)
{
  ++hmi_received_count_;
  std_msgs::msg::String message;
  message.data = frame;
  hmi_rx_publisher_->publish(message);
}

void SerialBridgeNode::record_sent_vision_frame(const std::string & frame)
{
  recent_sent_vision_frames_.push_back(frame);
  while (recent_sent_vision_frames_.size() > 10U) {
    recent_sent_vision_frames_.pop_front();
  }
}

void SerialBridgeNode::record_mcu_raw_bytes(const char * data, const std::size_t size)
{
  mcu_received_bytes_count_ += size;
  recent_mcu_raw_chunks_.push_back(summarize_raw_bytes(data, size));
  while (recent_mcu_raw_chunks_.size() > 10U) {
    recent_mcu_raw_chunks_.pop_front();
  }
}

void SerialBridgeNode::record_received_mcu_frame(const std::string & frame)
{
  ++mcu_received_frame_count_;
  recent_received_mcu_frames_.push_back(frame);
  while (recent_received_mcu_frames_.size() > 10U) {
    recent_received_mcu_frames_.pop_front();
  }
}

bool SerialBridgeNode::enqueue_frame(
  std::vector<std::uint8_t> bytes, const std::string & source,
  const std::uint32_t piece_id, const std::uint8_t task_id,
  const std::uint32_t generation)
{
  if (bytes.empty()) {
    return true;
  }
  if (bytes.size() > static_cast<std::size_t>(max_queue_bytes_) ||
    queued_bytes_ + bytes.size() > static_cast<std::size_t>(max_queue_bytes_))
  {
    ++queue_drop_count_;
    // BUG_POINT:SERIAL_TX_QUEUE -- Drop a complete frame rather than interleaving bytes.
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "BUG_POINT:SERIAL_TX_QUEUE dropping %s bytes=%zu queued=%zu",
      source.c_str(), bytes.size(), queued_bytes_);
    return false;
  }
  queued_bytes_ += bytes.size();
  transmit_queue_.push_back(
    PendingFrame{std::move(bytes), 0U, source, piece_id, task_id, generation});
  return true;
}

bool SerialBridgeNode::enqueue_hmi_frame(std::vector<std::uint8_t> bytes)
{
  if (bytes.empty()) {
    return true;
  }
  if (bytes.size() > static_cast<std::size_t>(max_queue_bytes_) ||
    hmi_queued_bytes_ + bytes.size() > static_cast<std::size_t>(max_queue_bytes_))
  {
    ++hmi_queue_drop_count_;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "BUG_POINT:HMI_TX_QUEUE dropping bytes=%zu queued=%zu", bytes.size(), hmi_queued_bytes_);
    return false;
  }
  hmi_queued_bytes_ += bytes.size();
  hmi_transmit_queue_.push_back(PendingFrame{std::move(bytes), 0U, "hmi"});
  return true;
}

bool SerialBridgeNode::flush_transmit_queue()
{
  while (!transmit_queue_.empty()) {
    auto & frame = transmit_queue_.front();
    const auto remaining = frame.bytes.size() - frame.written_bytes;
    const auto written = write(mcu_fd_, frame.bytes.data() + frame.written_bytes, remaining);
    if (written > 0) {
      frame.written_bytes += static_cast<std::size_t>(written);
      if (frame.written_bytes != frame.bytes.size()) {
        continue;
      }
      if (frame.source == "vision" || frame.source == "placement") {
        ++vision_transmitted_count_;
        record_sent_vision_frame(
          std::string(frame.bytes.begin(), frame.bytes.end()));
        if (frame.source == "placement" &&
          !move_ack_tracker_.mark_transmitted(
            frame.piece_id, frame.task_id, frame.generation))
        {
          RCLCPP_ERROR(
            get_logger(), "BUG_POINT:PLACEMENT_ACK_STATE transmitted piece=%u pending=%u",
            frame.piece_id, move_ack_tracker_.piece_id());
        }
      } else if (frame.source == "gyro") {
        ++gyro_transmitted_count_;
      } else if (frame.source == "hmi") {
        ++hmi_transmitted_count_;
      }
      queued_bytes_ -= frame.bytes.size();
      transmit_queue_.pop_front();
      continue;
    }
    if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      RCLCPP_WARN(get_logger(), "BUG_POINT:SERIAL_WRITE %s", std::strerror(errno));
      close_ports();
      return false;
    }
    return true;
  }
  return true;
}

bool SerialBridgeNode::flush_hmi_transmit_queue()
{
  while (!hmi_transmit_queue_.empty()) {
    auto & frame = hmi_transmit_queue_.front();
    const auto remaining = frame.bytes.size() - frame.written_bytes;
    const auto written = write(hmi_fd_, frame.bytes.data() + frame.written_bytes, remaining);
    if (written > 0) {
      frame.written_bytes += static_cast<std::size_t>(written);
      if (frame.written_bytes != frame.bytes.size()) {
        continue;
      }
      ++hmi_transmitted_count_;
      hmi_queued_bytes_ -= frame.bytes.size();
      hmi_transmit_queue_.pop_front();
      continue;
    }
    if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      RCLCPP_WARN(get_logger(), "BUG_POINT:HMI_WRITE %s", std::strerror(errno));
      close(hmi_fd_);
      hmi_fd_ = -1;
      hmi_receive_buffer_.clear();
      hmi_transmit_queue_.clear();
      hmi_queued_bytes_ = 0U;
      return false;
    }
    return true;
  }
  return true;
}

void SerialBridgeNode::publish_status()
{
  const auto now = std::chrono::steady_clock::now();
  const auto elapsed = std::chrono::duration<double>(now - status_window_start_).count();
  if (elapsed < 1.0) {
    return;
  }
  const auto observed_hz = static_cast<double>(bridge_tick_count_) / elapsed;
  std_msgs::msg::String status;
  std::ostringstream payload;
  payload << std::fixed << std::setprecision(1)
          << "{\"enabled\":" << (serial_enabled_ ? "true" : "false")
          << ",\"placement_tx_enabled\":" << (placement_tx_enabled_ ? "true" : "false")
          << ",\"gyro_forward_enabled\":" << (gyro_forward_enabled_ ? "true" : "false")
          << ",\"hmi_enabled\":" << (hmi_enabled_ ? "true" : "false")
          << ",\"hmi_over_mcu\":" << (hmi_over_mcu_ ? "true" : "false")
          << ",\"placement_coordinate_frame\":\"" << placement_coordinate_frame_ << "\""
          << ",\"placement_angle_mode\":\"" << placement_angle_mode_ << "\""
          << ",\"hmi_port\":";
  append_json_string(payload, hmi_port_);
  payload << ",\"gyro_input_port\":";
  append_json_string(payload, gyro_input_port_);
  payload << ",\"mcu_port\":";
  append_json_string(payload, mcu_port_);
  payload << ",\"hmi_baud_rate\":" << hmi_baud_rate_
          << ",\"configured_hz\":" << bridge_rate_hz_
          << ",\"observed_hz\":" << observed_hz
          << ",\"frequency_ok\":" <<
    (std::abs(observed_hz - bridge_rate_hz_) <= frequency_tolerance_hz_ ? "true" : "false")
          << ",\"vision_observed_hz\":" << static_cast<double>(vision_transmitted_count_) / elapsed
          << ",\"gyro_forwarded_hz\":" << static_cast<double>(gyro_transmitted_count_) / elapsed
          << ",\"mcu_connected\":" << (mcu_fd_ >= 0 ? "true" : "false")
          << ",\"gyro_connected\":" << (gyro_input_fd_ >= 0 ? "true" : "false")
          << ",\"hmi_connected\":" << (hmi_fd_ >= 0 ? "true" : "false")
          << ",\"hmi_rx_hz\":" << static_cast<double>(hmi_received_count_) / elapsed
          << ",\"hmi_tx_hz\":" << static_cast<double>(hmi_transmitted_count_) / elapsed
          << ",\"mcu_rx_bytes_hz\":" << static_cast<double>(mcu_received_bytes_count_) / elapsed
          << ",\"mcu_rx_frames_hz\":" << static_cast<double>(mcu_received_frame_count_) / elapsed
          << ",\"mcu_discarded_bytes_hz\":" <<
    static_cast<double>(mcu_discarded_bytes_count_) / elapsed
          << ",\"queue_frames\":" << transmit_queue_.size()
          << ",\"queue_drops\":" << queue_drop_count_
          << ",\"placement_ack_pending\":" <<
    (move_ack_tracker_.waiting_ack() ? "true" : "false")
          << ",\"placement_ack_piece_id\":" << move_ack_tracker_.piece_id()
          << ",\"placement_ack_task_id\":" <<
    static_cast<unsigned int>(move_ack_tracker_.task_id())
          << ",\"placement_ack_generation\":" << move_ack_tracker_.generation()
          << ",\"active_task\":" << static_cast<unsigned int>(active_task_id_)
          << ",\"active_generation\":" << active_generation_
          << ",\"deferred_placement\":" << (deferred_placement_ ? "true" : "false")
          << ",\"placement_ack_count\":" << placement_ack_count_
          << ",\"hmi_queue_frames\":" << hmi_transmit_queue_.size()
          << ",\"hmi_queue_drops\":" << hmi_queue_drop_count_
          << ",\"last_vision_frame\":";
  append_json_string(payload, last_vision_frame_);
  payload << ",\"recent_sent_vision_frames\":[";
  for (std::size_t index = 0U; index < recent_sent_vision_frames_.size(); ++index) {
    if (index > 0U) {
      payload << ",";
    }
    append_json_string(payload, recent_sent_vision_frames_[index]);
  }
  payload << "],\"recent_mcu_raw_chunks\":[";
  for (std::size_t index = 0U; index < recent_mcu_raw_chunks_.size(); ++index) {
    if (index > 0U) {
      payload << ",";
    }
    append_json_string(payload, recent_mcu_raw_chunks_[index]);
  }
  payload << "],\"recent_received_mcu_frames\":[";
  for (std::size_t index = 0U; index < recent_received_mcu_frames_.size(); ++index) {
    if (index > 0U) {
      payload << ",";
    }
    append_json_string(payload, recent_received_mcu_frames_[index]);
  }
  payload << "]}";
  status.data = payload.str();
  status_publisher_->publish(status);
  if (debug_mode_) {
    RCLCPP_DEBUG(get_logger(), "BUG_POINT:SERIAL_STATUS %s", status.data.c_str());
  }
  bridge_tick_count_ = 0U;
  vision_transmitted_count_ = 0U;
  gyro_transmitted_count_ = 0U;
  hmi_transmitted_count_ = 0U;
  hmi_received_count_ = 0U;
  mcu_received_bytes_count_ = 0U;
  mcu_received_frame_count_ = 0U;
  mcu_discarded_bytes_count_ = 0U;
  status_window_start_ = now;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
