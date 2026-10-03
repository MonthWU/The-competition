"""Remove small object boxes before preview and serial consumers receive them."""

import time

import rclpy
from ai_msgs.msg import PerceptionTargets
from rcl_interfaces.msg import IntegerRange, ParameterDescriptor
from rclpy.node import Node


def filter_targets_by_area(targets, minimum_area):
    """Keep valid primary boxes with width * height >= minimum_area pixels²."""
    if minimum_area < 0:
        raise ValueError("minimum_area must be nonnegative")
    kept = []
    for target in targets:
        if not target.rois:
            continue
        rect = target.rois[0].rect
        if (rect.width > 0 and rect.height > 0
                and rect.width * rect.height >= minimum_area):
            kept.append(target)
    return kept


class TargetAreaFilter(Node):
    """Filter both inference engines using their image-coordinate boxes."""

    def __init__(self):
        super().__init__("obj_target_area_filter")
        self.declare_parameter("input_topic", "hobot_dnn_detection_raw")
        self.declare_parameter("output_topic", "hobot_dnn_detection")
        self.declare_parameter(
            "min_target_area_px", 2000,
            ParameterDescriptor(
                description="Minimum primary box area in original image pixels squared; 0 disables the size gate.",
                integer_range=[IntegerRange(from_value=0, to_value=2147483647, step=1)],
            ),
        )
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        if input_topic == output_topic:
            raise ValueError("input_topic and output_topic must differ")
        self.publisher = self.create_publisher(PerceptionTargets, output_topic, 10)
        self.subscription = self.create_subscription(
            PerceptionTargets, input_topic, self.on_targets, 10
        )
        self._last_log = time.monotonic()
        self._dropped = 0
        self.get_logger().info(
            f"area filter ready: {input_topic} -> {output_topic}, "
            f"min_target_area_px={self.get_parameter('min_target_area_px').value}"
        )

    def on_targets(self, message):
        minimum_area = self.get_parameter("min_target_area_px").value
        original_count = len(message.targets)
        message.targets = filter_targets_by_area(message.targets, minimum_area)
        self._dropped += original_count - len(message.targets)
        # Forward empty frames as well, preserving timestamps and performance data.
        self.publisher.publish(message)
        now = time.monotonic()
        if self._dropped and now - self._last_log >= 1.0:
            self.get_logger().info(
                f"area filter dropped {self._dropped} target(s); "
                f"min_target_area_px={minimum_area}"
            )
            self._dropped = 0
            self._last_log = now


def main(args=None):
    rclpy.init(args=args)
    node = TargetAreaFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
