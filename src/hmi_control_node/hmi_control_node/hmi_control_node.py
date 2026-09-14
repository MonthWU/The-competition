"""Parse HMI screen commands and emit response bytes through the serial bridge."""

from dataclasses import dataclass
import json
import os
from pathlib import Path
import re
import tempfile
from typing import Dict, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray


TERMINATOR = bytes((0xFF, 0xFF, 0xFF))


@dataclass(frozen=True)
class CommandSpec:
    minimum: int
    maximum: int
    roam: str
    default: int
    persistent_parameter: str
    restart_required: bool = False


COMMAND_SPECS: Dict[str, CommandSpec] = {
    "debug": CommandSpec(0, 1, "sw0", 1, "hmi_value_debug", True),
    # Default to long exposure; an explicit flash=0 remains the short-exposure command.
    "flash": CommandSpec(0, 1, "sw1", 1, "hmi_value_flash"),
    "opx": CommandSpec(1, 16, "n0", 1, "hmi_value_opx"),
    "light": CommandSpec(0, 255, "n1", 128, "hmi_value_light"),
    "bound": CommandSpec(0, 255, "n2", 128, "hmi_value_bound"),
    "huidu": CommandSpec(0, 255, "n0", 128, "hmi_value_huidu"),
    "HSVb": CommandSpec(0, 255, "n1", 128, "hmi_value_HSVb"),
    "HSVl": CommandSpec(0, 255, "n2", 128, "hmi_value_HSVl"),
    "confps": CommandSpec(1, 5, "n0", 1, "hmi_value_confps"),
    "losfps": CommandSpec(1, 10, "n1", 1, "hmi_value_losfps"),
    "jumplmt": CommandSpec(10, 200, "n2", 50, "hmi_value_jumplmt"),
    "CAl": CommandSpec(20, 150, "n0", 50, "hmi_value_CAl"),
    "CAh": CommandSpec(80, 300, "n1", 150, "hmi_value_CAh"),
    "thres": CommandSpec(20, 150, "n2", 80, "hmi_value_thres"),
}


class HmiControlNode(Node):
    """HMI protocol state machine.

    BUG_POINT:HMI_SERIAL_BOUNDARY - This node never opens UART. It receives decoded
    bracket frames from serial_bridge_node and publishes response bytes back to it.
    BUG_POINT:HMI_YAML_PERSISTENCE - Every valid screen parameter write must update
    the deployed vision_system.yaml before ACK, so reboot keeps the selected value.
    BUG_POINT:HMI_EXPOSURE_REOPEN - flash is persisted before this node publishes the
    runtime command that asks puzzle_perception_node to reopen the camera.
    """

    def __init__(self) -> None:
        super().__init__("hmi_control_node")
        self.rx_topic = self.declare_parameter("hmi_rx_topic", "vision/hmi/rx_frame").value
        self.tx_topic = self.declare_parameter("hmi_tx_topic", "vision/hmi/tx_bytes").value
        self.status_topic = self.declare_parameter("hmi_status_topic", "vision/hmi/status").value
        self.command_topic = self.declare_parameter("hmi_command_topic", "vision/hmi/command").value
        self.task_command_topic = self.declare_parameter(
            "task_command_topic", "puzzle/task_command"
        ).value
        self.ack_text = self.declare_parameter("ack_text", "vis ACK,1").value
        self.error_ack_text = self.declare_parameter("error_ack_text", "vis ACK,0").value
        self.persistent_config_path = self.declare_parameter(
            "hmi_persistent_config_path", ""
        ).value
        self.debug_mode = bool(self.declare_parameter("debug_mode", False).value)

        self.values = {}
        for name, spec in COMMAND_SPECS.items():
            value = int(
                self.declare_parameter(spec.persistent_parameter, spec.default).value
            )
            self.values[name] = min(max(value, spec.minimum), spec.maximum)
        self.tx_publisher = self.create_publisher(UInt8MultiArray, self.tx_topic, 10)
        self.status_publisher = self.create_publisher(String, self.status_topic, 10)
        self.command_publisher = self.create_publisher(String, self.command_topic, 10)
        self.task_command_publisher = self.create_publisher(String, self.task_command_topic, 10)
        self.frame_subscription = self.create_subscription(
            String, self.rx_topic, self._on_frame, 10
        )
        self.get_logger().info(
            f"HMI control ready: rx={self.rx_topic} tx={self.tx_topic} "
            f"task={self.task_command_topic} commands={len(COMMAND_SPECS)} "
            f"persistent_yaml={self._persistent_yaml_path()}"
        )

    def _on_frame(self, message: String) -> None:
        frame = message.data.strip()
        parsed = self._parse_frame(frame)
        if parsed is None:
            self._publish_response(self.error_ack_text)
            self._publish_status("invalid", frame, "")
            return
        cmd, val = parsed
        if cmd == "get":
            self._handle_get(val, frame)
            return
        if cmd == "task":
            self._handle_task(val, frame)
            return
        self._handle_set(cmd, val, frame)

    def _parse_frame(self, frame: str) -> Optional[tuple[str, str]]:
        if not frame.startswith("[") or not frame.endswith("]"):
            return None
        body = frame[1:-1].strip()
        if "," not in body:
            return None
        cmd, val = body.split(",", 1)
        cmd = cmd.strip()
        val = val.strip()
        if not cmd or not val:
            return None
        return cmd, val

    def _handle_set(self, cmd: str, val: str, frame: str) -> None:
        spec = COMMAND_SPECS.get(cmd)
        if spec is None:
            self._publish_response(self.error_ack_text)
            self._publish_status("unknown", frame, cmd)
            return
        try:
            numeric_value = int(val, 10)
        except ValueError:
            self._publish_response(self.error_ack_text)
            self._publish_status("not_integer", frame, cmd)
            return
        if numeric_value < spec.minimum or numeric_value > spec.maximum:
            self._publish_response(self.error_ack_text)
            self._publish_status("out_of_range", frame, cmd)
            return
        try:
            self._persist_value(spec, numeric_value)
        except Exception as exception:
            self.get_logger().error(
                f"BUG_POINT:HMI_YAML_PERSISTENCE failed cmd={cmd}: {exception}"
            )
            self._publish_response(self.error_ack_text)
            self._publish_status("persist_failed", frame, cmd)
            return
        self.values[cmd] = numeric_value
        self._publish_command(cmd, numeric_value)
        self._publish_response(self.ack_text)
        state = "set_restart_required" if spec.restart_required else "set"
        self._publish_status(state, frame, cmd)

    def _handle_get(self, cmd: str, frame: str) -> None:
        spec = COMMAND_SPECS.get(cmd)
        if spec is None:
            self._publish_response(self.error_ack_text)
            self._publish_status("unknown_get", frame, cmd)
            return
        self._publish_response(f"{spec.roam}.val={self.values[cmd]}")
        self._publish_status("get", frame, cmd)

    def _handle_task(self, val: str, frame: str) -> None:
        if val not in ("1", "2", "3"):
            # BUG_POINT:HMI_TASK_COMMAND - The coordinator accepts only [task,1..3].
            self._publish_response(self.error_ack_text)
            self._publish_status("task_out_of_range", frame, "task")
            return
        command = String()
        command.data = f"[task,{val}]"
        self.task_command_publisher.publish(command)
        self._publish_response(self.ack_text)
        self._publish_status("task", frame, "task")

    def _publish_response(self, text: str) -> None:
        payload = text.encode("ascii", errors="strict") + TERMINATOR
        message = UInt8MultiArray()
        message.data = list(payload)
        self.tx_publisher.publish(message)

    def _publish_status(self, state: str, frame: str, cmd: str) -> None:
        status = String()
        status.data = json.dumps(
            {
                "state": state,
                "frame": frame,
                "cmd": cmd,
                "yaml_path": str(self._persistent_yaml_path()),
            },
            separators=(",", ":"),
        )
        self.status_publisher.publish(status)
        if self.debug_mode:
            self.get_logger().debug(f"BUG_POINT:HMI_FRAME state={state} frame={frame}")

    def _publish_command(self, cmd: str, value: int) -> None:
        spec = COMMAND_SPECS[cmd]
        command = String()
        command.data = json.dumps(
            {
                "cmd": cmd,
                "value": value,
                "persistent_parameter": spec.persistent_parameter,
                "restart_required": spec.restart_required,
            },
            separators=(",", ":"),
        )
        self.command_publisher.publish(command)

    def _persistent_yaml_path(self) -> Path:
        if self.persistent_config_path:
            return Path(str(self.persistent_config_path)).expanduser().resolve()
        try:
            from ament_index_python.packages import get_package_share_directory

            share = Path(get_package_share_directory("vision_bringup"))
            return (share / "config" / "vision_system.yaml").resolve()
        except Exception:
            return Path("vision_system.yaml").resolve()

    def _persist_value(self, spec: CommandSpec, value: int) -> None:
        path = self._persistent_yaml_path()
        original_text = path.read_text(encoding="utf-8")
        lines = original_text.splitlines(keepends=True)
        updated = False
        in_hmi_node = False
        in_params = False
        hmi_indent = 0
        params_indent = 0
        pattern = re.compile(rf"^(\s*{re.escape(spec.persistent_parameter)}\s*:\s*).*$")

        for index, line in enumerate(lines):
            stripped = line.strip()
            indent = len(line) - len(line.lstrip(" "))
            if stripped == "hmi_control_node:":
                in_hmi_node = True
                in_params = False
                hmi_indent = indent
                continue
            if in_hmi_node and stripped and indent <= hmi_indent:
                in_hmi_node = False
                in_params = False
            if in_hmi_node and stripped == "ros__parameters:":
                in_params = True
                params_indent = indent
                continue
            if in_params and stripped and indent <= params_indent:
                in_params = False
            if not in_params:
                continue
            match = pattern.match(line.rstrip("\r\n"))
            if match:
                newline = "\r\n" if line.endswith("\r\n") else "\n" if line.endswith("\n") else ""
                lines[index] = f"{match.group(1)}{int(value)}{newline}"
                updated = True
                break

        if not updated:
            raise RuntimeError(f"{spec.persistent_parameter} not found under hmi_control_node")

        new_text = "".join(lines)
        directory = path.parent
        with tempfile.NamedTemporaryFile(
            "w", encoding="utf-8", newline="", dir=directory, delete=False
        ) as temp_file:
            temp_file.write(new_text)
            temp_name = temp_file.name
        os.replace(temp_name, path)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = HmiControlNode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
