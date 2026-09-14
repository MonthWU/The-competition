"""Launch the 26E fixed-camera puzzle vision pipeline."""

from pathlib import Path
import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


SHORT_EXPOSURE_TIME = 78
LONG_EXPOSURE_TIME = 220


def _as_bool(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value not in ("true", "false"):
        raise RuntimeError(f"{name} must be true or false")
    return value == "true"


def _mode_from_hmi_yaml(config_path):
    try:
        import yaml

        data = yaml.safe_load(Path(config_path).read_text(encoding="utf-8")) or {}
        params = data.get("hmi_control_node", {}).get("ros__parameters", {})
        return "debug" if int(params.get("hmi_value_debug", 1)) != 0 else "work"
    except Exception:
        return "debug"


def _exposure_time_from_flash(value):
    try:
        flash_value = int(value)
    except (TypeError, ValueError) as exception:
        raise RuntimeError("hmi_value_flash must be 0 or 1") from exception
    if flash_value == 0:
        return SHORT_EXPOSURE_TIME
    if flash_value == 1:
        return LONG_EXPOSURE_TIME
    raise RuntimeError("hmi_value_flash must be 0 or 1")


def _camera_pipeline_from_hmi_yaml(config_path):
    try:
        import yaml

        data = yaml.safe_load(Path(config_path).read_text(encoding="utf-8")) or {}
        hmi_params = data.get("hmi_control_node", {}).get("ros__parameters", {})
        camera_params = data.get("puzzle_perception_node", {}).get("ros__parameters", {})
        pipeline = str(camera_params.get("camera_pipeline", ""))
        # BUG_POINT:DEFAULT_EXPOSURE_SELECTION - Missing HMI state must use the
        # long/default exposure. Only an explicit flash=0 may select 78.
        exposure_time = _exposure_time_from_flash(hmi_params.get("hmi_value_flash", 1))
    except RuntimeError:
        raise
    except Exception as exception:
        raise RuntimeError("failed to read the HMI exposure setting") from exception

    if not pipeline:
        raise RuntimeError("puzzle_perception_node.camera_pipeline must not be empty")
    updated_pipeline, replacements = re.subn(
        r"(exposure_time_absolute=)\d+",
        rf"\g<1>{exposure_time}",
        pipeline,
        count=1,
    )
    if replacements != 1:
        raise RuntimeError("camera_pipeline must contain one exposure_time_absolute value")
    return updated_pipeline


def _nodes_for_mode(context):
    config_file = PathJoinSubstitution(
        [FindPackageShare("vision_bringup"), "config", "vision_system.yaml"]
    )
    mode = LaunchConfiguration("mode").perform(context).strip().lower()
    config_path = config_file.perform(context)
    if mode == "auto":
        mode = _mode_from_hmi_yaml(config_path)
    if mode not in ("work", "debug"):
        raise RuntimeError("mode must be 'auto', 'work', or 'debug'")

    tuning_file = PathJoinSubstitution(
        [FindPackageShare("vision_bringup"), "config", "competition_tuning.yaml"]
    )
    debug_enabled = mode == "debug"
    web_enabled = _as_bool(context, "web_enabled")
    serial_enabled = _as_bool(context, "serial_enabled")
    hmi_enabled = _as_bool(context, "hmi_enabled")
    common = [config_file, tuning_file, {"debug_mode": debug_enabled}]
    # The persisted flash value selects the initial exposure. Runtime flash commands
    # are handled by puzzle_perception_node and reopen the active camera pipeline.
    camera_pipeline = _camera_pipeline_from_hmi_yaml(config_path)
    # Node-level debug_mode enables bounded diagnostics without turning on noisy rcl internals.
    debug_arguments = []

    nodes = [
        Node(
            package="puzzle_perception_node",
            executable="puzzle_perception_node",
            name="puzzle_perception_node",
            output="screen",
            parameters=common + [
                {
                    "publish_debug_image": debug_enabled,
                    "camera_pipeline": camera_pipeline,
                }
            ],
            arguments=debug_arguments,
        ),
        Node(
            package="puzzle_solver_node",
            executable="puzzle_solver_node",
            name="puzzle_solver_node",
            output="screen",
            parameters=common,
            arguments=debug_arguments,
        ),
        Node(
            package="puzzle_coordinator_node",
            executable="puzzle_coordinator_node",
            name="puzzle_coordinator_node",
            output="screen",
            parameters=common,
            arguments=debug_arguments,
        ),
    ]

    if serial_enabled:
        nodes.append(
            Node(
                package="serial_bridge_node",
                executable="serial_bridge_node",
                name="serial_bridge_node",
                output="screen",
                parameters=common + [{"hmi_enabled": hmi_enabled}],
                arguments=debug_arguments,
            )
        )
        if hmi_enabled:
            nodes.append(
                Node(
                    package="hmi_control_node",
                    executable="hmi_control_node",
                    name="hmi_control_node",
                    output="screen",
                    parameters=common,
                    arguments=debug_arguments,
                )
            )
    elif hmi_enabled:
        nodes.append(
            Node(
                package="hmi_control_node",
                executable="hmi_control_node",
                name="hmi_control_node",
                output="screen",
                parameters=common,
                arguments=debug_arguments,
            )
        )

    if debug_enabled and web_enabled:
        nodes.append(
            Node(
                package="web_tuner_node",
                executable="web_tuner_node",
                name="web_tuner_node",
                output="screen",
                parameters=[config_file, tuning_file, {"web_enabled": True}],
                arguments=debug_arguments,
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="auto",
                description="auto reads hmi_value_debug from vision_system.yaml; explicit work/debug overrides it.",
            ),
            DeclareLaunchArgument(
                "web_enabled",
                default_value="true",
                description="Start the Flask debug UI in debug mode; requires python3-flask.",
            ),
            DeclareLaunchArgument(
                "serial_enabled",
                default_value="true",
                description="Start the hardware serial bridge; disable for vision-only runs.",
            ),
            DeclareLaunchArgument(
                "hmi_enabled",
                default_value="true",
                description="Start the HMI protocol node and enable the HMI UART in serial bridge.",
            ),
            OpaqueFunction(function=_nodes_for_mode),
        ]
    )
