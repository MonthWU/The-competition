"""Scan the school task QR and send it to the MCU without an object camera."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


CAM_QRC = "/dev/v4l/by-id/usb-KINGSEN_KS1A293-video-index0"


def generate_launch_description():
    if not os.path.exists(CAM_QRC):
        raise FileNotFoundError(f"[run_qr_only] CAMERA_NOT_FOUND: {CAM_QRC}")
    qr_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("qrc_skandier"),
                         "launch/qrc_skandier.launch.py")
        ),
        launch_arguments={"cap_qrc": CAM_QRC}.items(),
    )
    serial_node = Node(
        package="obj_detect",
        executable="obj_serial",
        name="obj_serial",
        parameters=[{"qr_only": True, "serial_device": LaunchConfiguration("serial_device")}],
        output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument("serial_device", default_value="/dev/ttyS1"),
        qr_launch,
        serial_node,
    ])
