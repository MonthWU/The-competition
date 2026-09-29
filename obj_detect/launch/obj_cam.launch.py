import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_prefix

import subprocess
import re


def generate_launch_description():
    cap_objdet_dev_arg = DeclareLaunchArgument(
        "cap_objdet",
        default_value="/dev/video0",
        description="object detection camera device",
    )
    usb_image_width = DeclareLaunchArgument(
        "usb_image_width",
        default_value="640",
        description="usb camera image width",
    )
    usb_image_height = DeclareLaunchArgument(
        "usb_image_height",
        default_value="480",
        description="usb camera image height",
    )
    usb_framerate = DeclareLaunchArgument(
        "usb_framerate",
        default_value="90",
        description="usb camera framerate",
    )

    # 目标检测的 usb camera
    cap_objdet_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("hobot_usb_cam"),
                "launch/hobot_usb_cam.launch.py",
            )
        ),
        launch_arguments={
            "usb_image_width": LaunchConfiguration("usb_image_width"),
            "usb_image_height": LaunchConfiguration("usb_image_height"),
            "usb_framerate": LaunchConfiguration("usb_framerate"),
            "usb_video_device": LaunchConfiguration("cap_objdet"),
        }.items(),
    )

    return LaunchDescription(
        [
            cap_objdet_dev_arg,
            usb_image_width,
            usb_image_height,
            usb_framerate,
            
            cap_objdet_node,
        ]
    )
