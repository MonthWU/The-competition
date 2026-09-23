"""prescan.launch —— 全局扫描子 launch（2026-09-23）。

启动 LRCP AR0234（by-id 固定）+ dnn_node_example（task=task_obj_obstacle.json，1 类 ball）
+ prescan_dnn_node，供 mission_dispatcher.scan_angle() subprocess 调用。

分辨率：640×640 MJPG（与 yolo11 障碍模型输入对齐）。
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# 全局扫描相机 by-id（2026-09-23 固化）
SCAN_VIDEO_DEVICE = "/dev/v4l/by-id/usb-LRCP_AR0234_LRCP_AR0234_01.00.00-video-index0"
SCAN_WIDTH = 640
SCAN_HEIGHT = 640
SCAN_FPS = 30
DNN_TASK_JSON = "/root/dev_ws/appli/framework/dnn/task_obj_obstacle.json"


def generate_launch_description():
    width_arg = DeclareLaunchArgument(
        "scan_width", default_value=TextSubstitution(text=str(SCAN_WIDTH))
    )
    height_arg = DeclareLaunchArgument(
        "scan_height", default_value=TextSubstitution(text=str(SCAN_HEIGHT))
    )
    fps_arg = DeclareLaunchArgument(
        "scan_fps", default_value=TextSubstitution(text=str(SCAN_FPS))
    )
    device_arg = DeclareLaunchArgument(
        "scan_video_device", default_value=TextSubstitution(text=SCAN_VIDEO_DEVICE)
    )
    config_arg = DeclareLaunchArgument(
        "dnn_task_json", default_value=TextSubstitution(text=DNN_TASK_JSON)
    )

    usb_cam_node = Node(
        package="hobot_usb_cam",
        executable="hobot_usb_cam",
        name="prescan_usb_cam",
        parameters=[
            {"usb_image_width": LaunchConfiguration("scan_width")},
            {"usb_image_height": LaunchConfiguration("scan_height")},
            {"usb_framerate": LaunchConfiguration("scan_fps")},
            {"usb_video_device": LaunchConfiguration("scan_video_device")},
            {"usb_pixel_format": "mjpeg"},
            {"usb_io_method": "mmap"},
        ],
        output="screen",
    )

    # 解码 NV12 → shared_mem（jpeg 输入 → NV12 输出），与 v11 launch 一致
    codec_decode_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("hobot_codec"),
                "launch/hobot_codec_decode.launch.py",
            )
        ),
        launch_arguments={
            "codec_in_mode": "ros",
            "codec_out_mode": "shared_mem",
            "codec_sub_topic": "/image",
            "codec_pub_topic": "/hbmem_img",
        }.items(),
    )

    # hobot_shm 是环境配置包（设 FASTRTPS QoS 让共享内存零拷贝），不是 Node
    shm_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("hobot_shm"),
                "launch/hobot_shm.launch.py",
            )
        )
    )

    dnn_node = Node(
        package="dnn_node_example",
        executable="example",
        name="prescan_dnn_example",
        parameters=[
            {"config_file": LaunchConfiguration("dnn_task_json")},
            {"dump_render_img": 0},
            {"feed_type": 1},
            {"is_shared_mem_sub": 1},
            {"msg_pub_topic_name": "hobot_dnn_detection"},
        ],
        arguments=["--ros-args", "--log-level", "warn"],
        output="screen",
    )

    return LaunchDescription(
        [
            width_arg,
            height_arg,
            fps_arg,
            device_arg,
            config_arg,
            usb_cam_node,
            codec_decode_node,
            shm_node,
            dnn_node,
        ]
    )
# 注：prescan_dnn_node（订阅 DNN 输出的解析节点）由 mission_dispatcher.scan_angle()
# 在 Python 进程内直接 rclpy.init + PrescanDnnNode() 启动，**不通过 launch 拉起**——
# 因为 framework/ 不是一个 ROS 包（没有 package.xml / setup.py），无法 Node() 引用。