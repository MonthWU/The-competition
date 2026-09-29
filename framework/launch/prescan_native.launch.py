"""prescan_native.launch.py —— 全局扫描链路（obs_dnn 板端原生推理版，2026-09-28）。

与 prescan.launch.py 的差异
--------------------------
本版本**不再使用 dnn_node_example** —— 障碍物模型 yolo11_x5_obstacle.bin 的输出
tensor 与 TROS parser_yolov8 假设不兼容（cls 标 NCHW 但 shape 是 (1,H,W,C)），
会 SIGSEGV。改用 obs_dnn.py（pyeasy_dnn + 自写 DFL 解码，按 box 通道数 64 自适应
HWC/CHW 解码，与 obj_dnn.py 同一份骨架）。

由此带来的链路简化（与 prescan.launch.py 一致）：
  - 不再需要 hobot_codec / hobot_shm（obs_dnn 直订 /image 的 mjpeg）
  - 对外仍发布 ai_msgs/PerceptionTargets 到 /hobot_dnn_detection
    → prescan_dnn_node / obstacle_detector / mission_dispatcher.scan_angle() 全部
      无需改动
  - obs_dnn 不是 ROS 包（无 package.xml），与 prescan.launch.py 同样走
    ExecuteProcess(python3 obs_dnn.py) 启动

链路：
  hobot_usb_cam（DCXIN @1280x720 MJPEG, 带亮度修正 → /image）
    → obs_dnn（/image → /hobot_dnn_detection）
    → prescan_dnn_node（mission_dispatcher 用）+ websocket（:8000 预览）

注意：prescan.launch.py 历史上还包含 codec_decode + shm_node，但那是 dnn_node_example
时代的残留；obs_dnn 直订 mjpeg /image，根本不读 NV12 共享内存，那两个 include
已在本次修复中**删除**，本文件不再包含它们。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

# DCXIN by-id 固定（全局扫描相机）
SCAN_DEVICE = "/dev/v4l/by-id/usb-DCXIN_DCXIN_Camera_01.00.000-video-index0"
SCAN_WIDTH = 1280     # 640x360 会让 dnn 段错误（纵向放大 1.78x）→ 用 1280x720
SCAN_HEIGHT = 720
SCAN_FPS = 30

OBSTACLE_MODEL = "/root/dev_ws/appli/framework/dnn/yolo11_x5_obstacle.bin"
OBSTACLE_NAMES = "/root/dev_ws/appli/framework/dnn/classes_obstacle.names"
OBS_DNN_SCRIPT = "/root/dev_ws/appli/framework/obs_dnn.py"


def generate_launch_description():
    if not os.path.exists(SCAN_DEVICE):
        print("[prescan_native] 警告：未找到 DCXIN 设备 %s" % SCAN_DEVICE)
    if not os.path.exists(OBS_DNN_SCRIPT):
        print("[prescan_native] 错误：obs_dnn.py 不存在 %s" % OBS_DNN_SCRIPT)
    if not os.path.exists(OBSTACLE_MODEL):
        print("[prescan_native] 错误：障碍模型不存在 %s" % OBSTACLE_MODEL)

    cam_arg = DeclareLaunchArgument(
        "scan_video_device", default_value=TextSubstitution(text=SCAN_DEVICE))
    width_arg = DeclareLaunchArgument(
        "scan_width", default_value=TextSubstitution(text=str(SCAN_WIDTH)))
    height_arg = DeclareLaunchArgument(
        "scan_height", default_value=TextSubstitution(text=str(SCAN_HEIGHT)))
    fps_arg = DeclareLaunchArgument(
        "scan_fps", default_value=TextSubstitution(text=str(SCAN_FPS)))

    cam_node = Node(
        package="hobot_usb_cam",
        executable="hobot_usb_cam",
        name="prescan_usb_cam",
        parameters=[
            {"video_device": LaunchConfiguration("scan_video_device")},
            {"image_width": LaunchConfiguration("scan_width")},
            {"image_height": LaunchConfiguration("scan_height")},
            {"framerate": LaunchConfiguration("scan_fps")},
            {"pixel_format": "mjpeg"},
            {"io_method": "mmap"},
            # DCXIN 亮度修正（该机无真正 Auto 曝光；出厂 50/0 偏暗）
            {"brightness": 128},
            {"gain": 48},
        ],
        output="screen",
    )

    # 障碍识别：obs_dnn（pyeasy_dnn + 自写 DFL，绕开 TROS parser 段错误）
    obs_dnn_node = ExecuteProcess(
        cmd=[
            "python3", OBS_DNN_SCRIPT,
            "--ros-args",
            "-p", ("model_file:=" + OBSTACLE_MODEL),
            "-p", ("cls_names_list:=" + OBSTACLE_NAMES),
            "-p", "image_topic:=/image",
            "-p", "msg_pub_topic_name:=hobot_dnn_detection",
            "-p", "score_threshold:=0.4",
            "-p", "nms_threshold:=0.5",
        ],
        output="screen",
    )

    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("websocket"),
                         "launch/websocket.launch.py")
        ),
        launch_arguments={
            "websocket_image_topic": "/image",
            "websocket_image_type": "mjpeg",
            "websocket_smart_topic": "hobot_dnn_detection",
        }.items(),
    )

    return LaunchDescription([cam_arg, width_arg, height_arg, fps_arg,
                              cam_node, obs_dnn_node, web_node])