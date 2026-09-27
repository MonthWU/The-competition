"""prescan.launch —— 全局扫描子 launch（2026-09-23）。

启动 DCXIN（by-id 固定）+ dnn_node_example（task=task_obj_obstacle.json，1 类 ball）
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

# 全局扫描相机 by-id（2026-09-23 用户最终确认后二次修正）：
#   实际接线映射：检测 = LRCP AR0234（拍物块区），全局扫描 = DCXIN（车顶）
SCAN_VIDEO_DEVICE = "/dev/v4l/by-id/usb-DCXIN_DCXIN_Camera_01.00.000-video-index0"
SCAN_WIDTH = 1280
# 2026-09-27 修正（关键）：
#   640x640  → DCXIN 不支持（该相机 MJPG 仅 640x360 / 1280x720 / 1920x1080）
#   640x360  → 虽然相机支持，但会让 dnn **段错误**：640x360 → 640x640 是纵向
#              放大 1.78x，触发 hobot_cv VPS 问题（实测第一帧后 SIGSEGV）
#   1280x720 → 缩小到 640x640，实测稳定不崩 ✅ ← 采用
SCAN_HEIGHT = 720
SCAN_FPS = 30
DNN_TASK_JSON = "/root/dev_ws/appli/framework/dnn/task_obj_obstacle.json"
SETUP_SH = "/root/dev_ws/appli/framework/setup_dcxin.sh"   # 非 ROS 场景的手动备用工具


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
            # 注意：节点参数名不带 usb_ 前缀（看 /opt/tros/humble/share/hobot_usb_cam/launch/hobot_usb_cam.launch.py）
            # launch argument 名是 usb_*（带前缀），节点参数名是 video_device / image_* / framerate
            # 之前我误用了 launch argument 名作为节点参数，导致参数没传进去 → 默认 /dev/video8 → fallback video0
            # 2026-09-23 实机验证：用 TextSubstitution + 正确参数名 video_device 单点跑通
            {"image_width": LaunchConfiguration("scan_width")},
            {"image_height": LaunchConfiguration("scan_height")},
            {"framerate": LaunchConfiguration("scan_fps")},
            {"video_device": LaunchConfiguration("scan_video_device")},
            {"pixel_format": "mjpeg"},
            {"io_method": "mmap"},
            # DCXIN 亮度/增益修正（2026-09-25，实测有效）
            # 背景：该机固件只支持 auto_exposure=1/3，没有真正的 Auto(0)；默认的
            #       3（光圈优先）在 UVC 上无光圈可调、曝光时间控制被禁用(inactive)，
            #       画面亮度实际只由 brightness/gain 决定，出厂值 50/0 明显偏暗
            #       （实测画面均值 68.6，中央区仅 28.8）。
            # 关键：必须通过**节点参数**设置 —— 节点启动时会写入自己的 brightness
            #       默认值(50)，会覆盖 launch 之前用 v4l2-ctl 做的预设（已实测被覆盖）。
            #       而 gain 默认 -1 表示"不修改"。改为 128/48 后画面均值 138.9。
            {"brightness": 128},
            {"gain": 48},
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
    # Web 预览：订阅 LRCP 的 /image 推流到板端 :8000（nginx via websocket 节点）
    prescan_web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("websocket"),
                "launch/websocket.launch.py",
            )
        ),
        launch_arguments={
            "websocket_image_topic": "/image",
            "websocket_image_type": "mjpeg",
            "websocket_smart_topic": "hobot_dnn_detection",
            "websocket_only_show_image": "True",
        }.items(),
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
            prescan_web_node,
        ]
    )
# 注：prescan_dnn_node（订阅 DNN 输出的解析节点）由 mission_dispatcher.scan_angle()
# 在 Python 进程内直接 rclpy.init + PrescanDnnNode() 启动，**不通过 launch 拉起**——
# 因为 framework/ 不是一个 ROS 包（没有 package.xml / setup.py），无法 Node() 引用。