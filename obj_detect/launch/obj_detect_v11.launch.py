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


def get_max_fps(device):
    """使用 v4l2-ctl 查询摄像头支持的最大帧率"""
    try:
        # 调用 v4l2-ctl 列出支持的格式和帧率
        result = subprocess.run(
            ["v4l2-ctl", "--list-formats-ext", "-d", f"/dev/{device}"],
            capture_output=True,
            text=True,
            check=True,
        )
        output = result.stdout

        # 使用正则表达式匹配最大帧率
        fps_matches = re.findall(r"Interval: .*\((\d+\.\d+) fps\)", output)
        if not fps_matches:
            print(f"未能找到 {device} 的帧率信息")
            return None

        # 获取最高帧率
        max_fps = max(float(fps) for fps in fps_matches)
        return max_fps

    except subprocess.CalledProcessError:
        print(f"无法访问设备 {device}")
        return None


def find_camera(dev_nodes=["video0", "video2"]):
    """
    根据摄像头的最大帧率，返回两个摄像头设备节点
    返回顺序：帧率高的在前，作为二维码摄像头（因为他是黑白的）
    """
    # v11 修复：过滤不可访问设备（get_max_fps 返回 None），单相机时两路同用一路（分时复用）
    cands = [(d, get_max_fps(d)) for d in dev_nodes]
    cands = [(d, f) for d, f in cands if f is not None]
    if not cands:
        return "video0", "video0"
    cands.sort(key=lambda x: x[1], reverse=True)
    devs = [d for d, _ in cands]
    if len(devs) >= 2:
        return devs[0], devs[1]
    return devs[0], devs[0]


def generate_launch_description():
    # 三路相机任务映射（2026-09-10 初版，2026-09-23 二次修正已对调，见下）：
    #   扫码 = KS1A293（max 240fps，唯一支持240fps那台）→ by-id 固定
    #   检测 = LRCP AR0234 → by-id 固定（2026-09-23 修正）
    #   全局扫描 = DCXIN（不在本 launch 中使用，2026-09-23 修正）
    # by-id 路径在 /dev/v4l/by-id/ 下，不受 /dev/video* 编号漂移影响。
    # 三路相机任务映射（2026-09-23 用户最终确认，2026-09-23 二次修正）：
    #   扫码 = KS1A293（max 240fps，唯一支持 240fps 那台）→ by-id 固定
    #   检测 = LRCP AR0234（实际装在检测位 → 拍到物块区画面，2026-09-23 验证）
    #   全局扫描 = DCXIN（实际装在车顶，2026-09-23 验证）
    # by-id 路径在 /dev/v4l/by-id/ 下，不受 /dev/video* 编号漂移影响。
    cap_qrc_devnode = "/dev/v4l/by-id/usb-KINGSEN_KS1A293-video-index0"
    cap_objdet_devnode = "/dev/v4l/by-id/usb-LRCP_AR0234_LRCP_AR0234_01.00.00-video-index0"
    print(
        f"cap_qrc_devnode: {cap_qrc_devnode}, cap_objdet_devnode: {cap_objdet_devnode}"
    )

    cap_objdet_dev_arg = DeclareLaunchArgument(
        "cap_objdet",
        default_value=cap_objdet_devnode,
        description="object detection camera device",
    )

    cap_qrc_dev_arg = DeclareLaunchArgument(
        # 说明：本 launch 当前**不含**扫码链路（只有 obj_camd / codec / websocket /
        # dnn / serial / dumper），扫码由 launch/run_all.launch.py 独立拉起。
        # 此参数为预留接口，默认值已 by-id 固化，避免将来接入时又退回 /dev/video* 猜测。
        "cap_qrc",
        default_value=cap_qrc_devnode,
        description="qrcode camera device (预留，本 launch 未接入)",
    )

    config_file_launch_arg = DeclareLaunchArgument(
        "dnn_example_config_file",
        default_value=TextSubstitution(text="/root/dev_ws/appli/dnn/task_obj_v11.json"),
    )
    dump_render_launch_arg = DeclareLaunchArgument(
        "dnn_example_dump_render_img", default_value=TextSubstitution(text="0")
    )
    image_width_launch_arg = DeclareLaunchArgument(
        "dnn_example_image_width", default_value=TextSubstitution(text="960")
    )
    image_height_launch_arg = DeclareLaunchArgument(
        "dnn_example_image_height", default_value=TextSubstitution(text="544")
    )
    msg_pub_topic_name_launch_arg = DeclareLaunchArgument(
        "dnn_example_msg_pub_topic_name",
        default_value=TextSubstitution(text="hobot_dnn_detection"),
    )

    run_mode_arg = DeclareLaunchArgument(
        "run_mode",
        default_value="take",
        description="run mode: take(video) or non-take(video)",
    )

    # 目标检测的 usb camera
    # cap_objdet_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory("hobot_usb_cam"),
    #             "launch/hobot_usb_cam.launch.py",
    #         )
    #     ),
    #     launch_arguments={
    #         "usb_image_width": "640",
    #         "usb_image_height": "480",
    #         'usb_framerate': '120',
    #         "usb_video_device": LaunchConfiguration("cap_objdet"),
    #     }.items(),
    # )

    obj_camd_node = Node(
        package="obj_detect",
        executable="obj_camd",
        name="obj_camd",
        parameters=[
            {"usb_video_device": LaunchConfiguration("cap_objdet")},
            {"usb_image_width": LaunchConfiguration("dnn_example_image_width")},
            {"usb_image_height": LaunchConfiguration("dnn_example_image_height")},
            {"usb_framerate": 120},
            {
                "launch_file_path": os.path.join(
                    get_package_share_directory("obj_detect"),
                    "launch/obj_cam.launch.py",
                )
            },
        ],
    )

    # # 目标检测的 jpeg 编码+发布 # mipi camera manul sayonghada.
    # objdet_jpeg_codec_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory("hobot_codec"),
    #             "launch/hobot_codec_encode.launch.py",
    #         )
    #     ),
    #     launch_arguments={
    #         "codec_in_mode": "shared_mem",
    #         "codec_out_mode": "ros",
    #         "codec_sub_topic": "/hbmem_img",
    #         "codec_pub_topic": "/image",
    #     }.items(),
    # )

    # 目标检测的 nv12 解码+发布
    objdet_nv12_codec_node = IncludeLaunchDescription(
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

    # Web 展示 目标检测结果
    objdet_web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("websocket"), "launch/websocket.launch.py"
            )
        ),
        launch_arguments={
            "websocket_image_topic": "/image",
            "websocket_image_type": "mjpeg",
            "websocket_smart_topic": LaunchConfiguration(
                "dnn_example_msg_pub_topic_name"
            ),
        }.items(),
    )

    # 目标检测算法节点
    dnn_node_example_node = Node(
        package="dnn_node_example",
        executable="example",
        output="screen",
        parameters=[
            {"config_file": LaunchConfiguration("dnn_example_config_file")},
            {"dump_render_img": LaunchConfiguration("dnn_example_dump_render_img")},
            {"feed_type": 1},
            {"is_shared_mem_sub": 1},
            {
                "msg_pub_topic_name": LaunchConfiguration(
                    "dnn_example_msg_pub_topic_name"
                )
            },
        ],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    shared_mem_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("hobot_shm"), "launch/hobot_shm.launch.py"
            )
        )
    )

    # 串口发送（v11 回归：复用 obj_serial，其解析已支持 9 类新模型）
    obj_serial_node = Node(
        package="obj_detect",
        executable="obj_serial",
        name="obj_serial",
    )

    # 视频保存
    video_take_node = Node(
        package="obj_detect",
        executable="obj_video_dumper",
        name="obj_video_dumper",
        parameters=[
            {"video_dir": "/root/dev_ws/appli/_tmp_videos/"},
            {"video_fps": 20},
            {"video_width": 640},
            {"video_height": 480},
            {"video_fourcc": "MJPG"},
            {"video_update_time": 10},
        ],
    )

    # print("run_mode: ", LaunchConfiguration("run_mode"))
    # if LaunchConfiguration("run_mode").perform(None) == "take":
    if True:
        # print("run_mode: take")
        return LaunchDescription(
            [
                cap_objdet_dev_arg,
                cap_qrc_dev_arg,
                config_file_launch_arg,
                dump_render_launch_arg,
                image_width_launch_arg,
                image_height_launch_arg,
                msg_pub_topic_name_launch_arg,
                run_mode_arg,
                # cap_objdet_node, # 当前不开启目标检测的 usb camera, 开一个守护节点，等待二维码相机释放
                obj_camd_node,
                # objdet_jpeg_codec_node,
                objdet_nv12_codec_node,
                objdet_web_node,
                dnn_node_example_node,
                shared_mem_node,
                obj_serial_node,
                video_take_node,
            ]
        )
    else:
        return LaunchDescription(
            [
                cap_objdet_dev_arg,
                cap_qrc_dev_arg,
                config_file_launch_arg,
                dump_render_launch_arg,
                image_width_launch_arg,
                image_height_launch_arg,
                msg_pub_topic_name_launch_arg,
                run_mode_arg,
                # cap_objdet_node, # 当前不开启目标检测的 usb camera, 开一个守护节点，等待二维码相机释放
                obj_camd_node,
                # objdet_jpeg_codec_node,
                objdet_nv12_codec_node,
                objdet_web_node,
                dnn_node_example_node,
                shared_mem_node,
                obj_serial_node,
            ]
        )
