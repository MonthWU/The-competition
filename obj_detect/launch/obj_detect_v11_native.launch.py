"""obj_detect_v11_native.launch —— v11 物块识别链路（板端原生推理版，2026-09-25）。

与 obj_detect_v11.launch.py 的唯一差异：用 **obj_dnn**（pyeasy_dnn + 自写 DFL 解码）
替换 dnn_node_example。原因：dnn/yolo11_x5.bin 输出为 NCHW + float32，
与 TROS parser_yolov8 假设的 NHWC + int32 不兼容，dnn_node_example 会段错误（exit -11）。

由此带来的链路简化：
  - 不再需要 hobot_shm（obj_dnn 不走共享内存）
  - 不再需要 hobot_codec（obj_dnn 直接订阅 /image 的 mjpeg，自己解码+缩放）
  - obj_dnn 对外发布与 dnn_node_example 完全一致的 ai_msgs/PerceptionTargets，
    因此 obj_serial（串口下发）与 websocket（Web 画框）无需任何改动

链路：
  obj_camd（守护，收 /kill_qrc 后用 cap_objdet 拉起 obj_cam.launch.py）
    → hobot_usb_cam（LRCP AR0234 @640x480 MJPEG → /image）
    → obj_dnn（/image → /hobot_dnn_detection）
    → obj_serial（串口 ttyS1 下发） + websocket（板端 :8000 画框预览）
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# 三路相机 by-id 固化（2026-09-23 二次修正后），与 run_all.launch.py 保持一致
CAM_QRC = "/dev/v4l/by-id/usb-KINGSEN_KS1A293-video-index0"                      # 扫码
CAM_OBJDET = "/dev/v4l/by-id/usb-LRCP_AR0234_LRCP_AR0234_01.00.00-video-index0"  # 检测

MODEL_FILE = "/root/dev_ws/appli/dnn/yolo11_x5.bin"
CLASSES_FILE = "/root/dev_ws/appli/dnn/classes.names"


def _check_cam(path: str, role: str) -> str:
    """校验 by-id 路径存在；缺失时大声告警（不回退到猜测模式）。"""
    if not os.path.exists(path):
        print(
            f"[obj_detect_v11_native] 错误：{role}相机 {path} 不存在！请检查 USB 接线（by-id 路径）。"
        )
    return path


def generate_launch_description():
    cap_objdet_devnode = _check_cam(CAM_OBJDET, "检测")
    cap_qrc_devnode = _check_cam(CAM_QRC, "扫码")

    cap_objdet_dev_arg = DeclareLaunchArgument(
        "cap_objdet", default_value=cap_objdet_devnode,
        description="object detection camera device (by-id)",
    )
    cap_qrc_dev_arg = DeclareLaunchArgument(
        "cap_qrc", default_value=cap_qrc_devnode,
        description="qrcode camera device (by-id, 预留：扫码由 run_all 拉起)",
    )
    model_arg = DeclareLaunchArgument(
        "model_file", default_value=TextSubstitution(text=MODEL_FILE),
        description="BPU model (.bin)",
    )
    names_arg = DeclareLaunchArgument(
        "cls_names_list", default_value=TextSubstitution(text=CLASSES_FILE),
        description="class names file",
    )
    score_arg = DeclareLaunchArgument(
        "score_threshold", default_value=TextSubstitution(text="0.25"))
    nms_arg = DeclareLaunchArgument(
        "nms_threshold", default_value=TextSubstitution(text="0.45"))
    msg_topic_arg = DeclareLaunchArgument(
        "msg_pub_topic_name", default_value=TextSubstitution(text="hobot_dnn_detection"))

    # 守护节点：收到 /kill_qrc 后用 cap_objdet 拉起 obj_cam.launch.py（USB 相机）
    # 注意参数名不带 usb_ 前缀（见 obj_cam.launch.py / hobot_usb_cam 节点定义）
    obj_camd_node = Node(
        package="obj_detect",
        executable="obj_camd",
        name="obj_camd",
        parameters=[
            {"usb_video_device": LaunchConfiguration("cap_objdet")},
            {"usb_image_width": 640},
            {"usb_image_height": 480},
            {"usb_framerate": 120},
            {
                "launch_file_path": os.path.join(
                    get_package_share_directory("obj_detect"),
                    "launch/obj_cam.launch.py",
                )
            },
        ],
        output="screen",
    )

    # 推理节点：pyeasy_dnn + 自写 DFL 解码，发布 PerceptionTargets
    obj_dnn_node = Node(
        package="obj_detect",
        executable="obj_dnn",
        name="obj_dnn",
        parameters=[
            {"model_file": LaunchConfiguration("model_file")},
            {"cls_names_list": LaunchConfiguration("cls_names_list")},
            {"image_topic": "/image"},
            {"msg_pub_topic_name": LaunchConfiguration("msg_pub_topic_name")},
            {"score_threshold": LaunchConfiguration("score_threshold")},
            {"nms_threshold": LaunchConfiguration("nms_threshold")},
        ],
        output="screen",
    )

    # 串口下发（复用 obj_serial，其解析已支持 9 类新模型）
    obj_serial_node = Node(
        package="obj_detect",
        executable="obj_serial",
        name="obj_serial",
        output="screen",
    )

    # Web 展示：订阅 /image(mjpeg) + /hobot_dnn_detection，浏览器 :8000 叠加检测框
    objdet_web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("websocket"), "launch/websocket.launch.py"
            )
        ),
        launch_arguments={
            "websocket_image_topic": "/image",
            "websocket_image_type": "mjpeg",
            "websocket_smart_topic": LaunchConfiguration("msg_pub_topic_name"),
        }.items(),
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
        output="screen",
    )

    return LaunchDescription(
        [
            cap_objdet_dev_arg,
            cap_qrc_dev_arg,
            model_arg,
            names_arg,
            score_arg,
            nms_arg,
            msg_topic_arg,
            obj_camd_node,
            obj_dnn_node,
            objdet_web_node,
            obj_serial_node,
            video_take_node,
        ]
    )
