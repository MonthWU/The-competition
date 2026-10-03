"""obj_detect_v11.launch —— v11 物块识别链路（统一入口，2026-09-29）。

本 launch 是 v11 物块识别链路的**唯一对外入口**。它在两条可替换的推理实现之间切换，
两条链路对外暴露的接口完全对齐（同样的 topic、同样的 launch 参数、同样的字段语义），
因此下游消费者（obj_serial / websocket / 录像）无需任何改动，未来删除其中一条
链路时也不会牵连其他文件。

两条推理实现
-----------
1) `dnn_engine:=workaround`（**默认**）：使用 NCHW 物块模型（dnn/yolo11_x5.bin）
   + 自写 `obj_dnn` 节点（pyeasy_dnn + 自写 DFL 解码）。原因：现役 NCHW 模型与
   TROS parser_yolov8 假设的 NHWC + int32 不兼容，会段错误。
2) `dnn_engine:=native`：使用 NHWC 物块模型（dnn/yolo11_x5_nhwc.bin）
   + TROS 自带 `dnn_node_example`（dnn_Parser=yolov8 直接吃）。无需 obj_dnn 绕行层。

两条链路统一参数（命名完全一致，调用方零感知）
-----------------------------------------------
- cap_objdet             : 检测相机 by-id（默认 LRCP AR0234）
- dnn_example_image_width / dnn_example_image_height : 拉流分辨率（两链路同名）
- dnn_task_json          : task JSON 路径（仅原生链使用）
- model_file             : 物块模型 .bin（仅绕行链使用；与 task_obj_v11*.json 中
                           的 model_file 字段保持同名，调试时便于交叉对照）
- cls_names_list         : 类别名文件（绕行链使用，与 task JSON 内字段同名）
- msg_pub_topic_name     : DNN 输出话题（默认 hobot_dnn_detection，
                           PerceptionTargets 消息）
- score_threshold / nms_threshold : 评分 / NMS 阈值
                            （绕行链直接透传给 obj_dnn；原生链通过 task JSON 提供）
- min_target_area_px     : 原图检测框最小面积（默认 2000 像素²，两链路共用）

链路组成（取 dnn_engine:=workaround 时）
----------------------------------------
  obj_camd (守护 /kill_qrc → 拉起 obj_cam.launch.py)
    → hobot_usb_cam (LRCP AR0234 @640x480 MJPEG → /image)
    → obj_dnn (板端原生推理 /image → /hobot_dnn_detection_raw)
    → obj_target_area_filter (面积过滤 → /hobot_dnn_detection)
    → obj_serial (串口 ttyS1 下发) + websocket (板端 :8000 画框) + obj_video_dumper

链路组成（取 dnn_engine:=native 时）
------------------------------------
  obj_camd (同)
    → hobot_usb_cam (同 → /image)
    → hobot_codec_decode (/image → 共享内存 /hbmem_img)
    → dnn_node_example (/hbmem_img → /hobot_dnn_detection_raw，喂 task_obj_v11_nhwc.json)
    → obj_target_area_filter (面积过滤 → /hobot_dnn_detection)
    → obj_serial + websocket + obj_video_dumper（同）
    注：原生链需要 hobot_shm；绕行链不需要。

切换示例
--------
  ros2 launch obj_detect obj_detect_v11.launch.py                                   # 默认 workaround
  ros2 launch obj_detect obj_detect_v11.launch.py dnn_engine:=native               # 原生 NHWC
  ros2 launch obj_detect obj_detect_v11.launch.py dnn_engine:=native \
      dnn_task_json:=/root/dev_ws/appli/dnn/task_obj_v11_nhwc.json                 # 自定义 task JSON

未来删除其中一条链的步骤（保证另一条独立可用）
---------------------------------------------
- 删 workaround：删除 obj_dnn.py + setup.py 中 `obj_dnn = obj_detect.obj_dnn:main`
  条目 + 在本 launch 中删除 obj_dnn_node 分支；把 run_all.launch.py 的默认改为 native。
- 删 native：删除 dnn_node_example_node / shared_mem_node / codec_decode 分支；
  保留 obj_dnn 链路。
两边的下游接口（/hobot_dnn_detection）完全一致，删除任意一条都不需要修改 obj_serial。
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# by-id 固定（与 launch/run_all.launch.py / framework/launch/prescan.launch.py 保持一致）
CAM_OBJDET = "/dev/v4l/by-id/usb-LRCP_AR0234_LRCP_AR0234_01.00.00-video-index0"

# 两链路的默认资产（同名参数，便于对比 / 删除时切换）
NCHW_MODEL_FILE     = "/root/dev_ws/appli/dnn/yolo11_x5.bin"            # workaround 链使用
NCHW_TASK_JSON      = "/root/dev_ws/appli/dnn/task_obj_v11.json"        # 现役 NCHW（dnn_node_example 也会读到）
NHWC_MODEL_FILE     = "/root/dev_ws/appli/dnn/yolo11_x5_nhwc.bin"       # native 链使用
NHWC_TASK_JSON      = "/root/dev_ws/appli/dnn/task_obj_v11_nhwc.json"   # native 链 dnn_node_example 读取
CLASSES_FILE        = "/root/dev_ws/appli/dnn/classes.names"


def _check_cam(path: str, role: str) -> str:
    """Require the configured camera instead of allowing a silent fallback."""
    if not os.path.exists(path):
        raise FileNotFoundError(f"[obj_detect_v11] CAMERA_NOT_FOUND: {role}: {path}")
    return path


def generate_launch_description():
    cap_objdet_devnode = _check_cam(CAM_OBJDET, "检测")

    cap_objdet_dev_arg = DeclareLaunchArgument(
        "cap_objdet", default_value=cap_objdet_devnode,
        description="object detection camera device (by-id)",
    )
    cap_qrc_dev_arg = DeclareLaunchArgument(
        "cap_qrc",
        default_value="/dev/v4l/by-id/usb-KINGSEN_KS1A293-video-index0",
        description="qrcode camera device (by-id, 预留：扫码由 run_all 拉起)",
    )
    serial_device_arg = DeclareLaunchArgument(
        "serial_device", default_value="/dev/ttyS1",
        description="UART used after obstacle prescan releases the same device",
    )

    # === 两链路共有的 launch 参数（命名完全一致）===
    image_width_launch_arg = DeclareLaunchArgument(
        "dnn_example_image_width", default_value=TextSubstitution(text="640"),
        description="USB 相机拉流宽度（两链路同名共用）")
    image_height_launch_arg = DeclareLaunchArgument(
        "dnn_example_image_height", default_value=TextSubstitution(text="480"),
        description="USB 相机拉流高度（两链路同名共用）")
    msg_pub_topic_name_launch_arg = DeclareLaunchArgument(
        "msg_pub_topic_name",
        default_value=TextSubstitution(text="hobot_dnn_detection"),
        description="DNN 输出 topic（两链路同名；下游 obj_serial / websocket 订阅）")
    minimum_area_arg = DeclareLaunchArgument(
        "min_target_area_px", default_value=TextSubstitution(text="2000"),
        description="Minimum target box width * height in original image pixels squared; 0 disables size filtering")
    raw_detection_topic = PythonExpression([
        "'", LaunchConfiguration("msg_pub_topic_name"), "_raw'"
    ])

    # === 引擎选择（核心开关）===
    engine_arg = DeclareLaunchArgument(
        "dnn_engine",
        default_value=TextSubstitution(text="workaround"),
        description="推理引擎选择：workaround=NCHW+obj_dnn（默认）；native=NHWC+dnn_node_example",
        choices=["workaround", "native"],
    )

    # === workaround 链独有参数 ===
    workaround_model_arg = DeclareLaunchArgument(
        "model_file", default_value=TextSubstitution(text=NCHW_MODEL_FILE),
        description="workaround 链的物块模型（仅 dnn_engine:=workaround 生效）")
    workaround_names_arg = DeclareLaunchArgument(
        "cls_names_list", default_value=TextSubstitution(text=CLASSES_FILE),
        description="类别名文件（workaround 链使用；与 task JSON 内字段同名）")
    workaround_score_arg = DeclareLaunchArgument(
        "score_threshold", default_value=TextSubstitution(text="0.25"),
        description="workaround 链评分阈值")
    workaround_nms_arg = DeclareLaunchArgument(
        "nms_threshold", default_value=TextSubstitution(text="0.45"),
        description="workaround 链 NMS 阈值")

    # === native 链独有参数 ===
    native_task_json_arg = DeclareLaunchArgument(
        "dnn_task_json", default_value=TextSubstitution(text=NHWC_TASK_JSON),
        description="native 链 dnn_node_example 读取的 task JSON（仅 dnn_engine:=native 生效）")
    native_dump_render_arg = DeclareLaunchArgument(
        "dnn_example_dump_render_img", default_value=TextSubstitution(text="0"),
        description="native 链是否保存带框渲染图（仅 dnn_engine:=native 生效）")

    # === 共享节点（两链路都拉起） ===
    # 守护节点：收 /kill_qrc 后用 cap_objdet 拉起 obj_cam.launch.py
    obj_camd_node = Node(
        package="obj_detect",
        executable="obj_camd",
        name="obj_camd",
        parameters=[
            {"usb_video_device": LaunchConfiguration("cap_objdet")},
            {"qrc_video_device": LaunchConfiguration("cap_qrc")},
            {"usb_image_width": LaunchConfiguration("dnn_example_image_width")},
            {"usb_image_height": LaunchConfiguration("dnn_example_image_height")},
            {"usb_framerate": 90},
            {
                "launch_file_path": os.path.join(
                    get_package_share_directory("obj_detect"),
                    "launch/obj_cam.launch.py",
                )
            },
        ],
        output="screen",
    )

    # obj_serial（两链路共用：都订阅 /hobot_dnn_detection）
    target_area_filter_node = Node(
        package="obj_detect",
        executable="obj_target_area_filter",
        name="obj_target_area_filter",
        parameters=[{
            "input_topic": raw_detection_topic,
            "output_topic": LaunchConfiguration("msg_pub_topic_name"),
            "min_target_area_px": LaunchConfiguration("min_target_area_px"),
        }],
        output="screen",
    )
    obj_serial_node = Node(
        package="obj_detect",
        executable="obj_serial",
        name="obj_serial",
        parameters=[{"serial_device": LaunchConfiguration("serial_device")}],
        output="screen",
    )

    # 视频保存（两链路共用：订阅 /image）
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

    # Web 展示（两链路共用）
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

    # === 引擎分支：两链路可任选其一；字段 / topic 完全对齐 ===

    # ---- workaround 链（NCHW 物块模型 + obj_dnn 板端原生推理）----
    obj_dnn_node = Node(
        package="obj_detect",
        executable="obj_dnn",
        name="obj_dnn",
        parameters=[
            {"model_file": LaunchConfiguration("model_file")},
            {"cls_names_list": LaunchConfiguration("cls_names_list")},
            {"image_topic": "/image"},
            {"msg_pub_topic_name": raw_detection_topic},
            {"score_threshold": LaunchConfiguration("score_threshold")},
            {"nms_threshold": LaunchConfiguration("nms_threshold")},
        ],
        output="screen",
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("dnn_engine"), "' == 'workaround'"
        ])),
    )

    # ---- native 链（NHWC 物块模型 + TROS dnn_node_example）----
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
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("dnn_engine"), "' == 'native'"
        ])),
    )
    shared_mem_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("hobot_shm"), "launch/hobot_shm.launch.py"
            )
        ),
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("dnn_engine"), "' == 'native'"
        ])),
    )
    dnn_node_example_node = Node(
        package="dnn_node_example",
        executable="example",
        output="screen",
        parameters=[
            {"config_file": LaunchConfiguration("dnn_task_json")},
            {"dump_render_img": LaunchConfiguration("dnn_example_dump_render_img")},
            {"feed_type": 1},
            {"is_shared_mem_sub": 1},
            {"msg_pub_topic_name": raw_detection_topic},
        ],
        arguments=["--ros-args", "--log-level", "warn"],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("dnn_engine"), "' == 'native'"
        ])),
    )

    return LaunchDescription([
        # args
        cap_objdet_dev_arg, cap_qrc_dev_arg, serial_device_arg,
        engine_arg,
        image_width_launch_arg, image_height_launch_arg, msg_pub_topic_name_launch_arg,
        minimum_area_arg,
        workaround_model_arg, workaround_names_arg,
        workaround_score_arg, workaround_nms_arg,
        native_task_json_arg, native_dump_render_arg,
        # shared nodes (always)
        obj_camd_node, target_area_filter_node, obj_serial_node, video_take_node, objdet_web_node,
        # engine-specific
        obj_dnn_node,
        objdet_nv12_codec_node, shared_mem_node, dnn_node_example_node,
    ])
