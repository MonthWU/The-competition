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

    ⚠️ 已弃用（2026-09-25）：改用 by-id 固定（见下方 CAM_QRC / CAM_OBJDET）。
    帧率排序没有任何身份校验，相机掉线（get_max_fps 返回 None 被过滤）或
    插拔导致 /dev/video* 重排时会静默指错相机，现象仅为"扫不出码"。
    保留本函数仅供历史参考，generate_launch_description() 不再调用。
    """
    return sorted(dev_nodes, key=lambda x: get_max_fps(x), reverse=True)


# 三路相机 by-id 固化（2026-09-23 二次修正后），与 obj_detect_v11.launch.py /
# framework/launch/prescan.launch.py 保持一致。by-id 基于设备 VID:PID + 序列号，
# 不受 /dev/video* 编号漂移影响。
CAM_QRC = "/dev/v4l/by-id/usb-KINGSEN_KS1A293-video-index0"                     # 扫码（唯一 240fps@640x400）
CAM_OBJDET = "/dev/v4l/by-id/usb-LRCP_AR0234_LRCP_AR0234_01.00.00-video-index0"  # 检测（拍物块区）


def _check_cam(path: str, role: str) -> str:
    """校验 by-id 路径存在；缺失时大声告警（不回退到猜测模式）。"""
    if not os.path.exists(path):
        print(
            f"[run_all] 错误：{role}相机 {path} 不存在！请检查 USB 接线（by-id 路径）。"
            f"已禁用猜测回退，避免静默打开错误相机。"
        )
    return path



def generate_launch_description():
    cap_qrc_devnode = _check_cam(CAM_QRC, "扫码")
    cap_objdet_devnode = _check_cam(CAM_OBJDET, "检测")

    obj_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("obj_detect"),
                # 2026-09-25（P6）：一键入口切到 v11 native 链路 —— obj_dnn 板端原生推理
                # + 9 类物块/标识模型（dnn/yolo11_x5.bin）。
                # 原 obj_detect.launch.py（dnn_node_example + 老 6 类 task_obj.json）因模型
                # 输出格式与 TROS parser 不兼容会段错误，已弃用；文件保留以便回退。
                "launch/obj_detect_v11_native.launch.py",
            )
        ),
        launch_arguments={
            "cap_objdet": cap_objdet_devnode,
            "cap_qrc": cap_qrc_devnode,
        }.items(),
    )

    qrc_skandier = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("qrc_skandier"),
                "launch/qrc_skandier.launch.py",
            )
        ),
        launch_arguments={
            "cap_qrc": cap_qrc_devnode,
        }.items(),
    )

    return LaunchDescription(
        [
            obj_detection,
            qrc_skandier,
        ]
    )
