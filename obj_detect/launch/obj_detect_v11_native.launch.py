"""obj_detect_v11_native.launch —— 已弃用（DEPRECATED，2026-09-29）。

⚠️ 该 launch 文件已并入 `obj_detect_v11.launch.py`（统一入口）。
   本文件保留仅作为**向后兼容跳转**：原 `run_all.launch.py` 仍把它当工作入口时，
   会自动 fall-through 到统一入口（按 `dnn_engine:=native` 透传，与原行为一致）。

迁移说明
--------
- 原行为：拉起 obj_dnn 板端原生推理（绕开 dnn_node_example 段错误）
- 统一入口等价：ros2 launch obj_detect obj_detect_v11.launch.py dnn_engine:=workaround
  （workaround = obj_dnn 板端原生；native = dnn_node_example + NHWC 模型）

完整迁移方式
------------
- 旧调用：run_all.launch.py 走 obj_detect_v11_native.launch.py → obj_dnn
- 新调用：run_all.launch.py 走 obj_detect_v11.launch.py dnn_engine:=workaround|native

删除时机
--------
- 当你确认 dnn_engine:=workaround 链路在统一入口下能跑通、且下游全部使用新入口时，
  可删除本文件；同步删除 obj_detect/setup.py 中 `obj_dnn = obj_detect.obj_dnn:main`
  条目即移除绕行节点。
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution
from launch.actions import DeclareLaunchArgument

# 转发到统一入口，等价于：
#   ros2 launch obj_detect obj_detect_v11.launch.py dnn_engine:=native
# 注：原"native"语义实际是"obj_dnn 板端原生推理"（绕开 dnn_node_example 段错误），
#     这与新 launch 里 dnn_engine:=workaround 等价 —— 名称虽然不同，行为一致。
LEGACY_DNN_ENGINE = "workaround"


def generate_launch_description():
    # 大声告知调用方："这条入口已弃用，已自动跳到统一入口"
    print(
        "[DEPRECATED] obj_detect_v11_native.launch.py 已并入 obj_detect_v11.launch.py；"
        f"等价参数 dnn_engine:={LEGACY_DNN_ENGINE}。"
        "请改用 ros2 launch obj_detect obj_detect_v11.launch.py。"
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    os.path.dirname(__file__),
                    "obj_detect_v11.launch.py",
                )
            ),
            launch_arguments={
                "dnn_engine": TextSubstitution(text=LEGACY_DNN_ENGINE),
            }.items(),
        ),
    ])
