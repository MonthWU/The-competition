"""mission_dispatcher —— 预扫描流程编排节点（骨架，流程规定见 DESIGN.md §6）。

时序（用户定义）：
  [1] 开启摄像头3
  [2] 等待下位机串口指令（读 ttyS1）
  [3-5] 云台 0°/45°/90° 扫描，每角度回传成功指令
  [6] 障碍识别 → 就近候选点 → MapModel 标记
  [7] 其余 x 换 1，输出纯 1/0 地图
  [8] 串口传输 5×5 地图给下位机
  [9] 关闭摄像头3
  [10] 拉起原任务 run_all.launch.py

串口分时：预扫描阶段独占 ttyS1；原任务阶段由 obj_serial 独占。
"""

import serial  # pyserial

SERIAL_DEV = "/dev/ttyS1"
SERIAL_BAUD = 115200

# 摄像头3（预扫描专用，video 设备号待确认）
CAM3_VIDEO_DEVICE = "/dev/video3"  # 暂定

from serial_protocol import (
    TRIGGER_SCAN,
    build_ack,
    build_map_frame,
)


class MissionDispatcher:
    def __init__(self):
        # TODO: 初始化 camera_pan / map_scanner / obstacle_detector / road_judge / MapModel
        self.ser = None
        self.map_model = None
        self.cam3 = None

    # ---- 串口（预扫描阶段独占）----
    def open_serial(self):
        """打开 ttyS1（预扫描阶段独占）。"""
        raise NotImplementedError("TODO: serial.Serial(SERIAL_DEV, SERIAL_BAUD, timeout=...)")

    def close_serial(self):
        """关闭串口，交给原任务阶段 obj_serial。"""
        raise NotImplementedError("TODO: 释放串口")

    def wait_trigger(self) -> bytes:
        """[2] 等待下位机触发指令（超时后返回空）。"""
        raise NotImplementedError("TODO: 读串口，匹配 TRIGGER_SCAN")

    def send_ack(self, angle: float):
        """[3-5] 每角度扫描完成后回传成功指令。"""
        raise NotImplementedError("TODO: 调 build_ack(angle) 并 ser.write()")

    def send_map(self):
        """[8] 把纯 1/0 地图矩阵发给下位机。"""
        raise NotImplementedError("TODO: 调 build_map_frame(map_model.to_list()) 并 ser.write()")

    # ---- 摄像头3 ----
    def start_cam3(self):
        """[1] 开启摄像头3。"""
        raise NotImplementedError("TODO: 打开 CAM3_VIDEO_DEVICE（注意与其它相机的 USB 分时）")

    def kill_cam3(self):
        """[9] 关闭摄像头3。"""
        raise NotImplementedError("TODO: 释放摄像头3")

    # ---- 主流程 ----
    def run_prescan(self):
        """执行 [1]-[9] 预扫描流程。"""
        raise NotImplementedError("TODO: 按 DESIGN.md §6 时序串联")

    def launch_original(self):
        """[10] 拉起原任务 launch（原代码零改动）。"""
        raise NotImplementedError("TODO: subprocess 调 run_all.launch.py")


def main():
    disp = MissionDispatcher()
    disp.run_prescan()
    disp.launch_original()


if __name__ == "__main__":
    main()
