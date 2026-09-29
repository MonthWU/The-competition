#!/usr/bin/env python3
"""shot_grab.py —— [shot] 采图模式（标定/检查专用，不跑 dnn 推理）。

用途
----
下位机每发一次 [shot]，就把**当前相机画面留档**到本地，供后期检查 / 单应性标定。
与 `prescan_main.py`（完整障碍识别）的区别：**不做障碍推理**，只留档 + 走协议 ack，
因此不需要 dnn，也不会与已在运行的 prescan 预览链路抢相机。

前提
----
相机预览链路（prescan.launch.py）需**已在运行** —— 本脚本只订阅 /image，不自己拉相机。

协议（与 README §6.1 一致）
---------------------------
  等 [num] 启停位置 → 回 [ack]
  → 每次 [shot] → 存图 + 回 [ack]（云台 0/45/90 三次）
  → 三次完成后发**空地图帧** [0 00]（无障碍；仅供协议闭环，下位机不会卡住）

输出
----
  /root/dev_ws/appli/_tmp_scan_imgs/scan_<序号>_<时间戳>.jpg

用法
----
  python3 shot_grab.py [超时秒数] [串口设备]
"""
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from serial_protocol import (  # noqa: E402
    build_ack, build_map_frame, parse_trigger, parse_start_frame,
)

SCAN_IMG_DIR = "/root/dev_ws/appli/_tmp_scan_imgs"
BAUD = 115200
SERIAL_DEV = "/dev/ttyS1"


class ShotGrab(Node):
    """只缓存 /image 的极简节点。"""

    def __init__(self, name="shot_grab"):
        super().__init__(name)
        self.last_jpeg = None
        self.img_seen = 0
        self._sub = self.create_subscription(
            CompressedImage, "/image", self._img_cb, 10)

    def _img_cb(self, msg: CompressedImage):
        self.last_jpeg = bytes(msg.data)
        self.img_seen += 1

    def save(self, path: str) -> bool:
        if not self.last_jpeg:
            return False
        d = os.path.dirname(path)
        if d:
            os.makedirs(d, exist_ok=True)
        with open(path, "wb") as f:
            f.write(self.last_jpeg)
        return True


def main():
    timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    dev = sys.argv[2] if len(sys.argv) > 2 else SERIAL_DEV

    import serial

    rclpy.init()
    node = ShotGrab()

    # 后台 spin，持续缓存图像
    stop = threading.Event()

    def spin_loop():
        while not stop.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

    th = threading.Thread(target=spin_loop, daemon=True)
    th.start()

    print("[shot_grab] 打开串口 %s @ %d ..." % (dev, BAUD))
    ser = serial.Serial(dev, BAUD, timeout=0.2)
    print("[shot_grab] 就绪。等待下位机 [num] 启停位置 ...")
    print("[shot_grab] 图像源: /image（需 prescan 预览链路已在跑）")

    def wait_frame(parser, deadline):
        buf = bytearray()
        while time.time() < deadline:
            chunk = ser.read(256)
            if chunk:
                buf.extend(chunk)
                r = parser(buf)
                if r is not None:
                    return r
        return None

    try:
        # 1) 等 [num]，回 ack
        if wait_frame(parse_start_frame, time.time() + timeout) is None:
            print("[shot_grab] 超时：未收到 [num]")
            return 1
        ser.write(build_ack())
        print("[shot_grab] 已回 [ack]（启停）")

        # 2) 三次 [shot]，每次存图 + ack
        for i in range(3):
            if wait_frame(parse_trigger, time.time() + timeout) is None:
                print("[shot_grab] 超时：未收到第 %d 次 [shot]" % (i + 1))
                return 1
            # 给 /image 一点缓冲，取到的是「这一刻」的画面
            time.sleep(0.2)
            ts = time.strftime("%Y%m%d_%H%M%S")
            path = os.path.join(SCAN_IMG_DIR, "scan_%03d_%s.jpg" % (i + 1, ts))
            ok = node.save(path)
            print("[shot_grab] 第 %d 次 [shot]：图像留档 %s（img_seen=%d）"
                  % (i + 1, path if ok else "失败(未收到 /image)", node.img_seen))
            ser.write(build_ack())
            print("[shot_grab] 已回 [ack]（第 %d 次）" % (i + 1))

        # 3) 空地图帧（协议闭环）
        ser.write(build_map_frame([]))
        print("[shot_grab] 已下发空地图帧（无障碍）")

        # 留一点时间把最后的数据发完
        time.sleep(0.5)
        return 0
    finally:
        stop.set()
        try:
            ser.close()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("[shot_grab] 串口已释放")


if __name__ == "__main__":
    sys.exit(main())
