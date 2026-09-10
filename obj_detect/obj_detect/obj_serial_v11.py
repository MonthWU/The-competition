"""obj_serial_v11 —— 9 类（6 色圆台物块 + 3 放置区标识）串口下发节点（新链路，方案A）。

基于原 obj_serial 改造，**行为与原项目一致**：
  - 物块族（red1/black1/green1/yellow1/blue1/blue2，类比原 of）：每帧只发**最近的 1 个**
  - 标识物族（targetOne/targetTwo/targetThree，类比原 cf）：**全部发送**
  - mode 机制、qrc 心跳、serial_send 回显、FF 帧格式（FF CLASS XL XH YL YH FE）均不变
  - 原始区域过滤开关保留（默认关闭，与原实现实际行为一致；如需启用设 REGION_FILTER=True）

CLASS 映射（按 dnn/classes.names 类序，需与下位机一致）：
  targetOne=0x51 targetTwo=0x52 targetThree=0x53
  red1=0x41 black1=0x42 green1=0x43 yellow1=0x44 blue1=0x45 blue2=0x46
"""
import datetime

import numpy as np
import rclpy
from ai_msgs.msg import PerceptionTargets
from rclpy.node import Node
from serial import Serial
from std_msgs.msg import String

ser_dev = "/dev/ttyS1"

# 物块族（类比原 of：只发最近 1 个）
CLASS_BLOCK = {
    "red1": 0x41,
    "black1": 0x42,
    "green1": 0x43,
    "yellow1": 0x44,
    "blue1": 0x45,
    "blue2": 0x46,
}
# 标识物族（类比原 cf：全部发送）
CLASS_MARK = {
    "targetOne": 0x51,
    "targetTwo": 0x52,
    "targetThree": 0x53,
}


class ByteArray(bytearray):
    def __init__(self, data):
        super().__init__(data)
        self.data = data

    def __str__(self):
        return f"[{', '.join([f'0x{byte:02X}' for byte in self.data])}]"


class ObjSerialV11(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f"Init serial port, node {name} (v11 9cls)")
        self.ser = Serial(ser_dev, 115200)
        self.get_logger().info(f"Serial port {ser_dev} init")
        self.model_res = self.create_subscription(
            PerceptionTargets, "hobot_dnn_detection", self.det_callback, 10
        )
        self.qrc_res = self.create_subscription(
            String, "qrc_result", self.qrc_callback, 10
        )
        self.serial_send_pub = self.create_publisher(String, "serial_send", 10)
        self.xin = 180          # 区域过滤半宽（仅当 REGION_FILTER=True 生效）
        self.yin = 420
        self.ref_pt = (324, 204)  # "最近目标"参考点（沿原项目）
        self.REGION_FILTER = False
        self.mode = 2  # 0: qrc only; 1: det only; 2: both
        self.cnt = 0

    # ---------------- 二维码（与原实现一致） ----------------
    def qrc_callback(self, msg):
        if self.mode == 1:
            return
        self.get_logger().info(f"QRC: {msg.data}")
        if msg.data != "":
            if self.cnt == 50:
                self.cnt = 0
                self.send_qrc(msg.data)
            self.cnt += 1
            if msg.data != "0000000":
                self.send_qrc(msg.data)
                self.send_qrc(msg.data)
                self.send_qrc(msg.data)
                self.send_qrc(msg.data)
                self.mode = 1
                self.get_logger().info("valid qrc -> mode=1 (det only)")

    # ---------------- 检测（新 9 类规则） ----------------
    def det_callback(self, msg):
        if self.mode == 0:
            return

        def dist_to_ref(target):
            roi = target.rois[0].rect
            cx = roi.x_offset + roi.width // 2
            cy = roi.y_offset + roi.height // 2
            return float(np.hypot(cx - self.ref_pt[0], cy - self.ref_pt[1]))

        def in_region(roi):
            cx = roi.x_offset + roi.width // 2
            cy = roi.y_offset + roi.height // 2
            return (320 - self.xin) < cx < (320 + self.xin) and cy < self.yin

        if len(msg.targets) == 0:
            return

        blocks = [t for t in msg.targets if t.type in CLASS_BLOCK]
        marks = [t for t in msg.targets if t.type in CLASS_MARK]

        # 物块：只发最近的 1 个（类比原 of 行为）
        if blocks:
            nearest = min(blocks, key=dist_to_ref)
            if (not self.REGION_FILTER) or in_region(nearest.rois[0].rect):
                roi = nearest.rois[0].rect
                self.send(nearest.type, roi.x_offset + roi.width // 2,
                          roi.y_offset + roi.height // 2)

        # 标识物：全部发送（类比原 cf 行为）
        for tg in marks:
            roi = tg.rois[0].rect
            self.send(tg.type, roi.x_offset + roi.width // 2,
                      roi.y_offset + roi.height // 2)

    # ---------------- 编码/发送 ----------------
    def name2ser(self, c):
        if c in CLASS_MARK:
            return CLASS_MARK[c]
        if c in CLASS_BLOCK:
            return CLASS_BLOCK[c]
        return 0x00

    def send_qrc(self, data):
        data = data.encode("utf-8")
        sent = ByteArray([0xFF, 0x37] + list(data) + [0xFE])
        if len(data) > 0:
            self.ser.write(sent)
            self.get_logger().info(f"QRCode Result: {sent}")
        else:
            self.get_logger().info("Manually close qrc cam")
        self.pub_sent(f'{datetime.datetime.now().strftime("%F.%H:%M:%S")}: qrc: {sent}')

    def send(self, name, ctx, cty):
        sent = ByteArray([0xFF, self.name2ser(name),
                          int(ctx & 0xFF), int(ctx >> 8),
                          int(cty & 0xFF), int(cty >> 8), 0xFE])
        if sent[1] == 0x00:
            self.get_logger().warn(f"unknown class: {name}, skip")
            return
        self.ser.write(sent)
        self.get_logger().info(f"Sent: {sent} ({name})")
        self.pub_sent(f'{datetime.datetime.now().strftime("%F.%H:%M:%S")}: {sent} ({name})')

    def pub_sent(self, sent):
        msg = String()
        msg.data = f'[{datetime.datetime.now().strftime("%F.%H:%M:%S")}]: {sent}'
        self.serial_send_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObjSerialV11("obj_serial_v11")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()