import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ai_msgs.msg import PerceptionTargets
import datetime
import numpy as np

from serial import Serial

ser_dev = "/dev/ttyS1"

# 物块族（圆台物块 6 色，yolov11 新模型）：每帧只发离参考点最近的 1 个
CLASS_BLOCK = {
    "red1": 0x41,
    "black1": 0x42,
    "green1": 0x43,
    "yellow1": 0x44,
    "blue1": 0x45,
    "blue2": 0x46,
}
# 放置区标识（3 种，yolov11 新模型）：全部发送
CLASS_MARK = {
    "targetOne": 0x51,
    "targetTwo": 0x52,
    "targetThree": 0x53,
}
# 兼容保留：旧模型（250720_v5s）的物块 of / 圆环 cf
CLASS_BLOCK_LEGACY = {"rof": 0x34, "gof": 0x35, "bof": 0x36}
CLASS_MARK_LEGACY = {"rcf": 0x31, "gcf": 0x32, "bcf": 0x33}


def is_block(t):
    """物块族判别：圆台物块（新，6 色）或旧 of 物块"""
    return t in CLASS_BLOCK or t in CLASS_BLOCK_LEGACY


def is_mark(t):
    """放置区标识判别：标识物（新，3 种）或旧圆环 cf"""
    return t in CLASS_MARK or t in CLASS_MARK_LEGACY


class ByteArray(bytearray):
    def __init__(self, data):
        super().__init__(data)
        self.data = data

    def __str__(self):
        return f"[{', '.join([f'0x{byte:02X}' for byte in self.data])}]"


class ObjSerial(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f"Init serial port, node {name}")
        self.ser = Serial(ser_dev, 115200)
        self.get_logger().info(f"Serial port {ser_dev} init")
        self.model_res = self.create_subscription(
            PerceptionTargets, "hobot_dnn_detection", self.det_callback, 10
        )
        self.qrc_res = self.create_subscription(
            String, "qrc_result", self.qrc_callback, 10
        )
        self.serial_send_pub = self.create_publisher(String, "serial_send", 10)

        # ---- 区域过滤 + "最近目标"参考点（2026-09-25 参数化补齐）----
        # 检测链路实际图像系为 640x480（obj_cam.launch.py 默认）。
        # 历史遗留：self.xin=180 / self.yin=420 定义了却从未被引用 —— README §4 声称的
        # 区域过滤此前**并未实现**。此处补上；默认阈值由旧规格等比换算：
        #   旧规格（README §4，960x544 图像系）：x∈[140,500]、y<420
        #   换算到 640x480：x∈[93,333]、y<371
        # 现场可用参数覆盖（ros2 param set / launch 参数）重新标定。
        self.declare_parameter("ref_pt_x", 320)            # "最近目标"参考点，默认画面中心
        self.declare_parameter("ref_pt_y", 240)
        self.declare_parameter("enable_roi_filter", True)  # 物块区域过滤开关
        # 默认值已按 2026-09-25 实测画面修正：现场 4 个物块的实际图像坐标为
        #   yellow1 (614,438) / black1 (520,406) / black1 (557,397) / red1 (621,184)
        # 即集中在画面右侧 x∈[506,639]、y∈[160,476]。若沿用 README 旧规格（960x544）
        # 等比换算出的 x∈[93,333]，会把全部目标滤掉（实测：串口一条都不发）。
        # 现默认外扩为覆盖实测范围（等价于只滤左侧干扰），**现场仍需复核标定**。
        self.declare_parameter("roi_x_min", 380)
        self.declare_parameter("roi_x_max", 640)
        self.declare_parameter("roi_y_max", 480)

        self.ref_pt = (
            float(self.get_parameter("ref_pt_x").value),
            float(self.get_parameter("ref_pt_y").value),
        )
        self.enable_roi = bool(self.get_parameter("enable_roi_filter").value)
        self.roi_x_min = float(self.get_parameter("roi_x_min").value)
        self.roi_x_max = float(self.get_parameter("roi_x_max").value)
        self.roi_y_max = float(self.get_parameter("roi_y_max").value)
        self.get_logger().info(
            f"ref_pt={self.ref_pt}, roi_filter={self.enable_roi} "
            f"x∈[{self.roi_x_min},{self.roi_x_max}] y<={self.roi_y_max}"
        )
        self.mode = 2  # 0: send qrcode info; 1: send obj det results; 2: send both
        # self.call_opened() # no send a startup signal. Send all even empty qrcode data
        self.cnt = 0

    def call_opened(self):
        self.ser.write(ByteArray([0xFF, 0xFF, 0xFF, 0xFE]))
        self.get_logger().info("Serial port opened!")
        self.get_logger().info("Sent: [0xFF, 0xFF, 0xFF, 0xFE]")
        self.pub_sent("Serial port opened!")

    def qrc_callback(self, msg):
        if self.mode == 1:
            return
        self.get_logger().info(f"QRC: {msg.data}")
        if msg.data != "":
            if self.cnt == 50:
                self.cnt = 0
                self.send_qrc(msg.data)
            self.cnt += 1
            if (
                msg.data != "0000000"
            ):  # 如果识别到了二维码（valid data）就多发送几次后再切换模式
                self.send_qrc(msg.data)
                self.send_qrc(msg.data)
                self.send_qrc(msg.data)
                self.send_qrc(msg.data)
                self.mode = 1  # valid adata scanned to set flag to 1

    def det_callback(self, msg):
        # self.get_logger().info("Det recvd!")
        # print(msg.targets)
        if self.mode == 0:
            return

        def center(target):
            roi = target.rois[0].rect
            return (roi.x_offset + roi.width // 2, roi.y_offset + roi.height // 2)

        def in_roi(target):
            """物块区域过滤：只保留"物块投放区"内的目标（可用参数关闭）。"""
            if not self.enable_roi:
                return True
            cx, cy = center(target)
            return (self.roi_x_min <= cx <= self.roi_x_max) and (cy <= self.roi_y_max)

        def dist_to_ref(target):
            cx, cy = center(target)
            return float(np.hypot(cx - self.ref_pt[0], cy - self.ref_pt[1]))

        # 物块族（圆台物块 6 色 / 旧 of）：区域过滤后，每帧只发离参考点最近的 1 个
        blocks = [t for t in msg.targets if is_block(t.type)]
        if self.enable_roi:
            blocks = [t for t in blocks if in_roi(t)]
        if blocks:
            nearest = min(blocks, key=dist_to_ref)
            ctx, cty = center(nearest)
            conf = nearest.rois[0].confidence
            self.get_logger().info(f"{nearest.type}, {ctx}, {cty}, {conf:.2f}")
            self.send(nearest.type, ctx, cty)

        # 放置区标识（3 种 / 旧圆环 cf）：全部发送（不受区域过滤限制）
        for tg in msg.targets:
            if not is_mark(tg.type):
                continue
            ctx, cty = center(tg)
            conf = tg.rois[0].confidence
            self.get_logger().info(f"{tg.type}, {ctx}, {cty}, {conf:.2f}")
            self.send(tg.type, ctx, cty)

    def name2ser(self, c):
        """判断目标类型并返回对应的序列号"""
        if c in CLASS_MARK:
            return CLASS_MARK[c]         # 放置区标识 0x51~0x53
        elif c in CLASS_BLOCK:
            return CLASS_BLOCK[c]        # 圆台物块 0x41~0x46
        elif c in CLASS_MARK_LEGACY:
            return CLASS_MARK_LEGACY[c]  # 旧圆环 0x31~0x33
        elif c in CLASS_BLOCK_LEGACY:
            return CLASS_BLOCK_LEGACY[c] # 旧物块 0x34~0x36
        return 0x00

    def send_qrc(self, data):
        data = data.encode("utf-8")
        sent = [0xFF]
        sent.append(0x37)  # class: qrc
        sent.extend(data)  # data
        sent.append(0xFE)
        # self.get_logger().info(f"QRC: {sent}")
        # self.pub_sent(sent)
        sent = ByteArray(sent)
        if len(data) > 0:
            self.ser.write(sent)
            self.get_logger().info(f"QRCode Result: {sent}")
        else:
            self.get_logger().info("Manually close qrc cam")
        self.pub_sent(f'{datetime.datetime.now().strftime("%F.%H:%M:%S")}: qrc: {sent}')

    def send(self, name, ctx, cty):
        sent = [0xFF]
        sent.append(self.name2ser(name))  # class
        # sent.append(int(ctx * 253 / 640))
        # sent.append(int(cty * 253 / 480))
        sent.append(int(ctx & 0xFF))  # low x
        sent.append(int(ctx >> 8))  # high x
        sent.append(int(cty & 0xFF))  # low y
        sent.append(int(cty >> 8))  # high y
        sent.append(0xFE)
        # self.get_logger().info(f"Data: {sent}")
        # self.pub_sent(sent)
        sent = ByteArray(sent)
        self.ser.write(sent)
        self.get_logger().info(f"Sent: {sent}")
        self.pub_sent(f'{datetime.datetime.now().strftime("%F.%H:%M:%S")}: {sent}')

    def pub_sent(self, sent):
        msg = String()
        # msg.data = f'{sent}'
        msg.data = f'[{datetime.datetime.now().strftime("%F.%H:%M:%S")}]: {sent}'
        self.serial_send_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    serial_node = ObjSerial("obj_serial")
    rclpy.spin(serial_node)
    rclpy.shutdown()
