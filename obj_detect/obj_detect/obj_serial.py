import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ai_msgs.msg import PerceptionTargets
import datetime
import numpy as np

from serial import Serial

ser_dev = "/dev/ttyS1"


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
        self.xin = 180
        self.yin = 420
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
        def eucilidean_distance(pt1, pt2):
            assert len(pt1) == 2 and len(pt2) == 2, "Points must be 2D in cv::Mat!!"
            pt1, pt2 = np.array(pt1), np.array(pt2)
            return np.sqrt(np.sum((pt1 - pt2) ** 2))

        def target_distance(target):
            if target.type.endswith("cf"):
                return (
                    np.inf
                )  # 如果是圆环，直接返回无穷大，以便所有圆环排序在同一侧：[oooccc...]
            roi = target.rois[0].rect
            return eucilidean_distance(
                (324, 204),
                (roi.x_offset + roi.width // 2, roi.y_offset + roi.height // 2),
            )

        if self.mode == 0:
            return
        targets = sorted(
            msg.targets, key=target_distance, reverse=True
        )  # 降序排序，最后面的就是距离最近的
        if len(targets) == 0:
            return
        nearest = targets[-1]  # Most near object target
        if nearest.type.endswith('of'):
            roi = nearest.rois[0].rect
            ctx = roi.x_offset + roi.width // 2
            cty = roi.y_offset + roi.height // 2
            conf = nearest.rois[0].confidence
            self.get_logger().info(f"{nearest.type}, {ctx}, {cty}, {conf:.2f}")
            self.send(nearest.type, ctx, cty)

        # Note: 省赛，一范围内发送，超出范围的不发送
        for tg in msg.targets:
            if tg.type.endswith("of"):
                break
            roi = tg.rois[0].rect
            ctx = roi.x_offset + roi.width // 2
            cty = roi.y_offset + roi.height // 2
            conf = tg.rois[0].confidence
            self.get_logger().info(f"{tg.type}, {ctx}, {cty}, {conf:.2f}")
            if ctx > 320 - self.xin and ctx < 320 + self.xin and cty < self.yin:
                self.send(tg.type, ctx, cty)
            elif tg.type.endswith("cf"):
                self.send(tg.type, ctx, cty)
            else:
                self.get_logger().info("Target out from region!")

    def name2ser(self, c):
        """判断目标类型并返回对应的序列号"""
        if c == "rcf":
            return 0x31  # 49
        elif c == "gcf":
            return 0x32  # 50
        elif c == "bcf":
            return 0x33  # 51
        elif c == "rof":
            return 0x34  # 52
        elif c == "gof":
            return 0x35  # 53
        elif c == "bof":
            return 0x36  # 54
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
