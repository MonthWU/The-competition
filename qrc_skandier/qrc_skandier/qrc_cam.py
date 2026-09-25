import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String

import cv2 as cv
import numpy as np
import os

from utils.neo_img_trans import cv2ros
from utils.threCam import ThreadCap


def resolve_cam_identity(dev_path: str) -> tuple:
    """把 /dev/videoN 或 by-id 路径解析为 (内核卡名, by-id 名)。

    用于在打开相机前校验"这到底是不是扫码相机"，避免静默打开错误设备。
    """
    real = os.path.realpath(dev_path)
    name = os.path.basename(real)
    card = ""
    try:
        with open(f"/sys/class/video4linux/{name}/name") as f:
            card = f.read().strip()
    except OSError:
        pass
    byid = ""
    try:
        for entry in sorted(os.listdir("/dev/v4l/by-id")):
            if os.path.realpath(os.path.join("/dev/v4l/by-id", entry)) == real:
                byid = entry
                break
    except OSError:
        pass
    return card, byid


class QrcCam(Node):
    def __init__(self, name):
        super().__init__(name)

        self.declare_parameter("cam_idx", "/dev/video0")
        self.declare_parameter("fps", 240)
        self.declare_parameter("img_width", 640)
        self.declare_parameter("img_height", 400)
        # 期望的相机身份关键字（内核卡名 / by-id 名任一命中即通过）。
        # 扫码相机 = KINGSEN KS1A293（唯一支持 240fps@640x400 那台）。
        # 传空字符串 "" 可跳过校验（仅在明确知道后果时使用）。
        self.declare_parameter("expect_id", "KS1A293")
        # self.declare_parameter("img_fourcc", cv.VideoWriter.fourcc(*"MJPG"))

        cam_idx = self.get_parameter("cam_idx").get_parameter_value().string_value
        expect_id = (
            self.get_parameter("expect_id").get_parameter_value().string_value
        )

        card, byid = resolve_cam_identity(cam_idx)
        self.get_logger().info(f"QrcCam Node {name}")
        self.get_logger().info(
            f'cam_idx="{cam_idx}" -> {os.path.realpath(cam_idx)} '
            f'card="{card}" by-id="{byid}"'
        )
        if expect_id and expect_id not in f"{card} {byid}":
            self.get_logger().error(
                f"相机身份校验失败：期望标识 '{expect_id}'，实际 card=\"{card}\" "
                f'by-id="{byid}"（cam_idx={cam_idx}）。'
                f"扫码相机必须是 usb-KINGSEN_KS1A293-video-index0；"
                f"拒绝启动，避免静默扫不出码。"
            )
            raise RuntimeError(
                f"qrc_cam camera identity mismatch: expect '{expect_id}', "
                f'got card="{card}" by-id="{byid}"'
            )
        self.get_logger().info(f"相机身份校验通过：命中 '{expect_id}'")

        self.cam = ThreadCap(
            cam_idx,
            self.get_parameter("img_width").get_parameter_value().integer_value,
            self.get_parameter("img_height").get_parameter_value().integer_value,
            self.get_parameter("fps").get_parameter_value().integer_value,
        )
        self.get_logger().info(f"Using {cam_idx} for QRCcode Scan")

        # self.cam = cv.VideoCapture(
        #     self.get_parameter("cam_idx").get_parameter_value().integer_value
        # )
        # self.cam.set(
        #     cv.CAP_PROP_FRAME_WIDTH,
        #     self.get_parameter("img_width").get_parameter_value().integer_value,
        # )
        # self.cam.set(
        #     cv.CAP_PROP_FRAME_HEIGHT,
        #     self.get_parameter("img_height").get_parameter_value().integer_value,
        # )
        # self.cam.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc(*"MJPG"))
        # self.cam.set(
        #     cv.CAP_PROP_FPS,
        #     self.get_parameter("fps").get_parameter_value().integer_value,
        # )

        # self.qrc_image_pub = self.create_publisher(Image, "qrc_image", 10)
        self.qrc_image_pub = self.create_publisher(CompressedImage, "qrc_image", 10)

        timer_period = 0.01  # seconds
        self.qrc_image_pub_timer = self.create_timer(
            timer_period, self.qrc_image_pub_callback
        )
        self.msg = CompressedImage()

        self.frame_count = 0
        self.fps_timer = self.create_timer(1, self.fps_callback)

        self._done = False
        self.shutdown_sub = self.create_subscription(
            String,
            "kill_qrc",
            self.shutdown,
            10,
        )

    def qrc_image_pub_callback(self):
        _, frame = self.cam.read()
        if frame is None:
            self.get_logger().info("Get None Pic")
            return
        frame = cv.resize(frame, (0, 0), fx=0.35, fy=0.35)
        # cv.imshow("fr", frame)
        # cv.waitKey(1)
        self.msg.data = cv2ros(frame)
        # self.msg.width = frame.shape[1]
        # self.msg.height = frame.shape[0]
        # self.msg.encoding = "jpeg"
        # self.msg.step = frame.shape[1] * 2
        self.qrc_image_pub.publish(self.msg)
        self.frame_count += 1

    def fps_callback(self):
        self.get_logger().info(f"QRC Cam: Pub FPS: {self.frame_count}")
        self.frame_count = 0

    def shutdown(self, msg):
        # 注意：destroy_node() 之后不能再访问节点句柄（get_name / get_logger 会抛
        # rclpy._rclpy_pybind11.InvalidHandle）。故日志与硬件释放全部前置，
        # 节点销毁统一交给 main 的 finally，避免二次销毁。
        self.get_logger().info(f"收到 kill 信号: {msg.data}，释放摄像头并退出")
        self.cam.release()
        self.get_logger().info(f"节点 {self.get_name()} 已释放摄像头")
        # 不在回调里调用 rclpy.shutdown()：实测会让 rclpy.spin() 挂住不返回、
        # 进程残留。改为置标志位，由 main 的 spin_once 循环统一收尾
        # （与 qrc_cam_killer 同款，已实测可靠）。
        self._done = True


def main():
    rclpy.init()
    qrccam = QrcCam("qrc_cam")
    try:
        while rclpy.ok() and not qrccam._done:
            rclpy.spin_once(qrccam, timeout_sec=0.2)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        qrccam.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()
