import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String

import cv2 as cv
import numpy as np
import pyzbar.pyzbar as pyzbar
import time
import datetime


from utils.neo_img_trans import ros2cv
from qrc_skandier.task_code import is_valid_task_code


class QrcScanner(Node):
    def __init__(self, name):
        super().__init__(name)

        # self.qrc_img_sub = self.create_subscription(
        #     Image,
        #     "qrc_image",
        #     # "image_mjpeg",
        #     self.scan_code,
        #     10,
        # )
        self.qrc_img_sub = self.create_subscription(
            CompressedImage,
            "qrc_image",
            # "image_mjpeg",
            self.scan_code,
            10,
        )
        self.res_pub = self.create_publisher(String, "qrc_result", 10)
        self.show = True
        self._done = False
        self.shutdown_sub = self.create_subscription(
            String,
            "kill_qrc",
            self.shutdown,
            10,
        )

    def scan_code(self, msg):
        t0 = time.time()
        cv_img = ros2cv(msg)
        # cv_img = msg.data
        # cv_img = np.reshape(cv_img, (400, 640, 3))
        # cv_img = cv.cvtColor(cv_img, cv.COLOR_BGR2GRAY)
        # cv_img = cv.resize(cv_img, (0,0),fx=0.35, fy=0.35)
        # self.get_logger().info(f"{len(msg.data)}")
        # self.get_logger().info(f"{cv_img.shape}")
        # cv_img = msg.data
        if cv_img is None:
            # self.res_pub.publish(String(data="Get None image"))
            self.get_logger().info("Get None image")
            return
        decoded_objects = pyzbar.decode(cv_img)
        if decoded_objects:
            try:
                text = decoded_objects[0].data.decode("utf-8").strip()
            except UnicodeDecodeError:
                text = ""
            if is_valid_task_code(text):
                self.get_logger().info(f"Detected valid task code: {text}")
                self.res_pub.publish(String(data=text))
            else:
                self.get_logger().warn(f"Ignoring invalid QR payload: {text!r}")
                self.res_pub.publish(String(data="0000000"))
        else:
            self.res_pub.publish(String(data="0000000")) # send an empty string if no qrc detected, to show running
        self.get_logger().info(f"Scan time: {time.time() - t0:.3f}")
        
        # TODO: Remove this part showing the image instead, using a flask server
        # cv.imshow("qrc", cv_img)
        # cv.waitKey(1)
        # if self.show:
        #     try:
        #         cv.imshow("qrc", cv_img)
        #         if cv.waitKey(1) == ord("q"):
        #             cv.destroyAllWindows()
        #             self.show = False
        #     except Exception as e:
        #         self.get_logger().info(f"Show image error: {e}")
        #         self.show = False

    def shutdown(self, msg):
        # destroy_node() 之后访问节点句柄会抛 InvalidHandle，故日志前置；
        # 节点销毁统一交给 main 的 finally。
        self.get_logger().info(f"收到 kill 信号: {msg.data}，节点 {self.get_name()} 退出")
        # 不在回调里调用 rclpy.shutdown()（会让 rclpy.spin() 挂住、进程残留），
        # 改为置标志位，由 main 的 spin_once 循环统一收尾。
        self._done = True


def main():
    rclpy.init()
    qrc_scanner = QrcScanner("qrc_cam")
    try:
        while rclpy.ok() and not qrc_scanner._done:
            rclpy.spin_once(qrc_scanner, timeout_sec=0.2)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        qrc_scanner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()
