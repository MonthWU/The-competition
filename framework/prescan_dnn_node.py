"""prescan_dnn_node —— 全局扫描 DNN 订阅节点（2026-09-23）。

订阅 /hobot_dnn_detection（PerceptionTargets），过滤 type=='ball' 的目标，
把 bbox 中心 + 置信度回调给 mission_dispatcher.scan_angle()。

共享内存输入：/hbmem_img（hobot_usb_cam → hobot_codec_decode 的 NV12 输出，640×640）。
推理走 dnn_node_example（yolov8 parser, task=task_obj_obstacle.json）。
"""

import threading

import rclpy
from ai_msgs.msg import PerceptionTargets
from rclpy.node import Node


class PrescanDnnNode(Node):
    """订阅全局扫描阶段的 DNN 输出，回调通知 scan_angle()。

    使用方式（在 scan_angle 内）：
        node = PrescanDnnNode()
        rclpy.spin_once(node, timeout_sec=N)  # 直到 done_event set
        if node.last_balls:
            ...
    """

    def __init__(self, name="prescan_dnn"):
        super().__init__(name)
        self.last_balls = []  # [(cx_px, cy_px, conf), ...] 最近一帧 ball 检测
        self.frame_seen = 0
        self.done_event = threading.Event()
        self._sub = self.create_subscription(
            PerceptionTargets, "/hobot_dnn_detection", self._cb, 10
        )

    def _cb(self, msg: PerceptionTargets):
        balls = []
        for t in msg.targets:
            if t.type == "ball":
                roi = t.rois[0].rect
                cx = roi.x_offset + roi.width // 2
                cy = roi.y_offset + roi.height // 2
                conf = t.rois[0].confidence
                balls.append((cx, cy, float(conf)))
        self.last_balls = balls
        self.frame_seen += 1
        self.done_event.set()

    def reset(self):
        self.last_balls = []
        self.frame_seen = 0
        self.done_event.clear()


def main(args=None):
    rclpy.init(args=args)
    node = PrescanDnnNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()