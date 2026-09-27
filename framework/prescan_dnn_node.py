"""prescan_dnn_node —— 全局扫描 DNN 订阅节点（2026-09-23，2026-09-27 增补留档）。

订阅 /hobot_dnn_detection（PerceptionTargets），过滤 type=='block' 的目标，
把 bbox 中心 + 置信度回调给 mission_dispatcher.scan_angle()。

**2026-09-27 新增**：同时订阅 /image（CompressedImage, mjpeg），缓存最近一帧，
供 mission_dispatcher 在每次收到下位机 [shot] 后**把当时画面留档**（存 jpg），
用于后期人工检查（无论该次是否检出障碍）。

共享内存输入：/hbmem_img（hobot_usb_cam → hobot_codec_decode 的 NV12 输出）。
推理走 dnn_node_example（yolov8 parser, task=task_obj_obstacle.json）。
"""

import os
import threading

import rclpy
from ai_msgs.msg import PerceptionTargets
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


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
        self.last_blocks = []  # [(cx_px, cy_px, conf), ...] 最近一帧 block 检测
        self.frame_seen = 0
        self.done_event = threading.Event()
        self._sub = self.create_subscription(
            PerceptionTargets, "/hobot_dnn_detection", self._cb, 10
        )
        # 2026-09-27：缓存最近一帧相机图，供 [shot] 留档
        self.last_jpeg = None
        self.img_seen = 0
        self._img_sub = self.create_subscription(
            CompressedImage, "/image", self._img_cb, 10
        )

    def _img_cb(self, msg: CompressedImage):
        self.last_jpeg = bytes(msg.data)
        self.img_seen += 1

    def save_last_image(self, path: str) -> bool:
        """把最近一帧相机图（jpeg）存盘，返回是否成功。

        用途：下位机每发一次 [shot]，就把当时画面留档，供后期人工检查
        （**无论该次是否检出障碍** —— 未检出时的画面同样有排查价值）。
        """
        if not self.last_jpeg:
            return False
        d = os.path.dirname(path)
        if d:
            os.makedirs(d, exist_ok=True)
        with open(path, "wb") as f:
            f.write(self.last_jpeg)
        return True

    def _cb(self, msg: PerceptionTargets):
        blocks = []
        for t in msg.targets:
            if t.type == "block":
                roi = t.rois[0].rect
                cx = roi.x_offset + roi.width // 2
                cy = roi.y_offset + roi.height // 2
                conf = t.rois[0].confidence
                blocks.append((cx, cy, float(conf)))
        self.last_blocks = blocks
        self.frame_seen += 1
        self.done_event.set()

    def reset(self):
        self.last_blocks = []
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