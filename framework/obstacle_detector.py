"""obstacle_detector —— 黑色几何体识别（骨架）。

主识别：YOLOv11（1 类 obstacle，.bin 经 rdk_model_zoo ultralytics_yolo 转换）。
兜底：CV 黑色分割（低亮度阈值 + 连通域 + 面积/圆度过滤），形状无关，防漏检。
标定：单应性矩阵（图像像素 → 地面坐标），离线标定存盘。
"""

class ObstacleDetector:
    def __init__(self, model_path="", homography_path=""):
        raise NotImplementedError("TODO: 加载 YOLOv11 .bin / hobot_dnn 或 easy_dnn；加载 H 矩阵")

    def detect(self, frame):
        """返回 [(cx, cy, w, h, conf), ...]，像素坐标。"""
        raise NotImplementedError("TODO: YOLOv11 推理 + CV 分割兜底，双通道投票")

    def pixel_to_map(self, px) -> tuple:
        """像素坐标 → 地图坐标（单应性变换）。"""
        raise NotImplementedError("TODO: cv2.perspectiveTransform")

    def calibrate(self, image_points, map_points):
        """标定单应性矩阵并存盘。"""
        raise NotImplementedError("TODO: cv2.findHomography + save")
