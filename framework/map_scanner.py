"""map_scanner —— 新摄像头地图采集（骨架）。

采集斜视摄像头在 0/45/90 三个角度的场地图像。
待确认：USB video 节点、分辨率、是否单帧覆盖全场。
"""

ANGLES = (0, 45, 90)


class MapScanner:
    def __init__(self, video_device="/dev/videoX", width=1280, height=720):
        raise NotImplementedError("TODO: 打开新摄像头（注意与现有相机的 USB 带宽分时）")

    def capture(self, angles=ANGLES) -> list:
        """按角度列表采集帧，返回 [(angle, frame), ...]。"""
        raise NotImplementedError("TODO: 逐角度采集")

    def release(self):
        raise NotImplementedError("TODO: 释放摄像头")
