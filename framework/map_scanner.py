"""map_scanner —— 新摄像头地图采集（骨架）。

采集斜视摄像头在 0/45/90 三个角度的场地图像。
待确认：USB video 节点、分辨率、是否单帧覆盖全场。
"""

ANGLES = (0, 45, 90)


class MapScanner:
    # 全局扫描 = LRCP AR0234（2026-09-10 用户确认），by-id 路径固定
    # 出图节点 /dev/video2 / 伴随节点 /dev/video3 均不可单独使用 /dev/video* 编号
    # （会随插拔顺序漂移）。原占位 /dev/video3 已过时。
    def __init__(
        self,
        video_device="/dev/v4l/by-id/usb-LRCP_AR0234_LRCP_AR0234_01.00.00-video-index0",
        width=1280,
        height=720,
    ):
        raise NotImplementedError("TODO: 打开新摄像头（注意与现有相机的 USB 带宽分时）")

    def capture(self, angles=ANGLES) -> list:
        """按角度列表采集帧，返回 [(angle, frame), ...]。"""
        raise NotImplementedError("TODO: 逐角度采集")

    def release(self):
        raise NotImplementedError("TODO: 释放摄像头")
