"""camera_pan —— 云台转角控制接口（骨架）。

控制斜视摄像头（车顶）旋转到 0°/45°/90° 采集地图。
待确认：舵机/云台型号与转角精度；尺寸须满足 300×300 投影约束。
"""

class CameraPan:
    def __init__(self, channel=0, min_angle=0, max_angle=180):
        raise NotImplementedError("TODO: 初始化云台（PWM/舵机）")

    def goto(self, angle: float):
        """转到指定角度（0/45/90...），并等待稳定。"""
        raise NotImplementedError("TODO: 云台转角控制")

    def home(self):
        """回到初始角度。"""
        raise NotImplementedError("TODO: 回零")
