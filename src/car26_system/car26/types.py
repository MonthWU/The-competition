# Purpose: immutable frame and detection values shared by the four functional modules.

from dataclasses import dataclass


@dataclass(frozen=True)
class Detection:
    label: str
    confidence: float
    x1: int
    y1: int
    x2: int
    y2: int

    # ***
    # 函数名字：area
    # 函数作用：计算当前检测框的非负像素面积。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    # ***
    @property
    def area(self):
        return max(0, self.x2 - self.x1) * max(0, self.y2 - self.y1)

    # ***
    # 函数名字：center
    # 函数作用：计算当前检测框的整数像素中心。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    # ***
    @property
    def center(self):
        return self.x1 + (self.x2 - self.x1) // 2, self.y1 + (self.y2 - self.y1) // 2


@dataclass(frozen=True)
class ScanPoint:
    angle_deg: float
    distance_m: float
    quality: int


@dataclass(frozen=True)
class Obstacle:
    x_m: float
    y_m: float
    width_m: float
    point_count: int


@dataclass(frozen=True)
class RadarFrame:
    sequence: int
    timestamp: float
    points: tuple
    obstacles: tuple


@dataclass(frozen=True)
class CameraFrame:
    role: str
    sequence: int
    timestamp: float
    width: int
    height: int
    detections: tuple = ()
    qr_text: str = ""
    focus_active: bool = False
    focus_delta: tuple = None
    target_alpha: float = None
    disc_request_id: int = 0
    disc_delta: tuple = None
