"""obstacle_detector —— 黑色几何体识别（2026-09-23 最小可用版）。

主识别：YOLOv11（1 类 ball，framework/dnn/yolo11_x5_obstacle.bin，md5 6fd337ab…）。
标定：单应性矩阵（图像像素 → 网格坐标），离线标定存盘。
"""

# 2026-09-23 全局扫描阶段使用的 yolo 模型（1 类 ball，md5 6fd337ab…）
import os

# 2026-09-25 修正：原写法 `from map_model import OBSTACLE_CANDIDATES_13, grid_id as _grid_id`
# 是错误的 —— grid_id 定义在 serial_protocol.py（map_model 中并无该名字），
# 会导致本模块 import 阶段直接 ImportError，framework 整条链路连启动都进不去。
# 且 _grid_id 在本文件中从未被使用，故一并移除。
from map_model import OBSTACLE_CANDIDATES_13

_DEFAULT_TASK_JSON = "/root/dev_ws/appli/framework/dnn/task_obj_obstacle.json"
_DEFAULT_MODEL_BIN = "/root/dev_ws/appli/framework/dnn/yolo11_x5_obstacle.bin"
_DEFAULT_CLASSES = "/root/dev_ws/appli/framework/dnn/classes_obstacle.names"
# 单应性标定结果存盘路径（由 calibrate_homography.py 生成）
_DEFAULT_HOMOGRAPHY = "/root/dev_ws/appli/framework/dnn/obstacle_homography.npy"


class ObstacleDetector:
    def __init__(self, model_path="", homography_path=_DEFAULT_HOMOGRAPHY,
                 task_json=_DEFAULT_TASK_JSON):
        """加载 yolov11 全局扫描模型（1 类 ball）。

        通过 dnn_node_example 体系跑：此节点只负责输入图像 + 解析 PerceptionTargets，
        推理由配套 launch 启动的 dnn_node_example 节点承担。
        本类保存 config_path 与模型路径，便于 mission_dispatcher 注入共享内存。
        """
        self.model_path = model_path or _DEFAULT_MODEL_BIN
        self.task_json = task_json
        self.classes_path = _DEFAULT_CLASSES
        self.homography_path = homography_path
        # 校验文件存在
        for p in (self.model_path, self.task_json, self.classes_path):
            if not os.path.exists(p):
                raise FileNotFoundError(f"obstacle_detector: missing {p}")
        # 单应性矩阵（3×3），未标定/文件不存在时为 None
        # 2026-09-27 修：homography_path 原默认空串 → 即使标定文件存在也不会加载，
        # pixel_to_grid() 永远返回占位 (2,2)。现默认指向 _DEFAULT_HOMOGRAPHY。
        self._H = None
        if homography_path and os.path.exists(homography_path):
            import numpy as np
            self._H = np.load(homography_path)

    def detect_from_perception(self, perception) -> list:
        """从 PerceptionTargets 解析 block 检测结果。

        返回 [(cx_px, cy_px, conf), ...] 像素坐标（640×640）。
        不做单应性变换，由调用方（scan_angle）决定是否映射到网格。
        """
        blocks = []
        for t in perception.targets:
            if t.type == "block":
                roi = t.rois[0].rect
                cx = roi.x_offset + roi.width // 2
                cy = roi.y_offset + roi.height // 2
                conf = float(t.rois[0].confidence)
                blocks.append((cx, cy, conf))
        return blocks

    def pixel_to_grid(self, cx_px: float, cy_px: float) -> tuple:
        """像素 → 网格坐标 (r, c)。

        需要单应性矩阵 self._H（由 calibrate() 或 calibrate_homography.py 生成）；
        未标定时返回占位 (2, 2)（画面中心），调用方需判断并 log warning。
        """
        if self._H is None:
            # 占位：未标定返回中心 (2,2)，调用方需判断并 log warning
            return (2, 2)
        import numpy as np
        import cv2      # 2026-09-27 修：原函数内只 import numpy，用到 cv2 时 NameError
        pt = np.array([[[cx_px, cy_px]]], dtype=np.float64)
        warped = cv2.perspectiveTransform(pt, self._H)[0][0]
        # warped 是 5×5 网格内的实数坐标；找最近的候选点
        return self._nearest_grid_point(warped[0], warped[1])

    @staticmethod
    def _nearest_grid_point(r: float, c: float) -> tuple:
        """实数 (r, c) → 最近候选点 ((cr, cc), 距离)；限 13 候选点 + 8 节点。"""
        from map_model import NODE_CELLS
        best = None
        for (rr, cc) in OBSTACLE_CANDIDATES_13 + list(NODE_CELLS):
            d = ((rr - r) ** 2 + (cc - c) ** 2) ** 0.5
            if best is None or d < best[1]:
                best = ((rr, cc), d)
        return best[0]

    def detect(self, frame):
        """返回 [(cx, cy, w, h, conf), ...]，像素坐标。

        TODO: 当前 scan_angle 直接读 /hobot_dnn_detection，绕过本方法的单帧入口；
        本方法保留以备将来单帧推理入口（如本地 pyeasy_dnn 直接调用）。
        """
        return []

    def pixel_to_map(self, px):
        """像素坐标 → 地图坐标（单应性变换）。

        TODO: 待单应性标定完成后实现 cv2.perspectiveTransform。
        """
        return self.pixel_to_grid(px[0], px[1])

    def calibrate(self, image_points, map_points):
        """标定单应性矩阵并存盘。"""
        import numpy as np
        import cv2
        src = np.array(image_points, dtype=np.float32)
        dst = np.array(map_points, dtype=np.float32)
        H, _ = cv2.findHomography(src, dst, method=cv2.RANSAC)
        self._H = H
        if self.homography_path:
            np.save(self.homography_path, H)