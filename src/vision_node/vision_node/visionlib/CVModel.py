import os
from datetime import datetime

import cv2
import numpy as np


_undistort_map1 = None
_undistort_map2 = None
_undistort_roi = None
_undistort_size = None


def _normalize_roi(roi, width, height):
    if roi is None:
        return 0, 0, width, height

    x0, y0, x1, y1 = roi
    x0 = max(0, min(width - 1, int(x0)))
    y0 = max(0, min(height - 1, int(y0)))
    x1 = max(x0 + 1, min(width, int(x1)))
    y1 = max(y0 + 1, min(height, int(y1)))
    return x0, y0, x1, y1


def get_roi(img, cfg, external_roi=None):
    """
    红圆专线 ROI：
    - 默认使用 cfg["roi_range"] 做一次轻量裁切；
    - 若 YOLO 给了 roi，则与默认 ROI 取交集；
    - 若交集无效，则直接退回 YOLO ROI。
    """
    H, W = img.shape[:2]
    roi_range = cfg.get("roi_range", {})

    x0 = int(W * float(roi_range.get("x_min", 0.0)))
    y0 = int(H * float(roi_range.get("y_min", 0.0)))
    x1 = int(W * float(roi_range.get("x_max", 1.0)))
    y1 = int(H * float(roi_range.get("y_max", 1.0)))
    x0, y0, x1, y1 = _normalize_roi((x0, y0, x1, y1), W, H)

    if external_roi is not None:
        ex0, ey0, ex1, ey1 = _normalize_roi(external_roi, W, H)
        ix0 = max(x0, ex0)
        iy0 = max(y0, ey0)
        ix1 = min(x1, ex1)
        iy1 = min(y1, ey1)
        if ix1 > ix0 and iy1 > iy0:
            x0, y0, x1, y1 = ix0, iy0, ix1, iy1
        else:
            x0, y0, x1, y1 = ex0, ey0, ex1, ey1

    roi_img = img[y0:y1, x0:x1]
    return roi_img, x0, y0, x1, y1


# ================== 镜头畸变修正 ==================
def undistort_frame(frame, cfg):
    global _undistort_map1, _undistort_map2, _undistort_roi, _undistort_size

    if not cfg.get("undistort_enable", False):
        return frame

    h, w = frame.shape[:2]
    size = (w, h)

    camera_matrix = np.array(cfg.get("camera_matrix"), dtype=np.float32)
    dist_coeffs = np.array(cfg.get("dist_coeffs"), dtype=np.float32)
    alpha = float(cfg.get("undistort_alpha", 1))

    if camera_matrix.shape != (3, 3) or dist_coeffs.size < 4:
        print("[CV] invalid camera_matrix/dist_coeffs, skip undistort")
        return frame

    if _undistort_size != size:
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
            camera_matrix,
            dist_coeffs,
            size,
            alpha,
            size,
        )

        R = np.eye(3, dtype=np.float32)

        _undistort_map1, _undistort_map2 = cv2.initUndistortRectifyMap(
            camera_matrix,
            dist_coeffs,
            R,
            new_camera_matrix,
            size,
            cv2.CV_16SC2,
        )

        _undistort_roi = roi
        _undistort_size = size
        print("[CV] undistort map initialized")

    if _undistort_map1 is None or _undistort_map2 is None:
        return frame

    corrected = cv2.remap(
        frame,
        _undistort_map1,
        _undistort_map2,
        interpolation=cv2.INTER_LINEAR,
    )

    if alpha == 0 and _undistort_roi is not None:
        x, y, rw, rh = _undistort_roi
        corrected = corrected[y:y + rh, x:x + rw]

    return corrected


# ================== 可视化 ==================
def visualize(frame, result):
    """
    红圆专线显示：默认不在这里叠加任何 OpenCV 辅助标记。
    红圆绘制由 node.py 的 _annotate() 负责。
    """
    return frame.copy()


# ================== 主流程 ==================
def process_frame(frame, cfg, task_cfg=None, roi=None):
    """
    红圆专线主流程：
    - 可选去畸变；
    - 基于 YOLO ROI 生成轻量 ROI；
    - 不再运行白框/透视/scale 主链；
    - 返回给上层用于红圆检测的最小结果结构。
    """
    if frame is None:
        return {
            "frame_found": False,
            "frame_center": None,
            "white_frame_center": None,
            "laser_found": False,
            "laser_center": None,
            "laser_info": None,
            "reference_source": "none",
            "center_source": "none",
            "frame_box": None,
            "scale": 1.0,
            "status": "frame:LOST",
            "mode": "red_circle_only",
            "roi": None,
            "yolo_roi": roi,
            "undistorted": frame,
        }, frame

    corrected = undistort_frame(frame, cfg)
    _, x0, y0, x1, y1 = get_roi(corrected, cfg, external_roi=roi)
    used_roi = (x0, y0, x1, y1)

    result = {
        "frame_found": True,
        "frame_center": None,
        "white_frame_center": None,
        "laser_found": False,
        "laser_center": None,
        "laser_info": None,
        "reference_source": "red_circle_only",
        "center_source": "red_circle_only",
        "frame_box": None,
        "box": None,
        "white_box": None,
        "inner_box": None,
        "black_box": None,
        "outer_box": None,
        "inner_rect": None,
        "white_rect": None,
        "warped": None,
        "scale": 1.0,
        "scale_w": 1.0,
        "scale_h": 1.0,
        "w": corrected.shape[1],
        "h": corrected.shape[0],
        "roi": used_roi,
        "yolo_roi": roi,
        "undistorted": corrected,
        "status": "frame:OK",
        "mode": "red_circle_only",
    }

    vis = visualize(corrected, result) if cfg.get("draw", True) else corrected
    return result, vis
