import os
from datetime import datetime

import cv2
import numpy as np


_undistort_map1 = None
_undistort_map2 = None
_undistort_roi = None
_undistort_size = None


# ================== 基础工具 ==================
def order_points(pts):
    """
    将四个点排序为：左上、右上、右下、左下。
    """
    pts = np.asarray(pts, dtype=np.float32)

    s = pts.sum(axis=1)
    diff = np.diff(pts, axis=1)

    rect = np.zeros((4, 2), dtype=np.float32)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]

    return rect


def rect_size_sorted(rect):
    w, h = rect[1]

    if w <= 0 or h <= 0:
        return 0.0, 0.0

    short_side = min(float(w), float(h))
    long_side = max(float(w), float(h))
    return short_side, long_side


def rect_ratio(rect):
    short_side, long_side = rect_size_sorted(rect)
    if short_side <= 0:
        return 0.0
    return long_side / short_side


def contour_touches_roi_edge(cnt, roi_w, roi_h, margin=4):
    x, y, w, h = cv2.boundingRect(cnt)

    if x <= margin:
        return True
    if y <= margin:
        return True
    if x + w >= roi_w - margin:
        return True
    if y + h >= roi_h - margin:
        return True

    return False


def _normalize_roi(roi, width, height):
    if roi is None:
        return 0, 0, width, height

    x0, y0, x1, y1 = roi
    x0 = max(0, min(width - 1, int(x0)))
    y0 = max(0, min(height - 1, int(y0)))
    x1 = max(x0 + 1, min(width, int(x1)))
    y1 = max(y0 + 1, min(height, int(y1)))

    return x0, y0, x1, y1


def _expand_roi(roi, width, height, margin=0):
    if roi is None:
        return 0, 0, width, height

    x0, y0, x1, y1 = roi
    margin = int(margin)
    return _normalize_roi((x0 - margin, y0 - margin, x1 + margin, y1 + margin), width, height)


def get_roi(img, cfg, external_roi=None):
    """
    获取检测 ROI：
    - 默认使用 cfg["roi_range"] 去掉最外圈；
    - 如果 YOLO 给了 external_roi，则取二者交集。
    """
    H, W = img.shape[:2]
    roi_range = cfg.get("roi_range", {})

    x0 = int(W * float(roi_range.get("x_min", 0.10)))
    y0 = int(H * float(roi_range.get("y_min", 0.10)))
    x1 = int(W * float(roi_range.get("x_max", 0.90)))
    y1 = int(H * float(roi_range.get("y_max", 0.90)))

    x0, y0, x1, y1 = _normalize_roi((x0, y0, x1, y1), W, H)

    if external_roi is not None:
        ex0, ey0, ex1, ey1 = _normalize_roi(external_roi, W, H)
        x0 = max(x0, ex0)
        y0 = max(y0, ey0)
        x1 = min(x1, ex1)
        y1 = min(y1, ey1)

        if x1 <= x0 or y1 <= y0:
            x0, y0, x1, y1 = _normalize_roi(external_roi, W, H)

    roi_img = img[y0:y1, x0:x1]
    return roi_img, x0, y0, x1, y1


def rect_from_local_to_global(rect, ox, oy):
    box = cv2.boxPoints(rect).astype(np.float32)
    box[:, 0] += float(ox)
    box[:, 1] += float(oy)
    return cv2.minAreaRect(box)


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


# ================== 内白框检测 ==================
def _near_scene_params(cfg, roi_w, roi_h, cnt):
    """根据候选大小判断是否进入近景大目标分支。"""
    x, y, w, h = cv2.boundingRect(cnt)
    area_ratio = (w * h) / float(max(1, roi_w * roi_h))
    side_ratio = max(w, h) / float(max(1, min(roi_w, roi_h)))
    near_area_ratio = float(cfg.get("near_area_ratio", 0.10))
    near_side_ratio = float(cfg.get("near_side_ratio", 0.35))
    is_near_scene = (area_ratio >= near_area_ratio) or (side_ratio >= near_side_ratio)
    return is_near_scene, area_ratio, side_ratio


def _rect_from_contour(cnt, is_near_scene, approx_eps_ratio):
    peri = cv2.arcLength(cnt, True)
    if peri <= 0:
        return None, peri

    local_approx_eps_ratio = approx_eps_ratio * (1.4 if is_near_scene else 1.0)
    approx = cv2.approxPolyDP(cnt, local_approx_eps_ratio * peri, True)

    if is_near_scene:
        hull = cv2.convexHull(cnt)
        rect_local = cv2.minAreaRect(hull if len(hull) >= 3 else cnt)
    else:
        if len(approx) < 4 or len(approx) > 8:
            return None, peri
        if not cv2.isContourConvex(approx):
            return None, peri
        rect_local = cv2.minAreaRect(approx)
    return rect_local, peri


def _accept_candidate(rect_local, cnt, peri, roi_w, roi_h, cfg, is_near_scene, allow_edge_touch=False):
    ratio_min = float(cfg.get("ratio_min", 1.0))
    ratio_max = float(cfg.get("ratio_max", 2.5))
    min_short_side = float(cfg.get("min_short_side", 25))
    min_long_side = float(cfg.get("min_long_side", 40))
    max_side_ratio = float(cfg.get("max_side_ratio", 0.85))
    edge_margin = int(cfg.get("edge_margin", 4))
    target_ratio = float(cfg.get("target_ratio", 100.0 / 60.0))

    local_ratio_min = 0.50 if is_near_scene else ratio_min
    local_ratio_max = 4.50 if is_near_scene else ratio_max
    local_min_short_side = 10.0 if is_near_scene else min_short_side
    local_min_long_side = 16.0 if is_near_scene else min_long_side
    local_max_side_ratio = 1.02 if is_near_scene else max_side_ratio
    local_edge_margin = 0 if is_near_scene else edge_margin

    touches_edge = contour_touches_roi_edge(cnt, roi_w, roi_h, local_edge_margin)
    if touches_edge and not allow_edge_touch:
        return None

    short_side, long_side = rect_size_sorted(rect_local)
    if short_side <= 0 or long_side <= 0:
        return None
    if short_side < local_min_short_side or long_side < local_min_long_side:
        return None
    if long_side > max(roi_w, roi_h) * local_max_side_ratio:
        return None

    ratio = rect_ratio(rect_local)
    if not (local_ratio_min <= ratio <= local_ratio_max):
        return None

    (cx, cy), _, _ = rect_local
    roi_cx = roi_w / 2.0
    roi_cy = roi_h / 2.0
    center_dist = ((cx - roi_cx) ** 2 + (cy - roi_cy) ** 2) ** 0.5
    center_norm = center_dist / max(roi_w, roi_h)
    ratio_error = abs(ratio - target_ratio)

    edge_bonus = 0.15 if (allow_edge_touch and touches_edge) else 0.0
    near_bonus = 0.20 if is_near_scene else 0.0
    fill_ratio = (short_side * long_side) / float(max(1, roi_w * roi_h))
    fill_bonus = min(fill_ratio, 0.85)

    # 过滤掉几乎铺满 ROI 的大框，避免把 YOLO 粗框/外层大区域误当成白框
    local_max_fill_ratio = 0.92 if is_near_scene else 0.78
    if fill_ratio > local_max_fill_ratio:
        return None

    score = float(peri) / (1.0 + ratio_error * 2.0 + center_norm * 1.5)
    score *= (1.0 + edge_bonus + near_bonus + fill_bonus * 0.35)

    return {
        "rect": rect_local,
        "score": score,
        "ratio": ratio,
        "touches_edge": touches_edge,
        "is_near_scene": is_near_scene,
    }


def _find_inner_white_standard(contours, roi_w, roi_h, cfg):
    best = None
    candidate_count = 0
    approx_eps_ratio = float(cfg.get("approx_eps_ratio", 0.035))

    for cnt in contours:
        if len(cnt) < 4:
            continue
        is_near_scene, _, _ = _near_scene_params(cfg, roi_w, roi_h, cnt)
        rect_local, peri = _rect_from_contour(cnt, is_near_scene, approx_eps_ratio)
        if rect_local is None:
            continue
        accepted = _accept_candidate(rect_local, cnt, peri, roi_w, roi_h, cfg, is_near_scene, allow_edge_touch=False)
        if accepted is None:
            continue
        candidate_count += 1
        if best is None or accepted["score"] > best["score"]:
            best = accepted
    return best, candidate_count


def _find_inner_white_near_fallback(th, contours, roi_w, roi_h, cfg):
    """近端正对目标兜底：允许白框贴近 ROI 边缘，只要整体比例和矩形性仍合理。"""
    approx_eps_ratio = float(cfg.get("approx_eps_ratio", 0.035))
    candidates = []

    for cnt in contours:
        if len(cnt) < 4:
            continue
        is_near_scene, area_ratio, side_ratio = _near_scene_params(cfg, roi_w, roi_h, cnt)
        if not is_near_scene:
            continue
        rect_local, peri = _rect_from_contour(cnt, True, approx_eps_ratio)
        if rect_local is None:
            continue
        accepted = _accept_candidate(rect_local, cnt, peri, roi_w, roi_h, cfg, True, allow_edge_touch=True)
        if accepted is not None:
            accepted["path"] = "near_contour"
            accepted["area_ratio"] = area_ratio
            accepted["side_ratio"] = side_ratio
            candidates.append(accepted)

    if candidates:
        return max(candidates, key=lambda x: x["score"]), len(candidates)

    # 第二层兜底：把阈值图整体看成一个大白块，适配“近端正对时白框几乎铺满 ROI”场景。
    nz = cv2.findNonZero(th)
    if nz is None or len(nz) < 4:
        return None, 0

    rect_local = cv2.minAreaRect(nz)
    peri = cv2.arcLength(cv2.convexHull(nz), True)
    dummy_cnt = nz
    accepted = _accept_candidate(rect_local, dummy_cnt, peri, roi_w, roi_h, cfg, True, allow_edge_touch=True)
    if accepted is None:
        return None, 0
    accepted["path"] = "near_mask"
    return accepted, 1


def find_inner_white(img, cfg, roi=None):
    """
    检测内白框。

    API 保持不变，但内部改成两级检测：
    1. 常规路径：适合中远景，保留原有比例/中心/边长筛选；
    2. 近景兜底：允许目标更大、贴近 ROI 边缘，解决“近端正对目标物识别不到”。
    """
    roi_img, x0, y0, x1, y1 = get_roi(img, cfg, external_roi=roi)

    if roi_img is None or roi_img.size == 0:
        return None, (x0, y0, x1, y1)

    roi_h, roi_w = roi_img.shape[:2]
    gray = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)

    block_size = int(cfg.get("white_adaptive_block_size", 11))
    if block_size % 2 == 0:
        block_size += 1
    block_size = max(3, block_size)

    th = cv2.adaptiveThreshold(
        gray,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY,
        block_size,
        int(cfg.get("white_adaptive_c", 2)),
    )

    denoise_kernel = int(cfg.get("white_denoise_kernel", 0))
    if denoise_kernel and denoise_kernel > 1:
        if denoise_kernel % 2 == 0:
            denoise_kernel += 1
        kernel = np.ones((denoise_kernel, denoise_kernel), np.uint8)
        th = cv2.morphologyEx(th, cv2.MORPH_CLOSE, kernel)
        th = cv2.morphologyEx(th, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(th, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    if cfg.get("debug", False):
        dbg_dir = os.path.join(os.getcwd(), "debug_img")
        os.makedirs(dbg_dir, exist_ok=True)
        tag = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        cv2.imwrite(os.path.join(dbg_dir, f"{tag}_roi.png"), roi_img)
        cv2.imwrite(os.path.join(dbg_dir, f"{tag}_th.png"), th)
        contour_vis = cv2.cvtColor(th, cv2.COLOR_GRAY2BGR)
        cv2.drawContours(contour_vis, contours, -1, (0, 0, 255), 1)
        cv2.imwrite(os.path.join(dbg_dir, f"{tag}_contour.png"), contour_vis)

    best, candidate_count = _find_inner_white_standard(contours, roi_w, roi_h, cfg)
    detect_path = "standard"

    if best is None:
        best, near_candidate_count = _find_inner_white_near_fallback(th, contours, roi_w, roi_h, cfg)
        candidate_count += near_candidate_count
        detect_path = "near_fallback"

    if best is None:
        if cfg.get("debug", False):
            print(f"[CV] inner white not found, candidates={candidate_count}")
        return None, (x0, y0, x1, y1)

    inner_rect = rect_from_local_to_global(best["rect"], x0, y0)

    if cfg.get("debug", False):
        print(
            f"[CV] inner white found, candidates={candidate_count}, "
            f"ratio={rect_ratio(inner_rect):.3f}, path={detect_path}"
        )

    return inner_rect, (x0, y0, x1, y1)


# ================== 激光点检测 ==================
def _make_laser_masks(roi_img, cfg):
    """
    生成激光点候选 mask：
    - white_mask：激光中心，全白/过曝；
    - blue_mask：周围蓝色光晕。
    """
    hsv = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    b, g, r = cv2.split(roi_img)

    blue_h_min = int(cfg.get("laser_blue_h_min", 85))
    blue_h_max = int(cfg.get("laser_blue_h_max", 140))
    blue_s_min = int(cfg.get("laser_blue_s_min", 45))
    blue_v_min = int(cfg.get("laser_blue_v_min", 50))

    blue_mask = (
        (h >= blue_h_min)
        & (h <= blue_h_max)
        & (s >= blue_s_min)
        & (v >= blue_v_min)
    ).astype(np.uint8) * 255

    white_v_min = int(cfg.get("laser_white_v_min", 220))
    white_s_max = int(cfg.get("laser_white_s_max", 85))
    white_bgr_min = int(cfg.get("laser_white_bgr_min", 205))

    hsv_white = (v >= white_v_min) & (s <= white_s_max)
    bgr_white = (b >= white_bgr_min) & (g >= white_bgr_min) & (r >= white_bgr_min)
    white_mask = (hsv_white | bgr_white).astype(np.uint8) * 255

    ksize = int(cfg.get("laser_morph_kernel", 3))
    if ksize < 1:
        ksize = 1
    if ksize % 2 == 0:
        ksize += 1

    if ksize > 1:
        kernel = np.ones((ksize, ksize), dtype=np.uint8)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)

    return white_mask, blue_mask


def _circle_score(cnt):
    area = cv2.contourArea(cnt)
    peri = cv2.arcLength(cnt, True)
    if peri <= 0:
        return 0.0
    return float(4.0 * np.pi * area / (peri * peri))


def _laser_search_roi_from_frame_box(frame_box, used_roi, img_w, img_h, cfg):
    """
    激光点默认只在白框附近搜索，减少把背景蓝色误判成激光。
    如果没有白框 box，则退回本次 OpenCV 使用过的 ROI。
    
    注意：这个 ROI 只是为了减少计算量；真正防误检的关键是
    _make_inner_frame_mask() 生成的“白框内部 mask”。
    """
    margin = int(cfg.get("laser_search_margin", 20))

    if cfg.get("laser_search_in_frame_box", True) and frame_box is not None:
        box = np.asarray(frame_box, dtype=np.int32)
        x, y, w, h = cv2.boundingRect(box)
        return _expand_roi((x, y, x + w, y + h), img_w, img_h, margin)

    if used_roi is not None:
        return _expand_roi(used_roi, img_w, img_h, margin)

    return 0, 0, img_w, img_h


def _make_inner_frame_mask(roi_shape, frame_box, sx0, sy0, cfg):
    """
    生成“白框四边形内部”激光搜索 mask。

    目的：
    - 激光点只允许在绿色白框内部被检测；
    - 排除白框边缘附近一圈，避免把白框上边缘反光/紫色噪声误判为激光点。

    返回：
        uint8 mask，白色区域为允许检测区域；如果不启用严格内部检测，返回 None。
    """
    if not cfg.get("laser_strict_inside_frame", True):
        return None

    if frame_box is None:
        return None

    roi_h, roi_w = roi_shape[:2]
    if roi_h <= 0 or roi_w <= 0:
        return None

    box = np.asarray(frame_box, dtype=np.float32).copy()
    if box.shape[0] < 3:
        return None

    # 全局坐标转为当前 laser ROI 内的局部坐标。
    box[:, 0] -= float(sx0)
    box[:, 1] -= float(sy0)
    box = np.round(box).astype(np.int32)

    poly_mask = np.zeros((roi_h, roi_w), dtype=np.uint8)
    cv2.fillConvexPoly(poly_mask, box, 255)

    inner_margin = int(cfg.get("laser_inner_margin_px", 12))
    if inner_margin <= 0:
        return poly_mask

    # distanceTransform 比固定 kernel erode 更稳定：任何距离白框边缘小于 inner_margin
    # 的点都会被排除。
    dist = cv2.distanceTransform(poly_mask, cv2.DIST_L2, 5)
    inner_mask = (dist >= float(inner_margin)).astype(np.uint8) * 255

    # 如果白框太小导致腐蚀后空了，则退回未腐蚀的四边形 mask，避免完全失效。
    if cv2.countNonZero(inner_mask) <= 0:
        return poly_mask

    return inner_mask


def _apply_laser_inner_mask(white_mask, blue_mask, inner_mask, cfg):
    """
    将白芯/蓝光晕 mask 限制到白框内部。
    """
    if inner_mask is None:
        return white_mask, blue_mask

    white_mask = cv2.bitwise_and(white_mask, inner_mask)

    # 默认也限制蓝色光晕。这样可以强制“白芯和光晕都在白框内部”，
    # 有效排除白框外/边缘上的蓝紫色反光。
    if cfg.get("laser_mask_blue_inside_frame", True):
        blue_mask = cv2.bitwise_and(blue_mask, inner_mask)

    return white_mask, blue_mask


def find_laser_point(img, cfg, frame_box=None, used_roi=None):
    """
    检测激光点：内部全白/过曝，周围有蓝色光晕。

    返回：
        (laser_center, laser_info)
    其中 laser_center 为全局图像坐标 (x, y)，未检测到则为 None。
    """
    if img is None or not cfg.get("laser_detect_enable", True):
        return None, None

    img_h, img_w = img.shape[:2]
    sx0, sy0, sx1, sy1 = _laser_search_roi_from_frame_box(frame_box, used_roi, img_w, img_h, cfg)
    roi_img = img[sy0:sy1, sx0:sx1]

    if roi_img is None or roi_img.size == 0:
        return None, None

    white_mask, blue_mask = _make_laser_masks(roi_img, cfg)

    inner_mask = _make_inner_frame_mask(roi_img.shape[:2], frame_box, sx0, sy0, cfg)
    white_mask, blue_mask = _apply_laser_inner_mask(white_mask, blue_mask, inner_mask, cfg)
    inner_mask_applied = inner_mask is not None

    contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    min_white_area = float(cfg.get("laser_min_white_area", 2.0))
    max_white_area = float(cfg.get("laser_max_white_area", 650.0))
    min_core_side = float(cfg.get("laser_min_core_side", 2.0))
    max_core_side = float(cfg.get("laser_max_core_side", 36.0))
    min_circularity = float(cfg.get("laser_min_circularity", 0.20))
    min_blue_pixels = int(cfg.get("laser_min_blue_pixels", 8))
    min_blue_ratio = float(cfg.get("laser_min_blue_ratio", 0.010))
    ring_expand = int(cfg.get("laser_ring_expand", 18))

    best = None
    best_score = -1.0

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_white_area or area > max_white_area:
            continue

        x, y, w, h = cv2.boundingRect(cnt)
        if w < min_core_side or h < min_core_side:
            continue
        if w > max_core_side or h > max_core_side:
            continue

        circularity = _circle_score(cnt)
        if circularity < min_circularity:
            continue

        m = cv2.moments(cnt)
        if abs(m["m00"]) < 1e-6:
            cx = x + w / 2.0
            cy = y + h / 2.0
        else:
            cx = m["m10"] / m["m00"]
            cy = m["m01"] / m["m00"]

        # 检查白芯周围是否存在蓝色光晕。
        rx0 = max(0, int(x - ring_expand))
        ry0 = max(0, int(y - ring_expand))
        rx1 = min(roi_img.shape[1], int(x + w + ring_expand))
        ry1 = min(roi_img.shape[0], int(y + h + ring_expand))

        if rx1 <= rx0 or ry1 <= ry0:
            continue

        blue_patch = blue_mask[ry0:ry1, rx0:rx1]
        yy, xx = np.ogrid[ry0:ry1, rx0:rx1]
        dist = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2)

        inner_r = max(1.0, 0.5 * max(w, h) + 1.0)
        outer_r = max(inner_r + 2.0, inner_r + float(ring_expand))
        ring_mask = (dist >= inner_r) & (dist <= outer_r)

        ring_total = int(np.count_nonzero(ring_mask))
        if ring_total <= 0:
            continue

        blue_count = int(np.count_nonzero((blue_patch > 0) & ring_mask))
        blue_ratio = blue_count / float(ring_total)

        if blue_count < min_blue_pixels or blue_ratio < min_blue_ratio:
            continue

        # 得分越高越可信：白芯面积、蓝光晕数量、圆度越高越好。
        score = float(area) * 2.0 + float(blue_count) * 0.8 + circularity * 50.0

        if score > best_score:
            best_score = score
            best = {
                "center": (int(round(cx + sx0)), int(round(cy + sy0))),
                "core_bbox": (int(x + sx0), int(y + sy0), int(x + w + sx0), int(y + h + sy0)),
                "search_roi": (sx0, sy0, sx1, sy1),
                "strict_inside_frame": bool(cfg.get("laser_strict_inside_frame", True)),
                "inner_mask_applied": bool(inner_mask_applied),
                "inner_margin_px": int(cfg.get("laser_inner_margin_px", 12)),
                "white_area": float(area),
                "blue_count": blue_count,
                "blue_ratio": float(blue_ratio),
                "circularity": float(circularity),
                "score": float(score),
                "method": "white_core_blue_halo",
            }

    if best is not None:
        if cfg.get("laser_debug", False):
            print(
                "[LASER_DETECT] found "
                f"center={best['center']}, white_area={best['white_area']:.1f}, "
                f"blue_count={best['blue_count']}, blue_ratio={best['blue_ratio']:.3f}, "
                f"score={best['score']:.1f}"
            )
        return best["center"], best

    # 兜底 1：如果白芯 mask 因曝光/色彩导致不稳定，可用蓝色光晕轮廓再反查中心白亮区域。
    if cfg.get("laser_allow_blue_fallback", True):
        fallback_center, fallback_info = _find_laser_by_blue_halo(roi_img, white_mask, blue_mask, sx0, sy0, cfg)
        if fallback_center is not None:
            return fallback_center, fallback_info

    # 兜底 2：真实激光点在白框内部，但蓝色光晕太弱时，只用“小面积高亮白芯”判断。
    white_center, white_info = _find_laser_by_white_core_only(roi_img, white_mask, sx0, sy0, cfg)
    if white_center is not None:
        return white_center, white_info

    if cfg.get("laser_debug", False):
        print("[LASER_DETECT] not found")

    return None, {
        "search_roi": (sx0, sy0, sx1, sy1),
        "strict_inside_frame": bool(cfg.get("laser_strict_inside_frame", True)),
        "inner_mask_applied": bool(inner_mask_applied),
        "inner_margin_px": int(cfg.get("laser_inner_margin_px", 12)),
        "method": "not_found",
    }


def _find_laser_by_blue_halo(roi_img, white_mask, blue_mask, ox, oy, cfg):
    contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    min_blue_area = float(cfg.get("laser_min_blue_area", 6.0))
    max_blue_area = float(cfg.get("laser_max_blue_area", 2500.0))
    max_halo_side = float(cfg.get("laser_max_halo_side", 80.0))
    min_circularity = float(cfg.get("laser_blue_min_circularity", 0.08))
    min_white_pixels = int(cfg.get("laser_fallback_min_white_pixels", 2))

    best = None
    best_score = -1.0

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_blue_area or area > max_blue_area:
            continue

        x, y, w, h = cv2.boundingRect(cnt)
        if w > max_halo_side or h > max_halo_side:
            continue

        circularity = _circle_score(cnt)
        if circularity < min_circularity:
            continue

        x0 = max(0, x - 3)
        y0 = max(0, y - 3)
        x1 = min(roi_img.shape[1], x + w + 3)
        y1 = min(roi_img.shape[0], y + h + 3)

        white_patch = white_mask[y0:y1, x0:x1]
        white_pixels = int(np.count_nonzero(white_patch > 0))
        if white_pixels < min_white_pixels:
            continue

        # 用白亮像素质心作为激光点；没有时才用蓝色轮廓中心。
        ys, xs = np.where(white_patch > 0)
        if len(xs) > 0:
            cx = float(xs.mean() + x0)
            cy = float(ys.mean() + y0)
        else:
            m = cv2.moments(cnt)
            if abs(m["m00"]) < 1e-6:
                cx = x + w / 2.0
                cy = y + h / 2.0
            else:
                cx = m["m10"] / m["m00"]
                cy = m["m01"] / m["m00"]

        score = float(area) + white_pixels * 2.0 + circularity * 30.0

        if score > best_score:
            best_score = score
            best = {
                "center": (int(round(cx + ox)), int(round(cy + oy))),
                "core_bbox": (int(x0 + ox), int(y0 + oy), int(x1 + ox), int(y1 + oy)),
                "search_roi": (int(ox), int(oy), int(ox + roi_img.shape[1]), int(oy + roi_img.shape[0])),
                "strict_inside_frame": bool(cfg.get("laser_strict_inside_frame", True)),
                "inner_margin_px": int(cfg.get("laser_inner_margin_px", 12)),
                "blue_area": float(area),
                "white_pixels": white_pixels,
                "circularity": float(circularity),
                "score": float(score),
                "method": "blue_halo_fallback",
            }

    if best is None:
        return None, None

    if cfg.get("laser_debug", False):
        print(
            "[LASER_DETECT] fallback found "
            f"center={best['center']}, blue_area={best['blue_area']:.1f}, "
            f"white_pixels={best['white_pixels']}, score={best['score']:.1f}"
        )

    return best["center"], best


def _find_laser_by_white_core_only(roi_img, white_mask, ox, oy, cfg):
    """
    兜底检测：只在白框内部找小面积高亮白芯。

    使用场景：真实激光点已经在白框内部，但蓝色光晕很弱，导致“白芯+蓝环”规则不通过。
    为了降低误检，仍然要求：
    - 候选必须在白框内部 mask 之后；
    - 面积和宽高较小；
    - 圆度不能太差；
    - 局部亮度明显高。
    """
    if not cfg.get("laser_allow_white_core_fallback", True):
        return None, None

    if roi_img is None or roi_img.size == 0 or white_mask is None:
        return None, None

    hsv = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    b, g, r = cv2.split(roi_img)

    v_min = int(cfg.get("laser_white_only_v_min", 218))
    bgr_min = int(cfg.get("laser_white_only_bgr_min", 190))
    s_max = int(cfg.get("laser_white_only_s_max", 160))

    strict_white = (
        ((v >= v_min) & (s <= s_max))
        | ((b >= bgr_min) & (g >= bgr_min) & (r >= bgr_min))
    ).astype(np.uint8) * 255

    # 继续沿用已被白框内部 mask 限制过的 white_mask，避免背景高亮参与。
    strict_white = cv2.bitwise_and(strict_white, white_mask)

    ksize = int(cfg.get("laser_white_only_morph_kernel", 1))
    if ksize > 1:
        if ksize % 2 == 0:
            ksize += 1
        kernel = np.ones((ksize, ksize), dtype=np.uint8)
        strict_white = cv2.morphologyEx(strict_white, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(strict_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    min_area = float(cfg.get("laser_white_only_min_area", 1.0))
    max_area = float(cfg.get("laser_white_only_max_area", 120.0))
    min_side = float(cfg.get("laser_white_only_min_side", 1.0))
    max_side = float(cfg.get("laser_white_only_max_side", 20.0))
    min_circularity = float(cfg.get("laser_white_only_min_circularity", 0.08))

    best = None
    best_score = -1.0

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area or area > max_area:
            continue

        x, y, w, hh = cv2.boundingRect(cnt)
        if w < min_side or hh < min_side:
            continue
        if w > max_side or hh > max_side:
            continue

        circularity = _circle_score(cnt)
        if circularity < min_circularity:
            continue

        m = cv2.moments(cnt)
        if abs(m["m00"]) < 1e-6:
            cx = x + w / 2.0
            cy = y + hh / 2.0
        else:
            cx = m["m10"] / m["m00"]
            cy = m["m01"] / m["m00"]

        patch_v = v[y:y + hh, x:x + w]
        mean_v = float(np.mean(patch_v)) if patch_v.size else 0.0
        max_v = float(np.max(patch_v)) if patch_v.size else 0.0

        # 局部背景对比度：激光点应明显比周围亮。
        pad = int(cfg.get("laser_white_only_contrast_pad", 10))
        x0 = max(0, x - pad)
        y0 = max(0, y - pad)
        x1 = min(v.shape[1], x + w + pad)
        y1 = min(v.shape[0], y + hh + pad)
        local = v[y0:y1, x0:x1]
        local_mean = float(np.mean(local)) if local.size else mean_v
        contrast = max_v - local_mean
        min_contrast = float(cfg.get("laser_white_only_min_contrast", 8.0))
        if contrast < min_contrast:
            continue

        score = area * 2.0 + circularity * 30.0 + max_v * 0.15 + contrast * 2.0
        if score > best_score:
            best_score = score
            best = {
                "center": (int(round(cx + ox)), int(round(cy + oy))),
                "core_bbox": (int(x + ox), int(y + oy), int(x + w + ox), int(y + hh + oy)),
                "search_roi": (int(ox), int(oy), int(ox + roi_img.shape[1]), int(oy + roi_img.shape[0])),
                "strict_inside_frame": bool(cfg.get("laser_strict_inside_frame", True)),
                "inner_margin_px": int(cfg.get("laser_inner_margin_px", 12)),
                "white_area": float(area),
                "circularity": float(circularity),
                "mean_v": mean_v,
                "max_v": max_v,
                "contrast": float(contrast),
                "score": float(score),
                "method": "white_core_inside_fallback",
            }

    if best is None:
        return None, None

    if cfg.get("laser_debug", False):
        print(
            "[LASER_DETECT] white-core fallback found "
            f"center={best['center']}, area={best['white_area']:.1f}, "
            f"max_v={best['max_v']:.1f}, contrast={best['contrast']:.1f}, "
            f"score={best['score']:.1f}"
        )

    return best["center"], best


# ================== 透视变换与 scale ==================
def warp_perspective(img, rect):
    if rect is None:
        return None, 0, 0

    box = cv2.boxPoints(rect).astype(np.float32)
    rect_pts = order_points(box)

    w = int(rect[1][0])
    h = int(rect[1][1])

    if w <= 0 or h <= 0:
        return None, 0, 0

    if w > h:
        w, h = h, w

    dst = np.array(
        [
            [0, 0],
            [w - 1, 0],
            [w - 1, h - 1],
            [0, h - 1],
        ],
        dtype=np.float32,
    )

    M = cv2.getPerspectiveTransform(rect_pts, dst)
    warped = cv2.warpPerspective(img, M, (w, h))

    return warped, w, h


def _box_center(box):
    if box is None:
        return None
    return int(sum(pt[0] for pt in box) / 4), int(sum(pt[1] for pt in box) / 4)


# ================== 可视化 ==================
def visualize(frame, result):
    """
    精简显示，只保留：
    - YOLO 检测/粗定位框；
    - OpenCV 白框；
    - 白框中心点；
    - 激光点；
    - frame/laser 状态。

    FPS 在 main.draw_overlay() 中叠加。
    """
    vis = frame.copy()

    # YOLO 粗定位框：只在确实传入 YOLO ROI 时画，不再画默认全局 CV ROI。
    yolo_roi = result.get("yolo_roi")
    if yolo_roi is not None:
        x1, y1, x2, y2 = map(int, yolo_roi)
        cv2.rectangle(vis, (x1, y1), (x2, y2), (255, 160, 0), 1)

    box = result.get("frame_box")
    white_center = result.get("white_frame_center", result.get("frame_center"))

    # 白框：绿色轮廓。
    if box is not None:
        cv2.drawContours(vis, [box.astype(np.int32)], 0, (0, 255, 0), 2)

    # 白框中心点：黄色实心点。
    if white_center is not None:
        wcx, wcy = white_center
        cv2.circle(vis, (int(wcx), int(wcy)), 4, (0, 255, 255), -1)

    # 激光点：紫色圆 + 十字。
    if result.get("laser_found", False) and laser_center is not None:
        lx, ly = map(int, laser_center)
        cv2.circle(vis, (lx, ly), 7, (255, 0, 255), 2)
        cv2.drawMarker(
            vis,
            (lx, ly),
            (255, 0, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=14,
            thickness=2,
        )

    frame_status = "OK" if result.get("frame_found", False) else "LOST"
    cv2.putText(
        vis,
        f"frame:{frame_status}",
        (10, vis.shape[0] - 14),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2,
    )

    return vis


# ================== 主流程 ==================
def process_frame(frame, cfg, task_cfg=None, roi=None):
    """
    主处理流程：
    1. 可选镜头畸变修正；
    2. 检测内白框，计算白框中心和 scale；
    3. 近景时自动启用更宽松的白框兜底检测，缓解“近端正对目标识别不到”；
    4. 返回白框中心，最终 Ax/Ay 由上层按图像中心与白框中心的像素差乘真实比例计算。
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
            "status": "frame:LOST laser:SKIP",
        }, frame

    corrected = undistort_frame(frame, cfg)

    inner_rect, used_roi = find_inner_white(corrected, cfg, roi=roi)

    if inner_rect is None:
        result = {
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
            "scale_w": 1.0,
            "scale_h": 1.0,
            "roi": used_roi,
            "yolo_roi": roi,
            "undistorted": corrected,
            "status": "frame:LOST",
            "mode": "inner_white_reference",
        }
        vis = visualize(corrected, result) if cfg.get("draw", True) else corrected
        return result, vis

    warped, w2, h2 = warp_perspective(corrected, inner_rect)
    if warped is None or w2 <= 0 or h2 <= 0:
        result = {
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
            "scale_w": 1.0,
            "scale_h": 1.0,
            "roi": used_roi,
            "yolo_roi": roi,
            "undistorted": corrected,
            "status": "frame:LOST",
            "mode": "inner_white_reference",
        }
        vis = visualize(corrected, result) if cfg.get("draw", True) else corrected
        return result, vis

    short_side = min(w2, h2)
    long_side = max(w2, h2)

    if task_cfg is None:
        real_w = float(cfg.get("real_width", short_side))
        real_h = float(cfg.get("real_height", long_side))
    else:
        real_w = float(task_cfg.get("real_width", short_side))
        real_h = float(task_cfg.get("real_height", long_side))

    scale_w = real_w / float(short_side)
    scale_h = real_h / float(long_side)
    scale = (scale_w + scale_h) / 2.0

    box = cv2.boxPoints(inner_rect).astype(np.int32)
    white_center = _box_center(box)

    result = {
        "frame_found": True,

        # 当前版本只使用白框几何中心。
        "frame_center": white_center,
        "white_frame_center": white_center,
        "reference_source": "white_frame_center",
        "center_source": "white_frame_center",
        "frame_box": box,

        # 兼容 RDK_25_E 旧字段。
        "box": box,
        "white_box": box,
        "inner_box": box,
        "black_box": None,
        "outer_box": None,
        "inner_rect": inner_rect,
        "white_rect": inner_rect,
        "warped": warped,

        "scale": scale,
        "scale_w": scale_w,
        "scale_h": scale_h,
        "w": w2,
        "h": h2,
        "roi": used_roi,
        "yolo_roi": roi,
        "undistorted": corrected,
        "status": "frame:OK",
        "mode": "inner_white_reference",
    }

    vis = visualize(corrected, result) if cfg.get("draw", True) else corrected
    return result, vis
