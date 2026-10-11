# ================== vision/YoloModel.py ==================
"""
RDK X5 YOLO 粗定位模块

适配模型：
- YOLOv8 / YOLO11 / YOLOv5u 等 DFL 输出结构
- 典型 blank.bin 输出：
    Output[0] = (1, 64, 80, 80)  bbox, stride=8
    Output[1] = (1, 1, 80, 80)   cls,  stride=8
    Output[2] = (1, 64, 40, 40)  bbox, stride=16
    Output[3] = (1, 1, 40, 40)   cls,  stride=16
    Output[4] = (1, 64, 20, 20)  bbox, stride=32
    Output[5] = (1, 1, 20, 20)   cls,  stride=32

作用：
- YOLO 只做白框/目标的大致定位 ROI；
- OpenCV 继续在 ROI 内精确检测白框中心；
- 本文件不替代 CVModel.py 的白框几何中心计算。
"""

import os
import cv2
import numpy as np


try:
    from hobot_dnn import pyeasy_dnn as dnn
except Exception:
    # 在 Windows/Pylance/非 RDK 环境下没有 hobot_dnn 是正常的。
    dnn = None


def sigmoid(x):
    """稳定 sigmoid，避免 exp 溢出。"""
    return 1.0 / (1.0 + np.exp(-np.clip(x, -50, 50)))


class YoloModel:
    """
    RDK X5 YOLO DFL 后处理模型。

    公开接口：
        detect(frame) -> boxes, scores, classes
        get_roi(frame, return_info=False) -> roi 或 (roi, info)
        draw(frame, boxes, scores, classes, roi=None) -> frame
    """

    def __init__(
        self,
        model_path,
        conf_thresh=0.35,
        nms_thresh=0.50,
        class_names=("frame",),
        roi_margin=25,
        min_area=300,
        max_area_ratio=0.90,
        min_wh=8,
        reg_max=16,
        debug=False,
        enabled=True,
    ):
        self.model_path = str(model_path)
        self.conf_thresh = float(conf_thresh)
        self.nms_thresh = float(nms_thresh)
        self.class_names = list(class_names)
        self.num_classes = max(1, len(self.class_names))

        self.roi_margin = int(roi_margin)
        self.min_area = float(min_area)
        self.max_area_ratio = float(max_area_ratio)
        self.min_wh = int(min_wh)
        self.reg_max = int(reg_max)

        self.debug = bool(debug)
        self.enabled = bool(enabled)

        self.model = None
        self.input_h = None
        self.input_w = None
        self.input_format = None
        self.heads = []

        if not self.enabled:
            self._log("[YOLO] disabled")
            return

        if dnn is None:
            raise ImportError(
                "未找到 hobot_dnn.pyeasy_dnn。请在 RDK X5 板子环境运行，"
                "Windows/Pylance 下提示未知导入通常可以忽略。"
            )

        if not os.path.exists(self.model_path):
            raise FileNotFoundError(f"YOLO 模型不存在: {self.model_path}")

        self._load_model()
        self._parse_input_shape()
        self.heads = self._parse_heads()
        self._build_grids()

        if len(self.heads) == 0:
            raise RuntimeError(
                "YOLO 输出头解析失败：未找到 bbox/cls 配对输出。"
                "请确认模型是否为 6 输出 DFL 格式，例如 (64,80,80)+(1,80,80)。"
            )

        self._log(
            f"[YOLO] loaded model={self.model_path}, "
            f"input={self.input_w}x{self.input_h}, "
            f"classes={self.class_names}, heads="
            f"{[(h['stride'], h['hw'], h['bbox_fmt'], h['cls_fmt']) for h in self.heads]}"
        )

    # ======================================================
    # 基础工具
    # ======================================================

    def _log(self, msg):
        if self.debug:
            print(msg)

    def _load_model(self):
        models = dnn.load(self.model_path)
        if not models:
            raise RuntimeError(f"YOLO 模型加载失败: {self.model_path}")
        self.model = models[0]

    def _parse_input_shape(self):
        """
        自动解析输入 shape，兼容 NCHW / NHWC。
        RDK 转换后的模型虽然通常输入 NV12，但 properties.shape 仍可用于拿到 H/W。
        """
        in_shape = list(self.model.inputs[0].properties.shape)
        if len(in_shape) != 4:
            raise RuntimeError(f"不支持的模型输入 shape: {in_shape}")

        if in_shape[1] == 3:  # NCHW: [1, 3, H, W]
            self.input_h = int(in_shape[2])
            self.input_w = int(in_shape[3])
            self.input_format = "NCHW"
        elif in_shape[-1] == 3:  # NHWC: [1, H, W, 3]
            self.input_h = int(in_shape[1])
            self.input_w = int(in_shape[2])
            self.input_format = "NHWC"
        else:
            # 某些 NV12 模型可能显示为 [1, H*3/2, W, 1] 或类似格式。
            # 当前 blank.bin 根据你的输出应为标准 640 输入；这里保留明确报错，避免错误推理。
            raise RuntimeError(f"无法识别 YOLO 输入格式: {in_shape}")

        self._log(f"[YOLO] input shape={in_shape}, format={self.input_format}")

    def _parse_heads(self):
        """
        自动匹配 bbox 输出和 cls 输出。

        支持：
            bbox NCHW: [1, 64, H, W]
            cls  NCHW: [1, 1,  H, W]
            bbox NHWC: [1, H, W, 64]
            cls  NHWC: [1, H, W, 1]
        """
        bbox_map = {}
        cls_map = {}
        bbox_c = 4 * self.reg_max

        for idx, out in enumerate(self.model.outputs):
            shp = list(out.properties.shape)
            layout = getattr(out.properties, "layout", "")
            self._log(f"[YOLO] output[{idx}] shape={shp}, layout={layout}")

            if len(shp) != 4:
                continue

            # NCHW: [N, C, H, W]
            if shp[1] == bbox_c:
                H, W = int(shp[2]), int(shp[3])
                bbox_map[(H, W)] = (idx, "NCHW")
                continue

            if shp[1] in (1, self.num_classes) and shp[2] > 1 and shp[3] > 1:
                H, W = int(shp[2]), int(shp[3])
                cls_map[(H, W)] = (idx, "NCHW", int(shp[1]))
                continue

            # NHWC: [N, H, W, C]
            if shp[-1] == bbox_c:
                H, W = int(shp[1]), int(shp[2])
                bbox_map[(H, W)] = (idx, "NHWC")
                continue

            if shp[-1] in (1, self.num_classes) and shp[1] > 1 and shp[2] > 1:
                H, W = int(shp[1]), int(shp[2])
                cls_map[(H, W)] = (idx, "NHWC", int(shp[-1]))
                continue

        heads = []
        # 从大特征图到小特征图排序：80x80, 40x40, 20x20
        for hw in sorted(bbox_map.keys(), key=lambda x: -x[0]):
            if hw not in cls_map:
                self._log(f"[YOLO] warning: bbox head {hw} 没有匹配 cls head")
                continue

            H, W = hw
            bbox_idx, bbox_fmt = bbox_map[hw]
            cls_idx, cls_fmt, cls_c = cls_map[hw]

            stride_y = self.input_h / float(H)
            stride_x = self.input_w / float(W)
            if abs(stride_x - stride_y) < 1e-6:
                stride = float(stride_x)
            else:
                stride = (float(stride_x), float(stride_y))

            heads.append(
                {
                    "hw": hw,
                    "bbox_idx": bbox_idx,
                    "cls_idx": cls_idx,
                    "bbox_fmt": bbox_fmt,
                    "cls_fmt": cls_fmt,
                    "cls_c": cls_c,
                    "stride_x": float(stride_x),
                    "stride_y": float(stride_y),
                    "stride": stride,
                    "grid": None,
                }
            )

        return heads

    def _build_grids(self):
        for h in self.heads:
            H, W = h["hw"]
            gy, gx = np.meshgrid(
                np.arange(H, dtype=np.float32),
                np.arange(W, dtype=np.float32),
                indexing="ij",
            )
            h["grid"] = np.stack([gx, gy], axis=-1).reshape(-1, 2).astype(np.float32)

    # ======================================================
    # 预处理：BGR -> letterbox -> NV12
    # ======================================================

    def bgr_to_nv12(self, frame):
        """
        BGR 图像转 NV12，并保持 letterbox 信息用于坐标还原。

        返回：
            nv12:      shape = [input_h * 3 / 2, input_w]
            scale:     原图 -> 模型输入的缩放比例
            pad_left:  letterbox 左侧 padding
            pad_top:   letterbox 上侧 padding
        """
        if frame is None or frame.size == 0:
            raise ValueError("empty frame")

        orig_h, orig_w = frame.shape[:2]
        scale = min(self.input_h / float(orig_h), self.input_w / float(orig_w))
        new_w = int(round(orig_w * scale))
        new_h = int(round(orig_h * scale))

        resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        canvas = np.full((self.input_h, self.input_w, 3), 114, dtype=np.uint8)
        pad_left = (self.input_w - new_w) // 2
        pad_top = (self.input_h - new_h) // 2
        canvas[pad_top:pad_top + new_h, pad_left:pad_left + new_w] = resized

        # BGR -> YUV_I420 -> NV12。RDK pyeasy_dnn 的常见 BPU 输入为 NV12。
        yuv_i420 = cv2.cvtColor(canvas, cv2.COLOR_BGR2YUV_I420)
        y = yuv_i420[:self.input_h, :]

        u = yuv_i420[
            self.input_h:self.input_h + self.input_h // 4,
            :,
        ].reshape(self.input_h // 2, self.input_w // 2)

        v = yuv_i420[
            self.input_h + self.input_h // 4:,
            :,
        ].reshape(self.input_h // 2, self.input_w // 2)

        uv = np.stack([u, v], axis=-1).reshape(self.input_h // 2, self.input_w)
        nv12 = np.concatenate([y, uv], axis=0)
        return np.ascontiguousarray(nv12), float(scale), int(pad_left), int(pad_top)

    # ======================================================
    # 后处理：DFL + 坐标还原 + NMS
    # ======================================================

    def dfl_decode(self, bbox_raw):
        """
        DFL decoding: [N, 4 * reg_max] -> [N, 4]
        输出顺序为 left, top, right, bottom。
        """
        bbox = bbox_raw.reshape(-1, 4, self.reg_max).astype(np.float32)
        bbox_exp = np.exp(bbox - np.max(bbox, axis=-1, keepdims=True))
        bbox_prob = bbox_exp / np.sum(bbox_exp, axis=-1, keepdims=True)
        weights = np.arange(self.reg_max, dtype=np.float32).reshape(1, 1, -1)
        return np.sum(bbox_prob * weights, axis=-1)

    def _extract_feat(self, outs, idx, fmt, H, W, C):
        """
        从 pyeasy_dnn 输出中提取特征，并统一为 [H*W, C]。
        """
        buf = np.array(outs[idx].buffer, copy=False).astype(np.float32)

        if fmt == "NCHW":
            feat = buf.reshape(1, C, H, W)
            feat = feat.transpose(0, 2, 3, 1).reshape(-1, C)
        elif fmt == "NHWC":
            feat = buf.reshape(1, H, W, C).reshape(-1, C)
        else:
            raise RuntimeError(f"未知输出格式: {fmt}")

        return feat

    def _empty_result(self):
        return (
            np.empty((0, 4), dtype=np.float32),
            np.empty((0,), dtype=np.float32),
            np.empty((0,), dtype=np.int32),
        )

    def _filter_boxes(self, boxes, scores, classes, frame_w, frame_h):
        """宽高和面积过滤，减少明显误检。"""
        if len(boxes) == 0:
            return boxes, scores, classes

        max_area = float(frame_w * frame_h) * self.max_area_ratio
        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 2]
        y2 = boxes[:, 3]

        w = x2 - x1
        h = y2 - y1
        area = w * h

        keep = (
            (w >= self.min_wh)
            & (h >= self.min_wh)
            & (area >= self.min_area)
            & (area <= max_area)
            & (x2 > x1)
            & (y2 > y1)
        )

        if not np.any(keep):
            return self._empty_result()

        return boxes[keep], scores[keep], classes[keep]

    def detect(self, frame):
        """
        对整张图做 YOLO 粗检测。

        返回：
            boxes:   [N, 4]，xyxy，原图坐标
            scores:  [N]
            classes: [N]
        """
        if not self.enabled or self.model is None:
            return self._empty_result()
        if frame is None or frame.size == 0:
            return self._empty_result()

        orig_h, orig_w = frame.shape[:2]
        nv12, scale, pad_left, pad_top = self.bgr_to_nv12(frame)
        outs = self.model.forward(nv12)

        all_boxes = []
        all_scores = []
        all_classes = []

        for h in self.heads:
            H, W = h["hw"]
            grid = h["grid"]
            stride_x = h["stride_x"]
            stride_y = h["stride_y"]

            bbox_feat = self._extract_feat(
                outs,
                h["bbox_idx"],
                h["bbox_fmt"],
                H,
                W,
                4 * self.reg_max,
            )

            cls_feat = self._extract_feat(
                outs,
                h["cls_idx"],
                h["cls_fmt"],
                H,
                W,
                h.get("cls_c", self.num_classes),
            )

            scores_all = sigmoid(cls_feat)
            max_scores = np.max(scores_all, axis=1)
            max_classes = np.argmax(scores_all, axis=1).astype(np.int32)

            keep = max_scores >= self.conf_thresh
            if not np.any(keep):
                continue

            bbox_keep = bbox_feat[keep]
            score_keep = max_scores[keep].astype(np.float32)
            class_keep = max_classes[keep]
            grid_keep = grid[keep]

            ltrb = self.dfl_decode(bbox_keep)

            # 网格中心点，位于模型输入坐标系。
            xc = (grid_keep[:, 0] + 0.5) * stride_x
            yc = (grid_keep[:, 1] + 0.5) * stride_y

            # LTRB -> xyxy，仍在模型输入坐标系。
            x1 = xc - ltrb[:, 0] * stride_x
            y1 = yc - ltrb[:, 1] * stride_y
            x2 = xc + ltrb[:, 2] * stride_x
            y2 = yc + ltrb[:, 3] * stride_y

            # 去 letterbox padding，映射回原图坐标。
            x1 = (x1 - pad_left) / scale
            y1 = (y1 - pad_top) / scale
            x2 = (x2 - pad_left) / scale
            y2 = (y2 - pad_top) / scale

            x1 = np.clip(x1, 0, orig_w - 1)
            y1 = np.clip(y1, 0, orig_h - 1)
            x2 = np.clip(x2, 0, orig_w - 1)
            y2 = np.clip(y2, 0, orig_h - 1)

            boxes = np.stack([x1, y1, x2, y2], axis=1).astype(np.float32)

            all_boxes.append(boxes)
            all_scores.append(score_keep)
            all_classes.append(class_keep)

        if len(all_boxes) == 0:
            return self._empty_result()

        boxes = np.concatenate(all_boxes, axis=0)
        scores = np.concatenate(all_scores, axis=0)
        classes = np.concatenate(all_classes, axis=0)

        boxes, scores, classes = self._filter_boxes(boxes, scores, classes, orig_w, orig_h)
        if len(boxes) == 0:
            return self._empty_result()

        # cv2.dnn.NMSBoxes 需要 xywh。这里不能直接传 xyxy。
        nms_boxes = []
        for box in boxes:
            x1, y1, x2, y2 = box
            nms_boxes.append([float(x1), float(y1), float(x2 - x1), float(y2 - y1)])

        idxs = cv2.dnn.NMSBoxes(
            nms_boxes,
            scores.astype(float).tolist(),
            self.conf_thresh,
            self.nms_thresh,
        )

        if len(idxs) == 0:
            return self._empty_result()

        idxs = np.array(idxs).reshape(-1)
        return boxes[idxs], scores[idxs], classes[idxs]

    # ======================================================
    # ROI 粗定位接口
    # ======================================================

    def get_best_box(self, boxes, scores, classes):
        """选择最适合作为 OpenCV 精检测 ROI 的 YOLO 框。"""
        if len(boxes) == 0:
            return None

        # 默认选择置信度最高的框。后续如果多类别，可在这里增加类别优先级。
        best_idx = int(np.argmax(scores))
        return {
            "box": boxes[best_idx],
            "score": float(scores[best_idx]),
            "class_id": int(classes[best_idx]),
        }

    def expand_box(self, box, frame_w, frame_h):
        """扩大 YOLO 框，防止 ROI 裁剪过紧。"""
        x1, y1, x2, y2 = box
        margin = self.roi_margin

        x1 = int(max(0, np.floor(x1 - margin)))
        y1 = int(max(0, np.floor(y1 - margin)))
        x2 = int(min(frame_w, np.ceil(x2 + margin)))
        y2 = int(min(frame_h, np.ceil(y2 + margin)))

        if x2 <= x1 or y2 <= y1:
            return None

        return x1, y1, x2, y2

    def get_roi(self, frame, return_info=False):
        """
        获取 YOLO 粗定位 ROI。

        返回：
            roi = (x1, y1, x2, y2)
        """
        if frame is None or frame.size == 0:
            info = {
                "found": False,
                "reason": "empty_frame",
                "boxes": np.empty((0, 4), dtype=np.float32),
                "scores": np.empty((0,), dtype=np.float32),
                "classes": np.empty((0,), dtype=np.int32),
            }
            return (None, info) if return_info else None

        frame_h, frame_w = frame.shape[:2]
        boxes, scores, classes = self.detect(frame)
        best = self.get_best_box(boxes, scores, classes)

        if best is None:
            info = {
                "found": False,
                "reason": "no_detection",
                "boxes": boxes,
                "scores": scores,
                "classes": classes,
            }
            return (None, info) if return_info else None

        roi = self.expand_box(best["box"], frame_w, frame_h)
        if roi is None:
            info = {
                "found": False,
                "reason": "invalid_roi",
                "boxes": boxes,
                "scores": scores,
                "classes": classes,
            }
            return (None, info) if return_info else None

        class_id = int(best["class_id"])
        class_name = self.class_names[class_id] if 0 <= class_id < len(self.class_names) else str(class_id)

        info = {
            "found": True,
            "roi": roi,
            "best_box": best["box"],
            "score": float(best["score"]),
            "class_id": class_id,
            "class_name": class_name,
            "boxes": boxes,
            "scores": scores,
            "classes": classes,
        }

        if self.debug:
            x1, y1, x2, y2 = roi
            self._log(
                f"[YOLO] ROI=({x1},{y1},{x2},{y2}), "
                f"class={class_name}, score={best['score']:.3f}, dets={len(boxes)}"
            )

        return (roi, info) if return_info else roi

    # ======================================================
    # 可视化
    # ======================================================

    def draw(self, frame, boxes, scores, classes, roi=None):
        """绘制 YOLO 原始框和扩大后的 ROI。"""
        if frame is None:
            return frame

        out = frame

        for box, score, cls_id in zip(boxes, scores, classes):
            x1, y1, x2, y2 = map(int, box)
            cls_id = int(cls_id)
            class_name = self.class_names[cls_id] if 0 <= cls_id < len(self.class_names) else str(cls_id)
            label = f"{class_name}:{float(score):.2f}"

            cv2.rectangle(out, (x1, y1), (x2, y2), (0, 0, 255), 2)

            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
            y_text_top = max(0, y1 - th - 8)
            cv2.rectangle(out, (x1, y_text_top), (x1 + tw + 4, y_text_top + th + 8), (0, 0, 255), -1)
            cv2.putText(
                out,
                label,
                (x1 + 2, y_text_top + th + 3),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2,
            )

        if roi is not None:
            rx1, ry1, rx2, ry2 = map(int, roi)
            cv2.rectangle(out, (rx1, ry1), (rx2, ry2), (0, 255, 255), 2)
            cv2.putText(
                out,
                "YOLO ROI",
                (rx1, max(20, ry1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
            )

        return out


class YoloDetector:
    """
    VisionPipeline 适配器。

    VisionPipeline 只需要：
        yolo.enabled
        yolo.detect(frame) -> {"bbox": roi, ...} 或 None
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.enabled = bool(cfg.get("enabled", cfg.get("enable", False)))
        self.model = None

        if not self.enabled:
            print("[YOLO] disabled")
            return

        try:
            self.model = YoloModel(
                model_path=cfg.get("model_path", "models/blank.bin"),
                conf_thresh=cfg.get("conf_thresh", 0.35),
                nms_thresh=cfg.get("nms_thresh", 0.50),
                class_names=cfg.get("class_names", ("frame",)),
                roi_margin=cfg.get("roi_margin", 25),
                min_area=cfg.get("min_area", 300),
                max_area_ratio=cfg.get("max_area_ratio", 0.90),
                min_wh=cfg.get("min_wh", 8),
                reg_max=cfg.get("reg_max", 16),
                debug=cfg.get("debug", False),
                enabled=True,
            )
        except Exception as e:
            # YOLO 是粗定位辅助模块，初始化失败不能影响 OpenCV 主检测。
            print(f"[YOLO] init failed, disabled: {e}")
            self.enabled = False
            self.model = None

    def detect(self, frame):
        if not self.enabled or self.model is None:
            return None

        roi, info = self.model.get_roi(frame, return_info=True)
        if roi is None:
            return None

        return {
            "bbox": roi,
            "roi": roi,
            "score": info.get("score", 0.0),
            "class_id": info.get("class_id", 0),
            "class_name": info.get("class_name", "frame"),
            "info": info,
        }

    def draw_last(self, frame, info):
        """可选 debug 接口。"""
        if self.model is None or info is None:
            return frame
        return self.model.draw(
            frame,
            info.get("boxes", []),
            info.get("scores", []),
            info.get("classes", []),
            roi=info.get("roi"),
        )


def expand_bbox(bbox, img_w, img_h, ratio=0.2):
    """兼容旧 E1 接口。"""
    if bbox is None:
        return None

    x1, y1, x2, y2 = bbox
    bw = x2 - x1
    bh = y2 - y1
    ex = int(bw * ratio)
    ey = int(bh * ratio)

    nx1 = max(0, int(x1 - ex))
    ny1 = max(0, int(y1 - ey))
    nx2 = min(int(img_w), int(x2 + ex))
    ny2 = min(int(img_h), int(y2 + ey))

    if nx2 <= nx1 or ny2 <= ny1:
        return None

    return nx1, ny1, nx2, ny2
