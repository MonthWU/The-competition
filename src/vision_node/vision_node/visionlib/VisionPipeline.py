import time

from .CVModel import process_frame
from .YoloModel import YoloDetector


class VisionPipeline:
    """
    视觉流水线：
    - YOLO 只做粗定位 ROI；
    - OpenCV 在 ROI 内精确检测白框；
    - 如果 OpenCV 丢失白框，或者 SharedState 判定白框中心大跳变，主程序可调用 reset_roi()；
    - reset_roi() 后下一帧会重新跑 YOLO/全图搜索，避免错误 ROI 长时间污染 OpenCV。
    """

    def __init__(self, yolo_cfg, cv_cfg, target_cfg, task_cfg):
        self.yolo_cfg = yolo_cfg
        self.cv_cfg = cv_cfg
        self.target_cfg = target_cfg
        self.task_cfg = task_cfg

        self.yolo = YoloDetector(yolo_cfg)

        self.frame_index = 0
        self.last_roi = None
        self.last_yolo_result = None
        self.last_roi_source = "none"
        self.last_yolo_ts = 0.0

    def reset_roi(self):
        self.last_roi = None
        self.last_yolo_result = None
        self.last_roi_source = "reset"

    def process(self, frame):
        self.frame_index += 1

        roi = self.last_roi
        roi_source = self.last_roi_source
        ran_yolo_this_frame = False
        now = time.monotonic()
        detect_period_sec = float(self.yolo_cfg.get("detect_period_sec", 1.0))

        def run_yolo_once(update_source="yolo"):
            nonlocal roi, roi_source, ran_yolo_this_frame, now
            yolo_result = self.yolo.detect(frame)
            self.last_yolo_result = yolo_result
            self.last_yolo_ts = now
            ran_yolo_this_frame = True

            if yolo_result is not None and "bbox" in yolo_result:
                roi = yolo_result["bbox"]
                self.last_roi = roi
                self.last_roi_source = update_source
                roi_source = update_source
            else:
                roi_source = self.last_roi_source
            return yolo_result

        if self.yolo.enabled:
            need_periodic_yolo = (
                self.frame_index == 1
                or self.last_roi is None
                or detect_period_sec <= 0.0
                or (now - self.last_yolo_ts) >= detect_period_sec
            )
            if need_periodic_yolo:
                run_yolo_once("yolo_periodic")

        result, vis = process_frame(frame, self.cv_cfg, task_cfg=self.task_cfg, roi=roi)

        if (
            self.yolo.enabled
            and not result.get("frame_found", False)
            and not ran_yolo_this_frame
        ):
            run_yolo_once("yolo_recover")
            result, vis = process_frame(frame, self.cv_cfg, task_cfg=self.task_cfg, roi=roi)

        if isinstance(result, dict):
            result["yolo_roi"] = roi if roi_source == "yolo" or roi is not None else None
            result["roi_source"] = roi_source
            result["yolo_result"] = self.last_yolo_result

        if not result.get("frame_found", False):
            self.reset_roi()

        return result, vis
