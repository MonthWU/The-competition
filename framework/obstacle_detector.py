"""obstacle_detector —— 黑色几何体识别（骨架）。

主识别：YOLOv11（1 类 obstacle，.bin 经 rdk_model_zoo ultralytics_yolo 转换）。
兜底：CV 黑色分割（低亮度阈值 + 连通域 + 面积/圆度过滤），形状无关，防漏检。
标定：单应性矩阵（图像像素 → 地面坐标），离线标定存盘。
"""

# 2026-09-23 全局扫描阶段使用的 yolo 模型（1 类 ball，md5 6fd337ab…）
import os

_DEFAULT_TASK_JSON = "/root/dev_ws/appli/framework/dnn/task_obj_obstacle.json"
_DEFAULT_MODEL_BIN = "/root/dev_ws/appli/framework/dnn/yolo11_x5_obstacle.bin"
_DEFAULT_CLASSES = "/root/dev_ws/appli/framework/dnn/classes_obstacle.names"


class ObstacleDetector:
    def __init__(self, model_path="", homography_path="", task_json=_DEFAULT_TASK_JSON):
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

    def detect(self, frame):
        """返回 [(cx, cy, w, h, conf), ...]，像素坐标。

        TODO: 当前仅记录模型路径。实际推理由 launch 中的 dnn_node_example 完成，
        结果通过 /hobot_dnn_detection 话题传出，由 mission_dispatcher 解析后调用本模块。
        本方法保留作为占位，待 road_judge 与共享内存订阅接入后再实现像素→网格映射。
        """
        return []

    def pixel_to_map(self, px):
        """像素坐标 → 地图坐标（单应性变换）。

        TODO: 待单应性标定完成后实现 cv2.perspectiveTransform。
        """
        return (px[0], px[1])

    def calibrate(self, image_points, map_points):
        """标定单应性矩阵并存盘。"""
        raise NotImplementedError("TODO: cv2.findHomography + save")