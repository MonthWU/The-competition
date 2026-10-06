#!/usr/bin/env python3
"""obj_dnn —— YOLOv11 物块/标识识别推理节点（板端原生推理，替代 dnn_node_example）。

为什么需要本节点
----------------
dnn/yolo11_x5.bin 的输出张量是 **NCHW + float32**，而 TROS 的 parser_yolov8
硬编码假设 **NHWC + int32**（D-Robotics 官方导出格式）。二者不兼容，直接跑
dnn_node_example 会在后处理阶段段错误（实测 exit -11）。

本节点改用 hobot_dnn.pyeasy_dnn 直接推理 + 自写 DFL 解码，对外发布与
dnn_node_example **完全一致**的 ai_msgs/PerceptionTargets，因此：
  - obj_serial（串口下发）无需改动
  - websocket（Web 渲染画框）无需改动
  - 也不再需要 hobot_codec / hobot_shm（直接订阅 /image 的 mjpeg，
    自己解码 + 缩放，省掉一整段零拷贝链路）

坐标约定
--------
模型输入 640x640，图像为 W0xH0（默认 640x480，直接 resize 不保比例）。
输出框先按模型坐标解码，再按 (W0/in_w, H0/in_h) 映射回图像坐标，
保证 websocket 能在 /image 上正确叠框、obj_serial 能拿到图像系坐标。
"""

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, RegionOfInterest

from ai_msgs.msg import PerceptionTargets, Perf, Roi, Target
from hobot_dnn import pyeasy_dnn as dnn

STRIDES = (8, 16, 32)
REG_MAX = 16


def bgr2nv12(image):
    """BGR -> NV12（Y 平面 + 交错 UV 平面）。"""
    height, width = image.shape[0], image.shape[1]
    area = height * width
    yuv420p = cv2.cvtColor(image, cv2.COLOR_BGR2YUV_I420).reshape((area * 3 // 2,))
    y = yuv420p[:area]
    uv_planar = yuv420p[area:].reshape((2, area // 4))
    uv_packed = uv_planar.transpose((1, 0)).reshape((area // 2,))
    nv12 = np.zeros_like(yuv420p)
    nv12[:area] = y
    nv12[area:] = uv_packed
    return nv12


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


class ObjDnnNode(Node):
    def __init__(self, name="obj_dnn"):
        super().__init__(name)

        self.declare_parameter("model_file", "/root/dev_ws/appli/dnn/yolo11_x5.bin")
        self.declare_parameter("cls_names_list", "/root/dev_ws/appli/dnn/classes.names")
        self.declare_parameter("image_topic", "/image")
        self.declare_parameter("msg_pub_topic_name", "hobot_dnn_detection")
        self.declare_parameter("score_threshold", 0.25)
        self.declare_parameter("nms_threshold", 0.45)

        self.score_thr = float(self.get_parameter("score_threshold").value)
        self.nms_thr = float(self.get_parameter("nms_threshold").value)
        model_file = self.get_parameter("model_file").value
        names_file = self.get_parameter("cls_names_list").value
        image_topic = self.get_parameter("image_topic").value
        pub_topic = self.get_parameter("msg_pub_topic_name").value

        with open(names_file) as f:
            self.names = [l.strip() for l in f if l.strip()]

        self.get_logger().info(f"loading model: {model_file}")
        self.model = dnn.load(model_file)[0]
        p = self.model.inputs[0].properties
        self.in_h, self.in_w = int(p.shape[2]), int(p.shape[3])

        # ---- 探测输出布局（2026-09-28）----
        # 不同模型的输出排布不同，且 properties.layout 字段可能标错
        # （实测障碍模型的 cls 标 NCHW 但 shape 是 (1,H,W,C)），
        # 故按 shape 判断：box 通道数恒为 4*REG_MAX=64，用它做判据最可靠，
        # cls 与该布局保持一致。
        s1 = tuple(self.model.outputs[1].properties.shape)
        self.hwc = (len(s1) == 4 and s1[-1] == 4 * REG_MAX)
        self.get_logger().info(
            "output layout: %s  (box shape=%s, cls shape=%s)"
            % ("HWC/NHWC" if self.hwc else "CHW/NCHW",
               s1, tuple(self.model.outputs[0].properties.shape)))
        self.get_logger().info(
            "model input %dx%d, classes(%d): %s"
            % (self.in_w, self.in_h, len(self.names), self.names))

        self.pub = self.create_publisher(PerceptionTargets, pub_topic, 10)
        self.sub = self.create_subscription(CompressedImage, image_topic, self.on_image, 10)

        self._frames = 0
        self._fps = 0.0
        self._t_last = time.time()
        self._log_once = False
        self.get_logger().info(
            "obj_dnn ready: sub=%s pub=%s score=%.2f nms=%.2f"
            % (image_topic, pub_topic, self.score_thr, self.nms_thr))

    @staticmethod
    def _dfl(box):
        """DFL 解码（CHW 布局）：box(64,h,w) -> dist(4,h,w)，单位 grid。"""
        _, h, w = box.shape
        b = box.reshape(4, REG_MAX, h, w)
        b = b - b.max(axis=1, keepdims=True)
        e = np.exp(b)
        s = e / e.sum(axis=1, keepdims=True)
        proj = np.arange(REG_MAX, dtype=np.float32).reshape(1, REG_MAX, 1, 1)
        return (s * proj).sum(axis=1)

    @staticmethod
    def _dfl_hwc(box):
        """DFL 解码（HWC 布局）：box(h,w,64) -> dist(h,w,4)，单位 grid。"""
        h, w, _ = box.shape
        b = box.reshape(h, w, 4, REG_MAX)
        b = b - b.max(axis=-1, keepdims=True)
        e = np.exp(b)
        s = e / e.sum(axis=-1, keepdims=True)
        return (s * np.arange(REG_MAX, dtype=np.float32)).sum(axis=-1)

    def on_image(self, msg):
        t0 = time.time()
        arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn("imdecode failed, skip frame")
            return
        H0, W0 = frame.shape[:2]

        resized = cv2.resize(frame, (self.in_w, self.in_h), interpolation=cv2.INTER_AREA)
        outs = self.model.forward(bgr2nv12(resized))
        t1 = time.time()

        boxes, scores, ids = [], [], []
        for si, stride in enumerate(STRIDES):
            hh, ww = self.in_h // stride, self.in_w // stride
            cls_buf = np.asarray(outs[2 * si].buffer, dtype=np.float32)
            box_buf = np.asarray(outs[2 * si + 1].buffer, dtype=np.float32)
            if self.hwc:
                # NHWC：cls (h,w,C) / box (h,w,64)
                cls = cls_buf.reshape(hh, ww, -1)
                box = box_buf.reshape(hh, ww, 4 * REG_MAX)
                sc = sigmoid(cls)
                cid = sc.argmax(axis=-1)
                csc = sc.max(axis=-1)
                mask = csc >= self.score_thr
                if not mask.any():
                    continue
                dist = self._dfl_hwc(box)                    # (h,w,4)
                ys, xs = np.nonzero(mask)
                ax = (xs + 0.5) * stride
                ay = (ys + 0.5) * stride
                x1 = ax - dist[ys, xs, 0] * stride
                y1 = ay - dist[ys, xs, 1] * stride
                x2 = ax + dist[ys, xs, 2] * stride
                y2 = ay + dist[ys, xs, 3] * stride
            else:
                # NCHW：cls (C,h,w) / box (64,h,w)
                cls = cls_buf.reshape(-1, hh, ww)
                box = box_buf.reshape(4 * REG_MAX, hh, ww)
                sc = sigmoid(cls)
                cid = sc.argmax(axis=0)
                csc = sc.max(axis=0)
                mask = csc >= self.score_thr
                if not mask.any():
                    continue
                dist = self._dfl(box)                        # (4,h,w)
                ys, xs = np.nonzero(mask)
                ax = (xs + 0.5) * stride
                ay = (ys + 0.5) * stride
                x1 = ax - dist[0][ys, xs] * stride
                y1 = ay - dist[1][ys, xs] * stride
                x2 = ax + dist[2][ys, xs] * stride
                y2 = ay + dist[3][ys, xs] * stride
            for i in range(len(xs)):
                boxes.append([float(x1[i]), float(y1[i]), float(x2[i]), float(y2[i])])
                scores.append(float(csc[ys[i], xs[i]]))
                ids.append(int(cid[ys[i], xs[i]]))

        keep = []
        if boxes:
            idx = cv2.dnn.NMSBoxes(
                [[b[0], b[1], b[2] - b[0], b[3] - b[1]] for b in boxes],
                scores, self.score_thr, self.nms_thr)
            if len(idx):
                keep = np.array(idx).flatten().tolist()

        # 模型坐标 -> 图像坐标
        sx, sy = W0 / float(self.in_w), H0 / float(self.in_h)

        out_msg = PerceptionTargets()
        out_msg.header.stamp = msg.header.stamp
        out_msg.header.frame_id = msg.header.frame_id or "default_cam"

        for k in keep:
            b = boxes[k]
            x1 = max(0, int(round(b[0] * sx)))
            y1 = max(0, int(round(b[1] * sy)))
            x2 = min(W0 - 1, int(round(b[2] * sx)))
            y2 = min(H0 - 1, int(round(b[3] * sy)))
            if x2 <= x1 or y2 <= y1:
                continue
            cid = ids[k]
            tgt = Target()
            tgt.type = self.names[cid] if cid < len(self.names) else str(cid)
            tgt.track_id = 0
            roi = Roi()
            roi.type = tgt.type
            r = RegionOfInterest()
            r.x_offset, r.y_offset = x1, y1
            r.width, r.height = (x2 - x1), (y2 - y1)
            r.do_rectify = False
            roi.rect = r
            roi.confidence = float(scores[k])
            tgt.rois.append(roi)
            out_msg.targets.append(tgt)

        # 帧率统计（每秒刷新一次）
        self._frames += 1
        now = time.time()
        if now - self._t_last >= 1.0:
            self._fps = self._frames / (now - self._t_last)
            self._frames = 0
            self._t_last = now
            self._log_once = True
        out_msg.fps = int(self._fps)

        perf = Perf()
        perf.type = "obj_dnn"
        perf.stamp_start = msg.header.stamp
        perf.stamp_end = msg.header.stamp
        perf.time_ms_duration = (time.time() - t0) * 1000.0
        out_msg.perfs.append(perf)

        self.pub.publish(out_msg)

        # 每完成一个 1 秒统计窗口、且有目标时打印一行。
        # （原先按 int(fps)%5==0 判断，首帧会用"节点启动到首帧"的累积时长算出接近 0 的
        #   fps 却仍满足条件，导致日志里出现误导性的 "fps 0.0"。）
        if self._log_once and out_msg.targets:
            self._log_once = False
            self.get_logger().info(
                "det %d target(s), img %dx%d, pre+infer %.1fms, fps %.1f"
                % (len(out_msg.targets), W0, H0, (t1 - t0) * 1000, self._fps))


def main(args=None):
    rclpy.init(args=args)
    node = ObjDnnNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()
