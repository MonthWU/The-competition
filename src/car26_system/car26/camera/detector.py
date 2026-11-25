# Purpose: preserve image aspect ratio and decode YOLO11 NV12/DFL results into original pixels.

import numpy as np
import time
from concurrent.futures import ThreadPoolExecutor

from ..types import Detection
from .classmap import load_model_labels
from .candidates import NativeCandidates, select_classes


# ***
# 函数名字：letterbox_geometry
# 函数作用：计算等比例缩放后的整数尺寸和居中补边偏移，供预处理与坐标还原共用。
# 参数意义与范围：
#   width：原图宽度；正整数像素。
#   height：原图高度；正整数像素。
#   input_size：方形模型输入边长；正偶数像素，默认640。
# ***
def letterbox_geometry(width, height, input_size=640):
    scale = min(input_size / width, input_size / height)
    resized_width = max(1, min(input_size, round(width * scale)))
    resized_height = max(1, min(input_size, round(height * scale)))
    return resized_width, resized_height, (input_size-resized_width)//2, (input_size-resized_height)//2


# ***
# 函数名字：as_hwc
# 函数作用：将支持的模型张量布局统一为HWC数组。
# 参数意义与范围：
#   value：待写入的完整字节报文；范围为bytes，发送时必须在配置超时内写完。
#   channels：目标通道数；正整数，须与当前模型输出通道一致。
#   side：方形特征图边长；正整数，当前YOLO输出为80、40或20。
# ***
def as_hwc(value, channels, side):
    value = np.asarray(value)
    if not np.issubdtype(value.dtype, np.floating):
        raise ValueError("QUANTIZED_OUTPUT_REQUIRES_DEQUANTIZATION")
    if value.shape in ((1, channels, side, side), (channels, side, side)):
        return value.reshape(channels, side, side).transpose(1, 2, 0)
    if value.shape in ((1, side, side, channels), (side, side, channels)):
        return value.reshape(side, side, channels)
    raise ValueError("MODEL_OUTPUT_SHAPE_UNSUPPORTED:" + str(value.shape))


# ***
# 函数名字：decode_heads
# 函数作用：解码六路YOLO输出，执行分类别NMS并撤销等比例补边得到原图坐标。
# 参数意义与范围：
#   outputs：六路浮点模型输出；范围为三组分类张量与64通道DFL张量。
#   labels：非空类别名序列；长度须等于分类通道数，当前物块模型为九类。
#   width：图像宽度；正整数像素。
#   height：图像高度；正整数像素。
#   score_threshold：分类置信度阈值；有限数值[0, 1]。
#   nms_threshold：同类框NMS的IoU阈值；有限数值[0, 1]。
#   input_size：模型方形输入边长；当前BPU适配范围固定为640像素。
#   reg_max：DFL离散通道数；正整数，当前模型为16。
#   candidate_selector：NativeCandidates实例或None，筛选后仍保留原概率、DFL和NMS规则。
# ***
def decode_heads(outputs, labels, width, height, score_threshold, nms_threshold,
                 input_size=640, reg_max=16, candidate_selector=None):
    if len(outputs) != 6:
        raise ValueError("MODEL_REQUIRES_SIX_OUTPUTS")
    candidates = []
    for index, stride in enumerate((8, 16, 32)):
        side = input_size // stride
        raw_box = np.asarray(outputs[index * 2 + 1])
        boxes = as_hwc(raw_box, 4 * reg_max, side)
        # The existing models have misleading cls layout metadata. The box
        # tensor's actual shape determines the shared memory order for the pair.
        raw_cls = np.asarray(outputs[index * 2])
        if not np.issubdtype(raw_cls.dtype, np.floating):
            raise ValueError("QUANTIZED_OUTPUT_REQUIRES_DEQUANTIZATION")
        if raw_cls.size != side * side * len(labels):
            raise ValueError("MODEL_CLASS_COUNT_MISMATCH")
        chw = raw_box.shape in ((1, 4 * reg_max, side, side), (4 * reg_max, side, side))
        logits = (raw_cls.reshape(len(labels), side, side).transpose(1, 2, 0)
                  if chw else raw_cls.reshape(side, side, len(labels)))
        yy, xx, classes, scores = select_classes(logits, score_threshold, candidate_selector)
        if not len(xx):
            continue
        distributions = boxes[yy, xx].reshape(-1, 4, reg_max)
        distributions = np.exp(distributions - distributions.max(axis=2, keepdims=True))
        distributions /= distributions.sum(axis=2, keepdims=True)
        distances = (distributions * np.arange(reg_max)).sum(axis=2) * stride
        centers = np.column_stack((xx + .5, yy + .5)) * stride
        corners = np.column_stack((centers[:, 0] - distances[:, 0],
                                   centers[:, 1] - distances[:, 1],
                                   centers[:, 0] + distances[:, 2],
                                   centers[:, 1] + distances[:, 3]))
        for row, category, score in zip(corners, classes, scores):
            candidates.append((labels[category], float(score), row))
    # Each class has its own NMS; overlapping different semantic classes survive.
    result = []
    for candidate in sorted(candidates, key=lambda item: item[1], reverse=True):
        label, score, box = candidate
        suppress = False
        for other in result:
            if label != other[0]:
                continue
            other_box = other[2]
            inter = max(0, min(box[2], other_box[2]) - max(box[0], other_box[0])) * max(
                0, min(box[3], other_box[3]) - max(box[1], other_box[1]))
            union = ((box[2] - box[0]) * (box[3] - box[1])
                     + (other_box[2] - other_box[0]) * (other_box[3] - other_box[1]) - inter)
            if union > 0 and inter / union > nms_threshold:
                suppress = True
                break
        if not suppress:
            result.append(candidate)
    detections = []
    resized_width, resized_height, left, top = letterbox_geometry(width, height, input_size)
    for label, score, box in result:
        corners = np.rint((box - np.array([left, top, left, top]))
                          * np.array([width/resized_width, height/resized_height,
                                      width/resized_width, height/resized_height])).astype(int)
        corners[[0, 2]] = corners[[0, 2]].clip(0, width - 1)
        corners[[1, 3]] = corners[[1, 3]].clip(0, height - 1)
        detection = Detection(label, score, *map(int, corners))
        if detection.area:
            detections.append(detection)
    return tuple(detections)


# ***
# 函数名字：prepare_nv12
# 函数作用：应用颜色增益，等比例缩放并用114灰色居中补边，再转换为NV12输入。
# 参数意义与范围：
#   frame：当前图像BGR uint8数组；范围为非空H×W×3。
#   rgb_gains：R/G/B增益三元组；每项为[0.001, 10]内有限数值。
#   input_size：模型方形输入边长；当前BPU适配范围固定为640像素。
# ***
def prepare_nv12(frame, rgb_gains, input_size=640):
    import cv2
    gains = np.array(rgb_gains, dtype=np.float32)
    if not np.array_equal(gains, np.ones(3, dtype=np.float32)):
        lut = np.clip(np.arange(256, dtype=np.float32)[:, None] * gains[::-1],
                      0, 255).astype(np.uint8).reshape(1, 256, 3)
        frame = cv2.LUT(frame, lut)
    width, height, left, top = letterbox_geometry(frame.shape[1], frame.shape[0], input_size)
    resized = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
    image = cv2.copyMakeBorder(resized, top, input_size-height-top,
                               left, input_size-width-left, cv2.BORDER_CONSTANT, value=(114,114,114))
    i420 = cv2.cvtColor(image, cv2.COLOR_BGR2YUV_I420).reshape(-1)
    area = input_size * input_size
    nv12 = np.empty_like(i420)
    nv12[:area] = i420[:area]
    nv12[area:] = i420[area:].reshape(2, area // 4).transpose(1, 0).reshape(-1)
    return nv12


class BpuDetector:
    # ***
    # 函数名字：__init__
    # 函数作用：核对模型与类别绑定，加载原BPU模型并按配置创建唯一推理工作线程。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   configuration：项目Configuration对象；范围为已解析且满足对应模式约束的配置。
    #   settings：当前模块配置字典；键、类型及数值范围由Configuration.validate约束。
    # ***
    def __init__(self, configuration, settings):
        from hobot_dnn import pyeasy_dnn
        self.settings = settings
        self.candidate_selector = (NativeCandidates(configuration.path(settings['candidate_library']))
                                   if settings.get('candidate_library') else None)
        self.labels = load_model_labels(configuration,settings)
        models = pyeasy_dnn.load(str(configuration.path(settings["file"])))
        if len(models) != 1 or len(models[0].inputs) != 1:
            raise ValueError("MODEL_INPUT_UNSUPPORTED")
        self.model = models[0]
        shape = tuple(self.model.inputs[0].properties.shape)
        if shape not in ((1, 3, 640, 640), (1, 640, 640, 3)):
            raise ValueError("MODEL_INPUT_MUST_BE_640_NV12:" + str(shape))
        self.supports_prefetch = settings.get('overlap_capture', False)
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='car26-bpu') if self.supports_prefetch else None
        self.metrics = {}
        self.closed = False

    # ***
    # 函数名字：_forward
    # 函数作用：执行一次原BPU调用并提取独立输出，记录执行与输出提取耗时。
    # 参数意义与范围：self：当前BpuDetector实例；nv12：640×640对应的614400字节uint8输入。
    # ***
    def _forward(self, nv12):
        start = time.monotonic()
        tensors = self.model.forward(nv12)
        forwarded = time.monotonic()
        outputs = [tensor.buffer for tensor in tensors]
        self.metrics = {'forward_ms': (forwarded-start)*1000,
                        'outputs_ms': (time.monotonic()-forwarded)*1000}
        return outputs

    # ***
    # 函数名字：detect
    # 函数作用：执行一次BPU推理并返回图像坐标系中的检测框。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   frame：非空BGR uint8 H×W×3图像，物块默认640×360。
    #   prefetch：None或无参数回调，只能由唯一相机线程读取下一帧；每次最多调用一次。
    # ***
    def detect(self, frame, prefetch=None):
        if self.closed:
            raise RuntimeError('INFERENCE_CLOSED')
        nv12 = prepare_nv12(frame, self.settings["rgb_gains"])
        if self.executor is not None and prefetch is not None:
            # There is one submitted task and it is always joined before return,
            # including when capture raises. No inference backlog can accumulate.
            future = self.executor.submit(self._forward, nv12)
            started = time.monotonic()
            try:
                prefetch()
            finally:
                prefetch_ms = (time.monotonic()-started)*1000
                waiting = time.monotonic()
                outputs = future.result()
                self.metrics.update(wait_ms=(time.monotonic()-waiting)*1000,
                                    prefetch_ms=prefetch_ms, overlapped=True)
        else:
            outputs = self._forward(nv12)
            self.metrics.update(wait_ms=self.metrics['forward_ms']+self.metrics['outputs_ms'],
                                prefetch_ms=0.0, overlapped=False)
        return decode_heads(outputs, self.labels, frame.shape[1], frame.shape[0],
                            self.settings["score_threshold"], self.settings["nms_threshold"],
                            candidate_selector=self.candidate_selector)

    # ***
    # 函数名字：close
    # 函数作用：等待唯一在途推理结束并关闭工作线程，幂等释放模型引用。
    # 参数意义与范围：self：当前BpuDetector实例，不允许与其它调用者并发执行detect。
    # ***
    def close(self):
        if self.closed:
            return
        self.closed = True
        if self.executor is not None:
            self.executor.shutdown(wait=True, cancel_futures=True)
            self.executor = None
        self.model = None
