# Purpose: filter detections by area and overlap across the full image without a center-height cutoff.
from ..communication.protocol import BLOCK_CODES, MARK_CODES


# ***
# 函数名字：overlap
# 函数作用：计算两个检测框的相交面积相对较小框的比例。
# 参数意义与范围：
#   a：第一个Detection；范围为当前图像中的检测框。
#   b：第二个Detection；范围为当前图像中的检测框。
# ***
def overlap(a, b):
    area = max(0, min(a.x2, b.x2) - max(a.x1, b.x1)) * max(
        0, min(a.y2, b.y2) - max(a.y1, b.y1))
    return area / min(a.area, b.area) if min(a.area, b.area) > 0 else 0.0


# ***
# 函数名字：filter_objects
# 函数作用：在完整画面按面积和遮挡规则筛选有效物块及标靶，不限制中心高度。
# 参数意义与范围：
#   detections：当前帧Detection序列；各框坐标为对应图像内像素，面积非负。
#   height：图像高度；正整数像素，保留调用接口但不用于中心高度过滤。
#   settings：当前模块配置字典；键、类型及数值范围由Configuration.validate约束。
# ***
def filter_objects(detections, height, settings):
    kept = [d for d in detections if d.area > 0
            and d.area >= settings["minimum_area_px"]]
    threes = [d for d in kept if d.label == "targetThree"]
    threshold = settings["mark23_overlap"]
    kept = [d for d in kept if not (threshold > 0 and d.label == "targetTwo"
            and any(overlap(d, t) >= threshold for t in threes))]
    blocks = [d for d in kept if d.label in BLOCK_CODES]
    threshold = settings["mark_block_overlap"]
    return tuple(d for d in kept if not (threshold > 0 and d.label in MARK_CODES
                 and any(overlap(d, b) >= threshold for b in blocks)))
