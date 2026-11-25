# Purpose: gate three-target alignment and call the C++ OpenCV nearest-circle implementation.
import ctypes
import math
from pathlib import Path


# ***
# 函数名字：line_angle
# 函数作用：用三个当前中心的正交最小二乘拟合方向计算带符号水平夹角。
# 参数意义与范围：
#   points：三个不同的有限(x,y)像素坐标；图像x向右、y向下，退化数据返回None。
# ***
def line_angle(points):
    if len(points) != 3 or len(set(tuple(p) for p in points)) != 3:
        return None
    if not all(math.isfinite(v) for p in points for v in p):
        return None
    mx, my = (sum(p[i] for p in points)/3 for i in (0,1))
    xx = sum((p[0]-mx)**2 for p in points)
    yy = sum((p[1]-my)**2 for p in points)
    xy = sum((p[0]-mx)*(p[1]-my) for p in points)
    if math.hypot(xx-yy,2*xy) <= max(xx+yy,1)*1e-10:
        return None
    angle = math.degrees(.5*math.atan2(2*xy,xx-yy))
    return (angle+90)%180-90


# ***
# 函数名字：target_geometry
# 函数作用：从当前YOLO结果提取三个唯一标靶及其拟合偏转角。
# 参数意义与范围：
#   detections：当前帧Detection序列；要求三个标靶类别各一个且面积为正，否则返回无效结果。
# ***
def target_geometry(detections):
    marks = [d for d in detections if d.label in ("targetOne","targetTwo","targetThree") and d.area > 0]
    if len(marks)!=3 or len({d.label for d in marks})!=3:
        return None, ()
    marks = tuple(next(d for d in marks if d.label==label) for label in ("targetOne","targetTwo","targetThree"))
    return line_angle([d.center for d in marks]), marks


# ***
# 函数名字：alignment_target
# 函数作用：筛选满足三标靶连线角度和中心距离条件的标靶二。
# 参数意义与范围：
#   detections：当前帧Detection序列；各框坐标为对应图像内像素，面积非负。
#   size：图像(width, height)二元组；两个分量均为正整数像素。
#   threshold：允许的连线角度；有限数值[0, 45]度，当前配置为3度。
# ***
def alignment_target(detections, size, threshold):
    angle, marks = target_geometry(detections)
    if angle is None or abs(angle) > threshold + 1e-9:
        return None
    distance = [(d.center[0]-size[0]/2)**2+(d.center[1]-size[1]/2)**2 for d in marks]
    best = min(distance)
    nearest = [d for d, v in zip(marks, distance) if abs(v-best) < 1e-9]
    return nearest[0] if len(nearest) == 1 and nearest[0].label == "targetTwo" else None


# ***
# 函数名字：map_seed
# 函数作用：将YOLO框中心按归一化变换映射到精定位分辨率。
# 参数意义与范围：
#   target：触发帧的targetTwo检测框；范围为面积大于零且中心位于原图内的Detection。
#   source：源图像(width, height)；两个分量均为正整数像素。
#   destination：目标模式图像(width, height)；两个分量为正整数像素，默认(1920, 1080)。
#   transform：归一化(sx, sy, ox, oy)；各分量有限且在[-10, 10]内，sx和sy必须大于零。
# ***
def map_seed(target, source, destination, transform):
    x, y = target.center
    sx, sy, ox, oy = transform
    result = ((x/source[0]*sx+ox)*destination[0], (y/source[1]*sy+oy)*destination[1])
    if not (0 <= result[0] < destination[0] and 0 <= result[1] < destination[1]):
        raise ValueError("FOCUS_SEED_OUTSIDE_IMAGE")
    return result


class CircleFinder:
    # ***
    # 函数名字：__init__
    # 函数作用：初始化当前类的依赖、配置和内部状态。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   library：本机原生圆检测共享库路径；范围为当前平台可加载且具有规定C接口的文件。
    #   settings：当前模块配置字典；键、类型及数值范围由Configuration.validate约束。
    # ***
    def __init__(self, library, settings):
        import numpy as np
        self.settings = settings
        self.base_settings = dict(settings)
        self.library = ctypes.CDLL(str(Path(library)))
        self.function = self.library.car26_nearest_circle
        self.function.argtypes = [np.ctypeslib.ndpointer(dtype=np.uint8, flags="C_CONTIGUOUS"),
            ctypes.c_int, ctypes.c_int, ctypes.c_double, ctypes.c_double,
            ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_double, ctypes.c_double,
            ctypes.POINTER(ctypes.c_double)]
        self.function.restype = ctypes.c_int

    # ***
    # 函数名字：set_target_scale
    # 函数作用：依据触发标靶框在高分辨率下的尺寸排除细碎伪圆并覆盖真实圆环。
    # 参数意义与范围：
    #   self：当前CircleFinder实例。
    #   target：触发帧targetTwo检测框；宽高必须为正像素。
    #   source：触发图像(width,height)；两个分量为正整数像素。
    #   destination：精定位模式图像(width,height)；两个分量为正整数像素，默认(1920,1080)。
    # ***
    def set_target_scale(self, target, source, destination):
        sx,sy,_,_=self.base_settings['mode_transform']
        width=(target.x2-target.x1)*destination[0]/source[0]*sx
        height=(target.y2-target.y1)*destination[1]/source[1]*sy
        minimum=max(self.base_settings['minimum_radius_px'],round(min(width,height)*.20))
        maximum=min(530,max(minimum+1,round(max(width,height)*.65)))
        self.settings=dict(self.base_settings,minimum_radius_px=min(minimum,maximum-1),maximum_radius_px=maximum,
                           search_radius_px=min(540,max(self.base_settings['search_radius_px'],maximum+8,round(max(width,height)*.8))))

    # ***
    # 函数名字：find
    # 函数作用：调用原生圆检测寻找距离种子最近的有效圆。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   image：BGR uint8人工或实拍图像；非空H×W×3，本轮精定位为1920×1080。
    #   seed：圆搜索种子(x, y)；有限像素坐标且位于当前图像边界内。
    # ***
    def find(self, image, seed):
        import cv2
        import numpy as np
        gray = np.ascontiguousarray(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
        out = (ctypes.c_double * 3)()
        s = self.settings
        status = self.function(gray, gray.shape[1], gray.shape[0], *seed,
            s["search_radius_px"], s["minimum_radius_px"], s["maximum_radius_px"],
            s["canny_threshold"], s["circle_threshold"], out)
        if status < 0:
            raise RuntimeError("FOCUS_NATIVE_ERROR")
        if not status or not (0 <= out[0] < gray.shape[1] and 0 <= out[1] < gray.shape[0]):
            return None
        return tuple(out)
