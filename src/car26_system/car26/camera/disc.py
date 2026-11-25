# Purpose: load explicit white-disc parameters and call the native outer-ellipse detector.
import ctypes
import json
import math
from pathlib import Path

FIELDS = ('processing_width', 'maximum_saturation', 'minimum_value', 'morphology_size',
          'minimum_component_area_ratio', 'minimum_diameter_ratio', 'maximum_diameter_ratio',
          'minimum_axis_ratio', 'inlier_distance_px', 'minimum_inliers', 'minimum_coverage_deg',
          'minimum_visible_support', 'minimum_white_fraction', 'iterations',
          'bright_maximum_saturation', 'bright_minimum_value', 'minimum_enclosure_ratio')
LIMITS = ((160,1920),(0,255),(0,255),(1,15),(.001,.9),(.05,2),(.5,4),(.3,1),
          (.5,10),(20,1200),(60,300),(.3,1),(.3,1),(20,1000),(0,255),(0,255),(.7,1))
INTEGER_FIELDS = (0,1,2,3,9,13,14,15)


# ***
# 函数名字：load_disc_settings
# 函数作用：读取并严格校验独立圆盘参数，不在算法内固化现场阈值。
# 参数意义与范围：path：UTF-8 JSON配置路径，必须包含FIELDS中17项参数，范围见LIMITS。
# ***
def load_disc_settings(path):
    settings = json.loads(Path(path).read_text(encoding='utf-8-sig'))
    if set(settings) != set(FIELDS):
        raise ValueError('DISC_CONFIG_FIELDS')
    for index, (key, (low, high)) in enumerate(zip(FIELDS, LIMITS)):
        value = settings[key]
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or not low <= value <= high
                or (index in INTEGER_FIELDS and value != int(value))):
            raise ValueError('DISC_CONFIG_RANGE:' + key)
    if settings['morphology_size'] % 2 != 1:
        raise ValueError('DISC_CONFIG_ODD_KERNEL')
    if settings['minimum_diameter_ratio'] > settings['maximum_diameter_ratio']:
        raise ValueError('DISC_CONFIG_DIAMETER_ORDER')
    return settings


class DiscFinder:
    # ***
    # 函数名字：__init__
    # 函数作用：加载当前平台的大圆盘检测共享库及已验证参数。
    # 参数意义与范围：self：当前实例；library：本机libcar26_disc.so路径；config：17项参数JSON路径。
    # ***
    def __init__(self, library, config):
        import numpy as np
        self.settings = load_disc_settings(config)
        self.parameters = (ctypes.c_double * len(FIELDS))(*(self.settings[key] for key in FIELDS))
        self.library = ctypes.CDLL(str(Path(library).resolve()))
        self.function = self.library.car26_disc_center
        self.function.argtypes = [np.ctypeslib.ndpointer(dtype=np.uint8, flags='C_CONTIGUOUS'),
                                  ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_double),
                                  ctypes.c_int, ctypes.POINTER(ctypes.c_double)]
        self.function.restype = ctypes.c_int

    # ***
    # 函数名字：find
    # 函数作用：返回当前帧大圆盘外缘椭圆及像素偏差，缺少证据时返回None，不保留旧结果。
    # 参数意义与范围：self：当前实例；image：BGR uint8 H×W×3图像，宽高各[32,8192]，允许非连续数组。
    # ***
    def find(self, image):
        import numpy as np
        if (not isinstance(image, np.ndarray) or image.dtype != np.uint8 or image.ndim != 3
                or image.shape[2] != 3 or not all(32 <= size <= 8192 for size in image.shape[:2])):
            raise ValueError('DISC_INVALID_IMAGE')
        height, width = image.shape[:2]
        output = (ctypes.c_double * 12)()
        status = self.function(np.ascontiguousarray(image), width, height, self.parameters, len(FIELDS), output)
        if status < 0:
            raise RuntimeError('DISC_NATIVE_ERROR')
        if status == 0:
            return None
        x, y, axis_x, axis_y, angle, error, coverage, support, white, count, profile, threshold = output
        return dict(center=[x,y], axes=[axis_x,axis_y], angle_deg=angle,
                    offset=[x-width/2,y-height/2], residual_px=error, coverage_deg=coverage,
                    visible_support=support, white_fraction=white, inliers=int(count),
                    segmentation='low_saturation' if profile==0 else 'brightness', value_threshold=threshold)


# ***
# 函数名字：draw_disc
# 函数作用：在图像上标记外缘、圆心和相对画面中心偏差，检测失败时明确标记无目标。
# 参数意义与范围：image：可写BGR uint8预览图；result：DiscFinder.find返回的字典或None；coordinate_size：结果原图(width,height)，两个正整数；None表示与预览同尺寸。
# ***
def draw_disc(image, result, coordinate_size=None):
    import cv2
    height, width = image.shape[:2]
    source_width,source_height = coordinate_size or (width,height)
    sx,sy = width/source_width,height/source_height
    cv2.drawMarker(image, (width//2,height//2), (255,180,0), cv2.MARKER_CROSS, 36, 2)
    if result is None:
        cv2.putText(image, 'WHITE DISC: NOT FOUND', (24,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,120,255), 2)
        return
    raw_x,raw_y = result['center']
    x,y = raw_x*sx,raw_y*sy
    cv2.ellipse(image, ((x,y),(result['axes'][0]*sx,result['axes'][1]*sy),result['angle_deg']), (0,255,0), 2)
    if 0 <= x < width and 0 <= y < height:
        cv2.drawMarker(image, (round(x),round(y)), (0,0,255), cv2.MARKER_CROSS, 42, 3)
        cv2.line(image, (width//2,height//2), (round(x),round(y)), (0,220,255), 2)
    dx,dy = result['offset']
    cv2.putText(image, f'DISC ({raw_x:.1f},{raw_y:.1f}) dx={dx:+.1f} dy={dy:+.1f}',
                (12,height-16), cv2.FONT_HERSHEY_SIMPLEX, max(.4,.85*width/1920), (0,255,255), 1)


class DiscAlignment:
    # ***
    # 函数名字：__init__
    # 函数作用：创建当前0x61阶段的圆心对准锁存器，达标后不再调用检测器。
    # 参数意义与范围：self：当前实例；finder：提供find(image)的检测器；tolerance：非负有限(dx,dy)像素容差，默认业务配置(5,5)。
    # ***
    def __init__(self, finder, tolerance):
        if (len(tolerance)!=2 or any(isinstance(v,bool) or not isinstance(v,(int,float))
                                    or not math.isfinite(v) or v<0 for v in tolerance)):
            raise ValueError('DISC_INVALID_TOLERANCE')
        self.finder,self.tolerance = finder,tuple(tolerance)
        self.reset()

    # ***
    # 函数名字：reset
    # 函数作用：进入新0x61阶段时清除对准锁定及调用计数。
    # 参数意义与范围：self：当前实例。
    # ***
    def reset(self):
        self.aligned = False
        self.processed_frames = 0

    # ***
    # 函数名字：process
    # 函数作用：未对准时计算当前圆心，两轴浮点绝对偏差均不超过容差时锁定；锁定后不再计算。
    # 参数意义与范围：self：当前实例；image：当前物块相机完整采集BGR帧，容差使用该帧原始像素坐标。
    # ***
    def process(self, image):
        if self.aligned:
            return None
        self.processed_frames += 1
        result = self.finder.find(image)
        if result is not None:
            offset = result['offset']
            if len(offset)!=2 or not all(math.isfinite(v) for v in offset):
                return None
            self.aligned = all(abs(value)<=limit for value,limit in zip(offset,self.tolerance))
        return result


# ***
# 函数名字：validate_disc_centering
# 函数作用：在不打开设备或原生库时检查0x61圆盘对准功能的开关、容差和可选文件。
# 参数意义与范围：configuration：项目Configuration实例；check_files：布尔值，True同时验证参数JSON及原生库文件存在。
# ***
def validate_disc_centering(configuration, check_files):
    settings = configuration.data.get('disc_centering',{'enabled':False})
    if not isinstance(settings,dict) or type(settings.get('enabled')) is not bool:
        return ['INVALID_DISC_CENTERING_ENABLED']
    if not settings['enabled']:
        return []
    errors = []
    tolerance = settings.get('tolerance_px')
    if (not isinstance(tolerance,(list,tuple)) or len(tolerance)!=2
            or any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v)
                   or not 0<=v<=100 for v in tolerance)):
        errors.append('INVALID_DISC_CENTERING_TOLERANCE')
    for key in ('native_library','parameter_file'):
        value = settings.get(key)
        if not isinstance(value,str) or not value:
            errors.append('DISC_CENTERING_PATH_REQUIRED:'+key)
        elif check_files:
            path = configuration.path(value)
            if not path.is_file():
                errors.append('DISC_CENTERING_FILE_MISSING:'+key)
            elif key=='parameter_file':
                try:
                    load_disc_settings(path)
                except (ValueError,TypeError,OSError) as error:
                    errors.append('DISC_CENTERING_PARAMETERS_INVALID:'+str(error))
    return errors
