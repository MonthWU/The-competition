# Purpose: preserve probability rounding and class ties while evaluating sigmoid only for possible cells.
import ctypes
import math
import numpy as np


class NativeCandidates:
    # ***
    # 函数名字：__init__
    # 函数作用：加载短时C++候选筛选库，保留GIL并声明只读输入及有界输出接口。
    # 参数意义与范围：
    #   self：当前NativeCandidates实例。
    #   library：存在且适配当前平台的共享库绝对路径。
    # ***
    def __init__(self, library):
        # The active 640-input/9-class scan is short; releasing and reacquiring
        # the GIL for each head can cost more than the native scan itself.
        self.library = ctypes.PyDLL(str(library))
        self.function = self.library.car26_candidate_cells
        self.function.argtypes = [np.ctypeslib.ndpointer(dtype=np.float32, flags='ALIGNED'),
            ctypes.c_int, ctypes.c_int, ctypes.c_int,
            ctypes.c_ssize_t, ctypes.c_ssize_t, ctypes.c_ssize_t, ctypes.c_double,
            np.ctypeslib.ndpointer(dtype=np.int32, flags='C_CONTIGUOUS'), ctypes.c_int]
        self.function.restype = ctypes.c_int

    # ***
    # 函数名字：select
    # 函数作用：调用C++筛选float32网格；其它浮点类型保留NumPy精度路径。
    # 参数意义与范围：
    #   self：当前NativeCandidates实例。
    #   logits：非空H×W×C浮点数组，宽高不超过640、类别数不超过1024。
    #   cutoff：保守logit下界，有限数值或负无穷。
    # ***
    def select(self, logits, cutoff):
        if logits.dtype != np.float32:
            return np.flatnonzero(logits.max(axis=2) >= cutoff)
        if not logits.flags.aligned or any(value <= 0 for value in logits.strides):
            logits = np.ascontiguousarray(logits)
        height, width, classes = logits.shape
        output = np.empty(height*width, dtype=np.int32)
        count = self.function(logits, height, width, classes,
                              *(stride//logits.itemsize for stride in logits.strides),
                              cutoff, output, output.size)
        if count < 0:
            raise ValueError('CANDIDATE_NATIVE_INPUT_INVALID')
        return output[:count]


# ***
# 函数名字：select_classes
# 函数作用：先保守筛选原始分数，再精确计算候选全部类别的概率，保留阈值与首类别并列规则。
# 参数意义与范围：
#   logits：非空H×W×C浮点原始分类输出，允许与旧路径相同的NaN/无穷值。
#   score_threshold：最终概率下界，有限数值[0,1]。
#   selector：NativeCandidates实例或None；None使用NumPy筛选以支持无原生库的纯函数测试。
# ***
def select_classes(logits, score_threshold, selector=None):
    # A loose bound only discards impossible cells. Final probabilities and
    # argmax still use the original expression, including rounding/saturation ties.
    lower = max(0.0, float(score_threshold)-8*float(np.finfo(logits.dtype).eps))
    cutoff = math.log(lower)-math.log1p(-lower) if lower > 0 else -math.inf
    positions = (selector.select(logits, cutoff) if selector is not None
                 else np.flatnonzero(logits.max(axis=2) >= cutoff))
    yy, xx = positions//logits.shape[1], positions % logits.shape[1]
    scores = 1/(1+np.exp(-np.clip(logits[yy, xx], -80, 80)))
    classes = scores.argmax(axis=1)
    best = scores[np.arange(len(classes)), classes]
    keep = best >= score_threshold
    return yy[keep], xx[keep], classes[keep], best[keep]
