# Purpose: encode MCU frames and parse commands independently of device I/O and ROS.
from dataclasses import dataclass

BLOCK_CODES = dict(zip(("redblock", "blackblock", "greenblock", "yellowblock", "lightblueblock", "darkblueblock"), range(0x41, 0x47)))
MARK_CODES = dict(zip(("targetOne", "targetTwo", "targetThree"), range(0x51, 0x54)))
CANDIDATE_IDS = (1, 3, 5, 7, 9, 11, 12, 13, 15, 17, 19, 21, 23)
STATE_FLAGS = {"qr": 0x60, "block": 0x61, "target": 0x62}
TARGET_POSE_CODE = 0x72
BLOCK_BATCH_CODE = 0x71
QR_CODE = 0x70
# Internal identity marker: ASCII payloads cannot impersonate the raw 0x62 entry frame.
TARGET_FOCUS_REQUEST = object()
CONTROL_COMMANDS = frozenset(('qr_reset', 'task_switch', 'task_qr', 'task_object'))


# ***
# 函数名字：control_frame
# 函数作用：将网页或下位机控制动作编码为同一套ASCII串口命令帧。
# 参数意义与范围：
#   action：qr_reset、task_switch、task_qr或task_object，必须完全匹配。
# ***
def control_frame(action):
    if action not in CONTROL_COMMANDS:
        raise ValueError('UNKNOWN_CONTROL_COMMAND')
    return ('[' + action + ']').encode('ascii')


@dataclass(frozen=True)
class StateUpdate:
    flag: int
    active: bool


class CommandParser:
    """Extract bracket commands, resynchronizing on noise and nested starts."""

    # ***
    # 函数名字：__init__
    # 函数作用：初始化当前类的依赖、配置和内部状态。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   maximum_size：命令缓冲区最大字节数；正整数，默认64，超限时丢弃当前命令。
    # ***
    def __init__(self, maximum_size=64):
        self.maximum_size = maximum_size
        self.buffer = bytearray()
        self.dropped = 0

    # ***
    # 函数名字：feed
    # 函数作用：解析分片命令及原始字节状态通知；状态通知与控制命令使用不同类型。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   chunk：收到的字节序列；范围为bytes或可迭代的[0, 255]整数。
    # ***
    def feed(self, chunk):
        result = []
        for b in chunk:
            if b == ord("["):
                self.buffer[:] = b"["
            elif self.buffer:
                self.buffer.append(b)
                if len(self.buffer) > self.maximum_size:
                    self.dropped += 1
                    self.buffer.clear()
                elif b == ord("]"):
                    payload = bytes(self.buffer[1:-1])
                    self.buffer.clear()
                    if len(payload) >= 2 and payload[1] == ord(","):
                        if (len(payload) == 3 and payload[0] in STATE_FLAGS.values()
                                and payload[2] in (0, 1)):
                            result.append(StateUpdate(payload[0], bool(payload[2])))
                        else:
                            self.dropped += 1
                        continue
                    try:
                        result.append(payload.decode("ascii").strip())
                    except UnicodeDecodeError:
                        self.dropped += 1
        return result


# ***
# 函数名字：map_frame
# 函数作用：将合法障碍格编号编码为带异或校验的地图报文。
# 参数意义与范围：
#   ids：障碍格号序列；最多三个，元素必须属于CANDIDATE_IDS。
# ***
def map_frame(ids):
    ids = sorted(set(ids))
    if len(ids) > 3 or any(type(i) is not int or i not in CANDIDATE_IDS for i in ids):
        raise ValueError("INVALID_OBSTACLE_IDS")
    check = len(ids)
    for i in ids:
        check ^= i
    return ("[" + " ".join([str(len(ids)), *map(str, ids), f"{check:02X}"]) + "]").encode("ascii")


# ***
# 函数名字：qr_frame
# 函数作用：保留二维码UTF-8原文并编码转发报文。
# 参数意义与范围：
#   text：非空Unicode二维码原文，不能含帧尾字符]；保留原始空格并编码为UTF-8。
# ***
def qr_frame(text):
    if not isinstance(text, str) or not text:
        raise ValueError("EMPTY_QR_PAYLOAD")
    if "]" in text:
        raise ValueError("QR_PAYLOAD_CONTAINS_TERMINATOR")
    return bytes((ord('['), QR_CODE)) + text.encode("utf-8") + b"]"


# ***
# 函数名字：target_frame
# 函数作用：保留单目标绝对坐标的兼容编码；当前物块业务使用blocks_frame合包。
# 参数意义与范围：
#   label：类别字符串；范围为BLOCK_CODES或MARK_CODES中的合法类别。
#   x：目标绝对水平像素坐标；[0, 65535]内整数。
#   y：目标绝对垂直像素坐标；[0, 65535]内整数。
# ***
def target_frame(label, x, y):
    code = {**BLOCK_CODES, **MARK_CODES}.get(label)
    if code is None or any(type(v) is not int or not 0 <= v <= 65535 for v in (x, y)):
        raise ValueError("INVALID_TARGET_FRAME")
    return bytes((ord("["), code, x & 255, x >> 8, y & 255, y >> 8, ord("]")))


# ***
# 函数名字：alignment_frame
# 函数作用：将标靶二有符号像素偏差编码为独立0x54报文。
# 参数意义与范围：
#   dx：水平像素偏差；[-32768, 32767]内整数，向右为正。
#   dy：垂直像素偏差；[-32768, 32767]内整数，向下为正。
# ***
def alignment_frame(dx, dy):
    import struct
    if any(type(v) is not int or not -32768 <= v <= 32767 for v in (dx, dy)):
        raise ValueError("INVALID_ALIGNMENT_OFFSET")
    return b"[\x54" + struct.pack("<hh", dx, dy) + b"]"


# ***
# 函数名字：target_pose_frame
# 函数作用：编码含角度、二维像素偏差及识别来源的固定十字节标靶报文。
# 参数意义与范围：
#   alpha：当前三标靶拟合角；有限实数[-90,90)度，线上为0.01度单位有符号int16。
#   dx：当前图像水平像素偏差；整数[-32768,32767]，向右为正。
#   dy：当前图像垂直像素偏差；整数[-32768,32767]，向下为正。
#   source：识别来源整数；0为YOLO处理图，1为OpenCV标靶模式采集图。
# ***
def target_pose_frame(alpha, dx, dy, source):
    import math
    import struct
    if (type(source) is not int or source not in (0,1) or isinstance(alpha,bool)
        or not isinstance(alpha,(int,float)) or not math.isfinite(alpha) or not -90 <= alpha < 90
        or any(type(v) is not int or not -32768 <= v <= 32767 for v in (dx,dy))):
        raise ValueError("INVALID_TARGET_POSE")
    angle=max(-9000,min(8999,round(alpha*100)))
    return bytes((ord('['),TARGET_POSE_CODE,source))+struct.pack('<hhh',angle,dx,dy)+b']'


# ***
# 函数名字：state_frame
# 函数作用：编码五字节状态通知，原始Flagnum与BOOL之间使用英文逗号分隔。
# 参数意义与范围：
#   flag：用户定义的状态编号；0x60二维码、0x61物块、0x62标靶。
#   active：状态是否进入；bool或整数0/1，线上为原始单字节00/01。
# ***
def state_frame(flag, active):
    if (type(flag) is not int or flag not in STATE_FLAGS.values()
            or type(active) not in (bool, int) or active not in (0, 1)):
        raise ValueError("INVALID_STATE_FRAME")
    return bytes((ord('['), flag, ord(','), int(active), ord(']')))


# ***
# 函数名字：disc_offset_frame
# 函数作用：编码一次圆盘圆心测量的固定九字节回复，字段之间保留原始英文逗号。
# 参数意义与范围：
#   dx：圆心相对采集原图中心的水平偏差；int16整数，向右为正。
#   dy：圆心相对采集原图中心的垂直偏差；int16整数，向下为正。
# ***
def disc_offset_frame(dx, dy):
    import struct
    if any(type(value) is not int or not -32768 <= value <= 32767 for value in (dx, dy)):
        raise ValueError('INVALID_DISC_OFFSET')
    return b'[\x61,' + struct.pack('<h', dx) + b',' + struct.pack('<h', dy) + b']'


# ***
# 函数名字：blocks_frame
# 函数作用：将本帧各类物块合为一包，单字节num表示实际发送的物块种类数。
# 参数意义与范围：
#   records：(label, dx, dy)序列，0～6种且类别不重复；label属于BLOCK_CODES，dx/dy为[-32768,32767]整数。
# ***
def blocks_frame(records):
    import struct
    records = tuple(records)
    if len(records) > len(BLOCK_CODES):
        raise ValueError("TOO_MANY_BLOCK_TYPES")
    frame = bytearray((ord('['), BLOCK_BATCH_CODE, len(records)))
    seen = set()
    for label, dx, dy in records:
        code = BLOCK_CODES.get(label)
        if code is None or any(type(value) is not int or not -32768 <= value <= 32767
                               for value in (dx, dy)):
            raise ValueError("INVALID_BLOCK_RECORD")
        if label in seen:
            raise ValueError("DUPLICATE_BLOCK_CLASS")
        seen.add(label)
        frame.extend(struct.pack('<Bhh', code, dx, dy))
    frame.append(ord(']'))
    return bytes(frame)


# ***
# 函数名字：select_targets
# 函数作用：保留ROI内每种有效物块的最高置信度候选及全部有效标靶，不按距离选取单种。
# 参数意义与范围：
#   detections：当前帧Detection序列；各框坐标为对应图像内像素，面积非负。
#   reference：兼容旧接口的参考点(x,y)或None；当前忽略，不用于按距离截断物块。
#   roi：可选(xmin, xmax, ymax)；None不筛选，否则为[0, 65535]像素且xmin<=xmax。
# ***
def select_targets(detections, reference, roi=None):
    blocks = [d for d in detections if d.label in BLOCK_CODES and d.area > 0]
    if roi is not None:
        xmin, xmax, ymax = roi
        blocks = [d for d in blocks if xmin <= d.center[0] <= xmax and d.center[1] <= ymax]
    best = {}
    for block in blocks:
        if block.label not in best or block.confidence > best[block.label].confidence:
            best[block.label] = block
    return list(best.values()) + [d for d in detections if d.label in MARK_CODES and d.area > 0]
