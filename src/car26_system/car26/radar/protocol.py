import struct

from ..types import ScanPoint


# ***
# 函数名字：request
# 函数作用：编码C1命令及可选的带校验载荷。
# 参数意义与范围：
#   command：C1命令字节；[0, 255]内整数。
#   payload：可选雷达载荷字节；长度[0, 255]，空载荷不附加长度及校验。
# ***
def request(command, payload=b""):
    frame = bytes((0xA5, command))
    if payload:
        frame += bytes((len(payload),)) + payload
        checksum = 0
        for value in frame:
            checksum ^= value
        frame += bytes((checksum,))
    return frame


# ***
# 函数名字：descriptor
# 函数作用：解析并验证C1七字节响应描述符。
# 参数意义与范围：
#   data：C1响应描述符；必须恰好7字节并以A5 5A开头。
# ***
def descriptor(data):
    if len(data) != 7 or data[:2] != b"\xa5\x5a":
        raise ValueError("C1_BAD_DESCRIPTOR")
    value = struct.unpack("<I", data[2:6])[0]
    return value & 0x3fffffff, value >> 30, data[6]


class StandardParser:
    # ***
    # 函数名字：__init__
    # 函数作用：初始化当前类的依赖、配置和内部状态。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    # ***
    def __init__(self):
        self.buffer = bytearray()
        self.dropped_bytes = 0

    # ***
    # 函数名字：feed
    # 函数作用：累计输入字节并解析当前可用的完整协议消息。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   data：配置字典或协议字节；具体类型与范围按本函数所属类的输入协议确定。
    # ***
    def feed(self, data):
        self.buffer.extend(data)
        result = []
        while len(self.buffer) >= 5:
            a, b, c, d, e = self.buffer[:5]
            angle = ((b >> 1) | (c << 7)) / 64
            if ((a & 1) == ((a >> 1) & 1)) or not b & 1 or angle >= 360:
                del self.buffer[0]
                self.dropped_bytes += 1
                continue
            del self.buffer[:5]
            result.append((bool(a & 1), ScanPoint(angle, ((d | (e << 8)) / 4) / 1000, a >> 2)))
        return result
