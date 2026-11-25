"""Bounded, thread-safe snapshots for diagnostics and the HMI."""

from collections import deque
import copy
import threading
import time


class Telemetry:
    # ***
    # 函数名字：__init__
    # 函数作用：初始化当前类的依赖、配置和内部状态。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    # ***
    def __init__(self):
        self._lock = threading.Lock()
        self._status = {"application": {"state": "created"}}
        self._events = deque(maxlen=100)
        self._frames = {}
        self._serial = {direction: deque(maxlen=100) for direction in ('tx', 'rx')}

    # ***
    # 函数名字：serial
    # 函数作用：独立保存两个方向的原始串口记录，避免高频TX挤掉RX。
    # 参数意义与范围：self：当前Telemetry实例；direction：tx或rx；raw：实际传输的bytes。
    # ***
    def serial(self, direction, raw):
        with self._lock:
            self._serial[direction].append({'at': time.time(), 'hex': raw.hex(' '),
                                             'text': raw.decode('utf-8', errors='replace'), 'bytes': len(raw)})

    # ***
    # 函数名字：update
    # 函数作用：在锁保护下合并指定模块的最新状态。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   module：状态所属模块名称；非空字符串，通常为camera、radar、communication、mission等。
    #   values：需记录的关键字字段；范围为可深拷贝、可序列化的状态值。
    # ***
    def update(self, module, **values):
        with self._lock:
            self._status.setdefault(module, {}).update(values, updated_at=time.time())

    # ***
    # 函数名字：event
    # 函数作用：记录一条带时间戳的有界诊断事件。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   module：状态所属模块名称；非空字符串，通常为camera、radar、communication、mission等。
    #   kind：事件类型或HTTP内容类型；范围为调用方明确给定的非空字符串。
    #   values：需记录的关键字字段；范围为可深拷贝、可序列化的状态值。
    # ***
    def event(self, module, kind, **values):
        with self._lock:
            self._events.append({"module": module, "kind": kind, "at": time.time(), **values})

    # ***
    # 函数名字：snapshot
    # 函数作用：返回状态及事件的独立副本。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    # ***
    def snapshot(self):
        with self._lock:
            return copy.deepcopy({**self._status, "events": list(self._events),
                                  'previews': {key: {'age_s': time.monotonic()-value[0]}
                                               for key, value in self._frames.items()},
                                  'serial': {key: list(value) for key, value in self._serial.items()}})

    # ***
    # 函数名字：clear_image
    # 函数作用：关闭相机后清除其旧预览，防止重新开启时出现上轮画面。
    # 参数意义与范围：self：当前Telemetry实例；role：qr、object或prescan角色字符串。
    # ***
    def clear_image(self, role):
        with self._lock:
            self._frames.pop(role, None)

    # ***
    # 函数名字：image
    # 函数作用：保存指定角色的最新JPEG或读取其现有快照。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   role：相机角色字符串；范围为qr、object或prescan。
    #   jpeg：编码后的JPEG字节；None表示只读取快照，非空bytes表示更新。
    # ***
    def image(self, role, jpeg=None):
        with self._lock:
            if jpeg is not None:
                self._frames[role] = (time.monotonic(), jpeg)
            return self._frames.get(role)
