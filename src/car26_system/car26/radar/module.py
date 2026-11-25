import logging
import struct
import threading
import time

from ..types import RadarFrame
from .processing import cluster_points
from .protocol import StandardParser, descriptor, request


class RadarModule:
    # ***
    # 函数名字：__init__
    # 函数作用：初始化当前类的依赖、配置和内部状态。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   settings：当前模块配置字典；键、类型及数值范围由Configuration.validate约束。
    #   telemetry：共享Telemetry对象；范围为当前应用初始化的线程安全状态容器。
    # ***
    def __init__(self, settings, telemetry):
        self.settings, self.telemetry = settings, telemetry
        self.thread = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.latest = None
        self.error = None
        self.log = logging.getLogger("car26.radar")

    # ***
    # 函数名字：start
    # 函数作用：创建并启动当前模块的工作资源。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    # ***
    def start(self):
        self.thread = threading.Thread(target=self._scan, name="radar")
        self.thread.start()

    # ***
    # 函数名字：result
    # 函数作用：读取当前模块最新快照并传播已记录的错误。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    # ***
    def result(self):
        with self.lock:
            if self.error:
                raise RuntimeError(self.error)
            return self.latest

    # ***
    # 函数名字：_scan
    # 函数作用：独占C1串口采集并发布完整扫描快照。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    # ***
    def _scan(self):
        port = None
        try:
            import serial
            port = serial.Serial(self.settings["device"], self.settings["baudrate"],
                                 timeout=.05, write_timeout=1, exclusive=True)

            # ***
            # 函数名字：write
            # 函数作用：向串口传输层写入指定字节数据。
            # 参数意义与范围：
            #   frame：C1完整发送帧；范围为按雷达协议编码的bytes。
            # ***
            def write(frame):
                if port.write(frame) != len(frame):
                    raise RuntimeError("C1_SHORT_WRITE")
                port.flush()

            # ***
            # 函数名字：read_exact
            # 函数作用：在超时或停止前读取指定长度的雷达响应。
            # 参数意义与范围：
            #   size：需要读取的字节数；正整数，当前协议最多20字节，2秒内须读完。
            # ***
            def read_exact(size):
                data = bytearray()
                deadline = time.monotonic() + 2
                while len(data) < size and time.monotonic() < deadline and not self.stop_event.is_set():
                    data.extend(port.read(size - len(data)))
                if len(data) != size:
                    raise RuntimeError("C1_RESPONSE_TIMEOUT")
                return bytes(data)

            # ***
            # 函数名字：response
            # 函数作用：发送雷达命令并核对响应描述后读取载荷。
            # 参数意义与范围：
            #   command：C1查询命令字节；当前调用范围为0x50或0x52。
            #   size：期望载荷长度；正整数，当前查询为3或20字节。
            #   kind：期望响应类型字节；[0, 255]内整数，须与描述符一致。
            # ***
            def response(command, size, kind):
                write(request(command))
                if descriptor(read_exact(7)) != (size, 0, kind):
                    raise RuntimeError("C1_UNEXPECTED_DESCRIPTOR")
                return read_exact(size)

            write(request(0x25))
            time.sleep(.01)
            port.reset_input_buffer()
            info = response(0x50, 20, 0x04)
            if info[0] >> 4 != 4:
                raise RuntimeError("RADAR_MODEL_IS_NOT_C1")
            health = response(0x52, 3, 0x06)
            if health[0] not in (0, 1):
                raise RuntimeError("C1_HEALTH_ERROR:" + str(struct.unpack("<H", health[1:])[0]))
            if health[0] == 1:
                self.log.warning("C1 health warning %s", health.hex())
            write(request(0xA8, struct.pack("<H", round(self.settings["frequency_hz"] * 60))))
            write(request(0x20))
            if descriptor(read_exact(7)) != (5, 1, 0x81):
                raise RuntimeError("C1_STANDARD_DESCRIPTOR_REQUIRED")
            self.telemetry.update("radar", state="ready", model=info[0], health=health[0])
            parser = StandardParser()
            points, started, sequence = [], False, 0
            last_circle = time.monotonic()
            while not self.stop_event.is_set():
                raw = port.read(min(max(port.in_waiting, 1), 8192))
                for first, point in parser.feed(raw):
                    if first:
                        now = time.monotonic()
                        if started:
                            sequence += 1
                            obstacles = cluster_points(points, self.settings["cluster_gap_m"],
                                                       self.settings["minimum_cluster_points"])
                            result = RadarFrame(sequence, last_circle, tuple(points), obstacles)
                            with self.lock:
                                self.latest = result
                            self.telemetry.update("radar", state="scanning", sequence=sequence,
                                                  points=len(points), obstacles=[o.__dict__ for o in obstacles],
                                                  dropped_bytes=parser.dropped_bytes,
                                                  scan_hz=round(1 / max(now - last_circle, .0001), 2))
                            self.log.debug("circle=%s points=%s obstacles=%s", sequence, len(points), len(obstacles))
                        last_circle = now
                        points, started = [], True
                    if started and self.settings["min_range_m"] <= point.distance_m <= self.settings["max_range_m"] and point.quality >= self.settings["minimum_quality"]:
                        points.append(point)
                        if len(points) > 10000:
                            raise RuntimeError("C1_CIRCLE_TOO_LARGE")
                if time.monotonic() - last_circle > self.settings["stream_timeout_s"]:
                    raise RuntimeError("C1_SCAN_TIMEOUT")
        except Exception as error:
            with self.lock:
                self.error = str(error)
            self.telemetry.update("radar", state="failed", error=str(error))
            self.log.exception("radar failed")
        finally:
            if port is not None:
                try:
                    port.write(request(0x25))
                    port.flush()
                    time.sleep(.01)
                finally:
                    port.close()

    # ***
    # 函数名字：close
    # 函数作用：关闭当前模块持有的线程、连接或设备资源。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    # ***
    def close(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=5)
            if self.thread.is_alive():
                raise RuntimeError("RADAR_STOP_TIMEOUT")
        self.telemetry.update("radar", state="stopped")
