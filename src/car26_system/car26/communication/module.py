# Purpose: own the MCU UART, frame writes and diagnostic snapshots.
import logging
import sys
import time

from .protocol import BLOCK_CODES, CommandParser, StateUpdate, TARGET_FOCUS_REQUEST, blocks_frame, state_frame, select_targets, target_pose_frame, disc_offset_frame


class SerialTransport:
    # ***
    # 函数名字：__init__
    # 函数作用：初始化当前类的依赖、配置和内部状态。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   settings：当前模块配置字典；键、类型及数值范围由Configuration.validate约束。
    # ***
    def __init__(self, settings):
        import serial

        options = dict(port=settings["device"], baudrate=settings["baudrate"],
                       timeout=0.02, write_timeout=settings["write_timeout_s"],
                       bytesize=8, parity="N", stopbits=1, xonxoff=False, rtscts=False, dsrdtr=False)
        if sys.platform.startswith("linux"):
            options["exclusive"] = True
        self.serial = serial.Serial(**options)

    # ***
    # 函数名字：read
    # 函数作用：从串口传输层读取一段有界字节数据。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    # ***
    def read(self):
        return self.serial.read(min(max(self.serial.in_waiting, 1), 4096))

    # ***
    # 函数名字：write
    # 函数作用：向串口传输层写入指定字节数据。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   value：待写入的完整字节报文；范围为bytes，发送时必须在配置超时内写完。
    # ***
    def write(self, value):
        return self.serial.write(value)

    # ***
    # 函数名字：flush
    # 函数作用：等待串口发送缓冲区排空。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    # ***
    def flush(self):
        self.serial.flush()

    # ***
    # 函数名字：close
    # 函数作用：关闭当前模块持有的线程、连接或设备资源。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    # ***
    def close(self):
        self.serial.close()


class CommunicationModule:
    # ***
    # 函数名字：__init__
    # 函数作用：初始化当前类的依赖、配置和内部状态。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   settings：当前模块配置字典；键、类型及数值范围由Configuration.validate约束。
    #   telemetry：共享Telemetry对象；范围为当前应用初始化的线程安全状态容器。
    #   transport：串口传输对象；None使用真实SerialTransport，否则须实现read/write/flush/close接口。
    # ***
    def __init__(self, settings, telemetry, transport=None):
        self.settings = settings
        self.telemetry = telemetry
        self.transport = transport
        self.parser = CommandParser()
        self.log = logging.getLogger("car26.communication")
        self.sent = 0
        self.received = 0
        self.local_states = set()
        self.peer_states = set()

    # ***
    # 函数名字：start
    # 函数作用：创建并启动当前模块的工作资源。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    # ***
    def start(self):
        if self.transport is None:
            self.transport = SerialTransport(self.settings)
        self.telemetry.update("communication", state="ready", device=self.settings["device"],
                              baudrate=self.settings["baudrate"])

    # ***
    # 函数名字：poll
    # 函数作用：记录MCU进度，将0x61圆盘及0x62标靶进入请求交给主线程门控。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    # ***
    def poll(self):
        raw = self.transport.read()
        self.received += len(raw)
        if raw:
            self.telemetry.serial('rx', raw)
        commands = []
        for command in self.parser.feed(raw):
            if isinstance(command, StateUpdate):
                if command.active:
                    self.peer_states.add(command.flag)
                else:
                    self.peer_states.discard(command.flag)
                self.telemetry.event("communication", "peer_state", flag=command.flag, active=command.active)
                if command.flag == 0x61 and command.active:
                    commands.append('disc_once')
                elif command.flag == 0x62 and command.active:
                    commands.append(TARGET_FOCUS_REQUEST)
                continue
            commands.append(command)
            self.telemetry.event("communication", "rx", payload=command)
            self.log.debug("RX %s", command)
        self.telemetry.update("communication", received_bytes=self.received,
                              parser_dropped=self.parser.dropped, peer_states=sorted(self.peer_states))
        return commands

    # ***
    # 函数名字：send_state
    # 函数作用：发送本机真实状态变化，成功写完后独立更新本机状态集合，不控制任务或相机。
    # 参数意义与范围：
    #   self：当前CommunicationModule实例；仅通信所属主线程调用。
    #   flag：当前状态编号；0x60二维码、0x61物块、0x62标靶。
    #   active：是否进入该状态；bool或整数0/1，支持多个状态独立进入和退出。
    # ***
    def send_state(self, flag, active):
        value = state_frame(flag, active)
        self.send(value)
        if active:
            self.local_states.add(flag)
        else:
            self.local_states.discard(flag)
        self.telemetry.update("communication", local_states=sorted(self.local_states))
        self.telemetry.event("communication", "local_state", flag=flag, active=bool(active))

    # ***
    # 函数名字：send
    # 函数作用：在超时范围内完整写入一帧并更新通信快照。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   value：待写入的完整字节报文；范围为bytes，发送时必须在配置超时内写完。
    # ***
    def send(self, value):
        deadline = time.monotonic() + self.settings["write_timeout_s"]
        offset = 0
        while offset < len(value):
            if time.monotonic() >= deadline:
                raise TimeoutError("MCU_WRITE_TIMEOUT")
            count = self.transport.write(value[offset:])
            if type(count) is not int or not 0 < count <= len(value) - offset:
                raise OSError("MCU_SHORT_WRITE")
            offset += count
            self.telemetry.serial('tx', value[offset-count:offset])
        self.sent += offset
        self.telemetry.event("communication", "tx", hex=value.hex(), bytes=offset)
        self.telemetry.update("communication", sent_bytes=self.sent, received_bytes=self.received,
                              parser_dropped=self.parser.dropped)
        self.log.debug("TX %s", value.hex())

    # ***
    # 函数名字：flush
    # 函数作用：等待串口发送缓冲区排空。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    # ***
    def flush(self):
        self.transport.flush()

    # ***
    # 函数名字：send_detections
    # 函数作用：将本帧ROI内各类物块的类别与画面中心偏差合为一包，num为种类数，空帧为0。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   detections：当前帧Detection序列；各框坐标为对应图像内像素，面积非负。
    #   processing：已验证camera_processing配置；block_roi可选筛选，reference_px不再参与选择。
    #   width：当前算法图像宽度；正整数像素，水平偏差相对width/2。
    #   height：当前算法图像高度；正整数像素，垂直偏差相对height/2。
    # ***
    def send_detections(self, detections, processing, width, height):
        selected = select_targets(detections, None, processing["block_roi"])
        records = [(d.label, round(d.center[0]-width/2), round(d.center[1]-height/2))
                   for d in selected if d.label in BLOCK_CODES]
        self.send(blocks_frame(records))

    # ***
    # 函数名字：send_alignment
    # 函数作用：发送当前三标靶角度及标靶二的有符号像素偏差。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   delta：(dx, dy)像素差；两个分量为[-32768, 32767]内整数，右和下为正。
    #   alpha：当前三标靶拟合角；有限实数[-90,90)度。
    #   source：当前定位来源；整数0为YOLO，1为OpenCV。
    # ***
    def send_alignment(self, alpha, delta, source):
        self.send(target_pose_frame(alpha, *delta, source))

    # ***
    # 函数名字：send_disc_offset
    # 函数作用：由唯一UART主线程发送一次有效圆盘中心偏差。
    # 参数意义与范围：self：当前通信实例；delta：(dx,dy)，两个分量为int16整数原图像素差。
    # ***
    def send_disc_offset(self, delta):
        self.send(disc_offset_frame(*delta))

    # ***
    # 函数名字：close
    # 函数作用：关闭当前模块持有的线程、连接或设备资源。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    # ***
    def close(self):
        if self.transport is not None:
            self.transport.close()
        self.telemetry.update("communication", state="stopped")
