# Purpose: coordinate the four modules and forward only current vision results to the MCU.
import logging
import signal
import threading
import time
import queue
import uuid
from collections import deque

from .camera.module import CameraModule
from .communication.protocol import (BLOCK_CODES, STATE_FLAGS, TARGET_FOCUS_REQUEST,
                                     CONTROL_COMMANDS, CommandParser, control_frame)
from .communication.module import CommunicationModule
from .hmi.module import HmiModule
from .mapping import CameraMapper, RadarMapper
from .mission import Mission
from .navigation import NavigationModule
from .radar.module import RadarModule
from .telemetry import Telemetry


class Application:
    # ***
    # 函数名字：__init__
    # 函数作用：初始化当前类的依赖、配置和内部状态。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   configuration：项目Configuration对象；范围为已解析且满足对应模式约束的配置。
    # ***
    def __init__(self, configuration):
        self.configuration = configuration
        self.telemetry = Telemetry()
        self.stop_event = threading.Event()
        self.communication = CommunicationModule(configuration.data["communication"], self.telemetry)
        self.camera = CameraModule(configuration, self.telemetry)
        self.radar = RadarModule(configuration.data["radar"], self.telemetry)
        self.control_queue = queue.Queue(maxsize=1)
        self.control_lock = threading.Lock()
        self.control_results = deque(maxlen=32)
        self.qr_cutoff = float('-inf')
        self.hmi = HmiModule(configuration.data["hmi"], self.telemetry,
                             self.request_qr_reset, self.request_task_switch, mode=configuration.mode)
        settings = configuration.data["mission"]
        self.mapper = None
        if settings["entry"] == "full":
            self.mapper = (CameraMapper if settings["prescan_source"] == "camera" else RadarMapper)(configuration)
        self.mission = Mission(settings, self.mapper.supports if self.mapper else lambda _: False)
        self.log = logging.getLogger("car26.app")
        self.focus_ending = False
        self.last_disc_request_id = 0
        self.navigation = (NavigationModule(configuration, self.telemetry)
                           if configuration.data.get("navigation", {}).get("enabled", False) else None)

    # ***
    # 函数名字：request_qr_reset
    # 函数作用：将二维码结果清空请求加入有界控制队列。
    # 参数意义与范围：self：当前Application实例。
    # ***
    def request_qr_reset(self):
        return self._request_control('qr_reset')

    # ***
    # 函数名字：request_task_switch
    # 函数作用：将二维码与物块任务切换请求加入有界控制队列，入队时固定目标。
    # 参数意义与范围：self：当前Application实例。
    # ***
    def request_task_switch(self):
        return self._request_control('task_switch')

    # ***
    # 函数名字：_request_control
    # 函数作用：把网页或UART控制动作编码成相同命令帧，仅提交主线程执行。
    # 参数意义与范围：
    #   self：当前Application实例。
    #   action：CONTROL_COMMANDS中支持的重置、切换或明确目标命令。
    #   source：web或uart；用于权限检查和追踪，不伪造串口读写记录。
    # ***
    def _request_control(self, action, source='web'):
        if source not in ('web', 'uart'):
            raise ValueError('INVALID_CONTROL_SOURCE')
        if source == 'web' and self.configuration.mode != 'debug':
            raise ValueError('WEB_REQUIRES_DEBUG')
        control_frame(action)
        if not self.control_lock.acquire(blocking=False):
            raise ValueError('CONTROL_BUSY')
        try:
            snapshot = self.telemetry.snapshot()
            if snapshot['application']['state'] != 'running' or self.mission.state not in ('QR', 'OBJECT'):
                raise ValueError('TASK_NOT_READY')
            if action != 'qr_reset' and not self.configuration.data['mission']['object_enabled']:
                raise ValueError('OBJECT_TASK_DISABLED')
            if action == 'task_switch':
                action = 'task_object' if self.mission.state == 'QR' else 'task_qr'
            frame = control_frame(action)
            request_id = uuid.uuid4().hex
            self.telemetry.update('control', request_id=request_id, status='pending',
                                  source=source, frame_hex=frame.hex(' '), error=None)
            self.control_queue.put_nowait((frame, request_id, source))
            return {'status': 'accepted', 'request_id': request_id, 'frame_hex': frame.hex(' ')}
        except Exception:
            self.control_lock.release()
            raise

    # ***
    # 函数名字：_process_controls
    # 函数作用：由主线程解析统一串口命令帧，执行重置/任务选择并发布执行结果。
    # 参数意义与范围：self：当前Application实例。
    # ***
    def _process_controls(self):
        try:
            frame, request_id, source = self.control_queue.get_nowait()
        except queue.Empty:
            return
        try:
            commands = CommandParser().feed(frame)
            if len(commands) != 1 or commands[0] not in CONTROL_COMMANDS:
                raise ValueError('INVALID_CONTROL_FRAME')
            action = commands[0]
            if self.mission.state not in ('QR', 'OBJECT'):
                raise ValueError('TASK_NOT_READY')
            if action == 'qr_reset':
                self.qr_cutoff = time.monotonic()
                self.telemetry.update('mission', qr_text=None, qr_seen_at=None, recognized_frames=0)
            else:
                target = 'object' if action == 'task_object' else 'qr'
                if self.mission.state != target.upper():
                    self.camera.switch(target)
                    self.focus_ending = False
                self.mission.state = 'OBJECT' if target == 'object' else 'QR'
                self.telemetry.update('mission', state=self.mission.state)
            self._complete_control(request_id, 'done')
            self.telemetry.event('control', 'command_done', source=source, command=action,
                                 frame_hex=frame.hex(' '))
        except Exception as error:
            self._complete_control(request_id, 'failed', str(error))
            raise
        finally:
            self.control_lock.release()

    # ***
    # 函数名字：_complete_control
    # 函数作用：保留最近32次控制结果，避免紧随其后的MCU命令覆盖网页正在等待的结果。
    # 参数意义与范围：
    #   self：当前Application实例，仅主线程调用。
    #   request_id：本次控制请求的UUID字符串。
    #   status：done或failed。
    #   error：失败原因字符串；成功时为None。
    # ***
    def _complete_control(self, request_id, status, error=None):
        self.control_results.append(dict(request_id=request_id, status=status, error=error))
        self.telemetry.update('control', request_id=request_id, status=status, error=error,
                              results=list(self.control_results))

    # ***
    # 函数名字：_send_camera_states
    # 函数作用：由唯一通信主线程发送已发生的识别功能变化，不将通知作为控制命令。
    # 参数意义与范围：
    #   self：当前Application实例，持有MCU通信模块。
    #   events：相机原子快照携带的有序(mode, active)序列；mode为qr、block或target，active为bool。
    # ***
    def _send_camera_states(self, events):
        for mode, active in events:
            self.communication.send_state(STATE_FLAGS[mode], active)

    # ***
    # 函数名字：_object_frame
    # 函数作用：去重发送单次圆盘测量，并按当前模式发送物块及标靶结果。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   frame：最新CameraFrame或RadarFrame；序号唯一且时间戳未超过配置的新鲜度门限。
    # ***
    def _object_frame(self, frame):
        if frame.disc_request_id and frame.disc_request_id > self.last_disc_request_id:
            self.last_disc_request_id = frame.disc_request_id
            if frame.disc_delta is not None:
                self.communication.send_disc_offset(frame.disc_delta)
            self.telemetry.event('camera', 'disc_reply' if frame.disc_delta is not None else 'disc_not_found',
                                 request_id=frame.disc_request_id, delta=frame.disc_delta)
        if frame.focus_active:
            if not self.focus_ending and frame.focus_delta is not None and frame.target_alpha is not None:
                self.communication.send_alignment(frame.target_alpha, frame.focus_delta, 1)
        else:
            self.focus_ending = False
            self.communication.send_detections(tuple(d for d in frame.detections if d.label in BLOCK_CODES),
                self.configuration.data["camera_processing"], frame.width, frame.height)
            twos = [d for d in frame.detections if d.label == 'targetTwo' and d.area > 0]
            if frame.target_alpha is not None and len(twos) == 1:
                x,y = twos[0].center
                self.communication.send_alignment(frame.target_alpha,
                    (round(x-frame.width/2),round(y-frame.height/2)),0)

    # ***
    # 函数名字：_effects
    # 函数作用：按状态机返回的动作列表协调模块和串口。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   effects：状态机动作序列；范围为应用支持的(action, value)二元组。
    # ***
    def _effects(self, effects):
        for action, value in effects:
            if action == "send":
                self.communication.send(value)
            elif action == "flush":
                self.communication.flush()
            elif action == "camera":
                self.camera.switch(value)
            elif action == "camera_stop":
                self.camera.close()
            elif action == "prescan" and self.configuration.data["mission"]["prescan_source"] == "camera":
                self.camera.switch("prescan")
            elif action == "qr_sent":
                self._effects(self.mission.qr_sent())

    # ***
    # 函数名字：run
    # 函数作用：执行应用主循环并在退出时关闭所持资源。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    # ***
    def run(self):
        previous_signals = {}
        for number in (signal.SIGINT, signal.SIGTERM):
            previous_signals[number] = signal.signal(number, lambda *_: self.stop_event.set())
        exit_code = 0
        try:
            self.communication.start()
            if self.configuration.data["radar"]["enabled"]:
                self.radar.start()
            if self.configuration.mode == 'debug' and self.configuration.data["hmi"]["enabled"]:
                self.hmi.start()
            else:
                self.telemetry.update('hmi', state='disabled')
            if self.mission.state == "QR":
                self.camera.switch("qr")
            elif self.mission.state == "OBJECT":
                self.camera.switch("object")
            self.telemetry.update("application", state="running", mode=self.configuration.mode)
            self.telemetry.update('mission', state=self.mission.state, qr_text=None, qr_seen_at=None,
                                  qr_reset_enabled=True,
                                  task_switch_enabled=self.configuration.data['mission']['object_enabled'])
            last_camera_sequence = None
            last_camera_received = time.monotonic()
            previous_role = None
            while not self.stop_event.is_set():
                self._process_controls()
                now = time.monotonic()
                for command in self.communication.poll():
                    if isinstance(command, str) and command in CONTROL_COMMANDS:
                        try:
                            self._request_control(command, source='uart')
                        except ValueError as error:
                            self.telemetry.event('control', 'command_rejected', source='uart',
                                                 command=command, error=str(error))
                            continue
                        self._process_controls()
                        continue
                    if command is TARGET_FOCUS_REQUEST:
                        accepted = self.mission.state == 'OBJECT' and self.camera.request_focus()
                        if accepted:
                            self.focus_ending = False
                        self.telemetry.event('camera', 'focus_request', accepted=bool(accepted))
                        continue
                    if command == 'disc_once':
                        accepted = self.mission.state == 'OBJECT' and self.camera.request_disc()
                        self.telemetry.event('camera', 'disc_request', accepted=bool(accepted))
                        continue
                    if command in ("align_done", "align_cancel") and self.mission.state == "OBJECT":
                        self.focus_ending = True
                        self.camera.finish_focus()
                        continue
                    self._effects(self.mission.command(command, now))
                radar = self.radar.result() if self.configuration.data["radar"]["enabled"] else None
                if self.navigation is not None:
                    self.navigation.tick(radar, now, time.time())
                camera, mode_events = self.camera.result(include_mode_events=True)
                self._send_camera_states(mode_events)
                if previous_role != self.camera.role:
                    last_camera_received, previous_role = now, self.camera.role
                if camera and camera.sequence != last_camera_sequence:
                    if now - camera.timestamp > self.configuration.data["mission"]["sensor_timeout_s"]:
                        raise RuntimeError("CAMERA_FRAME_STALE:" + camera.role)
                    last_camera_sequence = camera.sequence
                    last_camera_received = now
                    if camera.role == "qr" and camera.timestamp > self.qr_cutoff:
                        if camera.qr_text and ']' not in camera.qr_text:
                            self.telemetry.update('mission', qr_text=camera.qr_text, qr_seen_at=time.time())
                        self._effects(self.mission.qr(camera.qr_text))
                    elif camera.role == "object" and self.mission.state == "OBJECT":
                        if now - camera.timestamp <= self.configuration.data["mission"]["sensor_timeout_s"]:
                            self._object_frame(camera)
                if self.camera.role and now - last_camera_received > self.configuration.data["mission"]["sensor_timeout_s"]:
                    raise RuntimeError("CAMERA_STREAM_STALE:" + self.camera.role)
                if self.mission.state == "COLLECT":
                    frame = radar if self.configuration.data["mission"]["prescan_source"] == "radar" else camera
                    if frame and (not hasattr(frame, "role") or frame.role == "prescan"):
                        observations = self.mapper.observations(frame, self.mission.start_id, self.mission.angle)
                        self._effects(self.mission.observe(frame.sequence, frame.timestamp, observations, now))
                self._effects(self.mission.tick(now))
                self.telemetry.update("mission", state=self.mission.state, start=self.mission.start_id,
                                      shots=self.mission.shots, votes=dict(self.mission.votes), error=self.mission.error)
                if self.mission.state == "FAILED":
                    raise RuntimeError(self.mission.error)
                if self.mission.state == "DONE":
                    break
                self.stop_event.wait(.005)
        except Exception as error:
            exit_code = 1
            self.log.exception("application failed: %s", error)
            self.telemetry.update("application", state="failed", error=str(error))
        finally:
            for module in (self.camera, self.radar, self.hmi, self.communication):
                try:
                    module.close()
                    if module is self.camera:
                        self._send_camera_states(self.camera.drain_mode_events())
                except Exception:
                    exit_code = 1
                    self.log.exception("module cleanup failed")
            for number, previous in previous_signals.items():
                signal.signal(number, previous)
            self.telemetry.update("application", state="stopped", exit_code=exit_code)
        return exit_code
