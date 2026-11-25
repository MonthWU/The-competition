# Purpose: own one camera, run YOLO, search the disc until found after MCU confirmation, and switch target profiles.
import logging
from collections import deque
import subprocess
import threading
import time
import math

from ..types import CameraFrame
from .detector import BpuDetector
from .processing import filter_objects
from .modes import object_capture_settings, capture_signature
from .focus import alignment_target, map_seed, CircleFinder, target_geometry, line_angle
from .disc import DiscFinder, draw_disc


class CameraModule:
    """One camera owner. Role changes close and join before opening another device."""

    # ***
    # 函数名字：__init__
    # 函数作用：初始化当前类的依赖、配置和内部状态。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   configuration：项目Configuration对象；范围为已解析且满足对应模式约束的配置。
    #   telemetry：共享Telemetry对象；范围为当前应用初始化的线程安全状态容器。
    # ***
    def __init__(self, configuration, telemetry):
        self.configuration = configuration
        self.telemetry = telemetry
        self.log = logging.getLogger("car26.camera")
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.thread = None
        self.latest = None
        self.pending_qr = None
        self.error = None
        self.sequence = 0
        self.role = None
        self.finish_focus_event = threading.Event()
        self.recognition_mode = None
        self.mode_events = deque()
        self.disc_request = None
        self.pending_disc = None
        self.disc_request_sequence = 0
        self.disc_finder = None
        self.disc_processed_frames = 0
        self.capture_profile = None
        self.latest_yolo = None
        self.focus_request = None
        self.focus_request_sequence = 0

    # ***
    # 函数名字：_set_recognition_mode
    # 函数作用：按真实模式变化记录进入/退出事件，预留退出容量并拒绝静默丢失通知。
    # 参数意义与范围：
    #   self：当前CameraModule实例；通常由唯一相机线程调用。
    #   mode：qr、block、target或None；None表示当前没有进入识别功能。
    # ***
    def _set_recognition_mode(self, mode):
        if mode not in (None, "qr", "block", "target"):
            raise ValueError("INVALID_OBJECT_RECOGNITION_MODE")
        with self.lock:
            if mode == self.recognition_mode:
                return
            changes = []
            if self.recognition_mode is not None:
                changes.append((self.recognition_mode, False))
            if mode is not None:
                changes.append((mode, True))
            if len(self.mode_events) + len(changes) > (64 if mode is None else 63):
                raise RuntimeError("CAMERA_STATE_QUEUE_FULL")
            self.mode_events.extend(changes)
            self.recognition_mode = mode
            if mode != 'block':
                self.disc_request = self.pending_disc = None
                self.focus_request = self.latest_yolo = None
        self.telemetry.update("camera", recognition_mode=mode)

    # ***
    # 函数名字：request_disc
    # 函数作用：仅在真实物料识别阶段登记一个新圆盘请求，忙时合并重复确认。
    # 参数意义与范围：self：当前CameraModule实例；返回bool表示本次确认是否被接受。
    # ***
    def request_disc(self):
        with self.lock:
            if (self.recognition_mode != 'block'
                    or not self.configuration.data.get('disc_centering', {}).get('enabled', False)
                    or self.focus_request is not None
                    or self.disc_request is not None or self.pending_disc is not None):
                return False
            self.disc_request_sequence += 1
            self.disc_request = (self.disc_request_sequence, time.monotonic())
            self.latest_yolo = None
        return True

    # ***
    # 函数名字：request_focus
    # 函数作用：接收0x62进入请求，冻结最近一次新鲜且包含三个唯一标靶的YOLO结果。
    # 参数意义与范围：self：当前CameraModule实例；返回bool表示是否接受请求。
    # ***
    def request_focus(self):
        focus = self.configuration.data.get('target_focus', {})
        with self.lock:
            snapshot = self.latest_yolo
            if (self.recognition_mode != 'block' or not focus.get('enabled', False)
                    or focus.get('trigger', 'auto') != 'command'
                    or self.disc_request is not None or self.pending_disc is not None
                    or self.focus_request is not None or snapshot is None):
                return False
            if not 0 <= time.monotonic()-snapshot[1] <= focus.get('seed_max_age_s', 1.0):
                return False
            angle, marks = target_geometry(snapshot[4])
            if angle is None or len(marks) != 3:
                return False
            self.focus_request_sequence += 1
            self.finish_focus_event.clear()
            self.focus_request = (self.focus_request_sequence, time.monotonic(), snapshot)
        return True

    # ***
    # 函数名字：_measure_disc
    # 函数作用：只在圆盘采集档位逐帧检测至有效检出，保留未完成请求并返回完成编号。
    # 参数意义与范围：
    #   self：当前CameraModule实例。
    #   image：当前完整采集BGR图像。
    #   captured：该帧read之前的单调时间，非负秒数。
    # ***
    def _measure_disc(self, image, captured):
        with self.lock:
            request = self.disc_request
            if (self.recognition_mode != 'block' or self.capture_profile != 'disc'
                    or request is None or captured <= request[1]):
                return 0, None, None, 0.0
        settings = self.configuration.data['disc_centering']
        begin = time.monotonic()
        if self.disc_finder is None:
            self.disc_finder = DiscFinder(self.configuration.path(settings['native_library']),
                                          self.configuration.path(settings['parameter_file']))
        self.disc_processed_frames += 1
        result = self.disc_finder.find(image)
        delta = None
        if result is not None:
            offset = result['offset']
            if len(offset) == 2 and all(math.isfinite(value) and -32768 <= round(value) <= 32767 for value in offset):
                delta = tuple(round(value) for value in offset)
            else:
                result = None
        return request[0] if delta is not None else 0, delta, result, (time.monotonic()-begin)*1000

    # ***
    # 函数名字：drain_mode_events
    # 函数作用：取出并清空待发送的真实状态事件，供主线程在关闭设备后发送退出通知。
    # 参数意义与范围：
    #   self：当前CameraModule实例；返回最多64个(mode, active)事件。
    # ***
    def drain_mode_events(self):
        with self.lock:
            events = tuple(self.mode_events)
            self.mode_events.clear()
            return events

    # ***
    # 函数名字：finish_focus
    # 函数作用：请求结束精定位并清除待发送的旧相机结果。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    # ***
    def finish_focus(self):
        self.finish_focus_event.set()
        with self.lock:
            self.latest = None
            self.focus_request = None

    # ***
    # 函数名字：_open_capture
    # 函数作用：由唯一相机线程打开设备，设置有界缓冲并核对显式缓冲配置。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   settings：已校验的采集配置，buffer_count为1～4整数，缺省为1以兼容旧相机。
    # ***
    def _open_capture(self, settings):
        import cv2
        cap = cv2.VideoCapture(settings["device"], cv2.CAP_V4L2)
        try:
            if not cap.isOpened():
                raise RuntimeError("CAMERA_OPEN_FAILED:" + self.role)
            for key, value in ((cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*settings["fourcc"])),
                (cv2.CAP_PROP_FRAME_WIDTH, settings["width"]), (cv2.CAP_PROP_FRAME_HEIGHT, settings["height"]),
                (cv2.CAP_PROP_FPS, settings["fps"])):
                cap.set(key, value)
            buffers = settings.get('buffer_count', 1)
            accepted = cap.set(cv2.CAP_PROP_BUFFERSIZE, buffers)
            if 'buffer_count' in settings:
                if not accepted or cap.get(cv2.CAP_PROP_BUFFERSIZE) != buffers:
                    raise RuntimeError('CAMERA_BUFFER_COUNT_REJECTED:' + self.role)
            for name, value in settings.get("controls", {}).items():
                prop = {"brightness": cv2.CAP_PROP_BRIGHTNESS, "gain": cv2.CAP_PROP_GAIN,
                        "exposure": cv2.CAP_PROP_EXPOSURE}.get(name)
                if prop is None or not cap.set(prop, value):
                    raise RuntimeError("CAMERA_CONTROL_REJECTED:" + name)
            return cap
        except Exception:
            cap.release()
            raise

    # ***
    # 函数名字：_stream_controls
    # 函数作用：首帧出图后设置并回读UVC曝光、亮度和增益。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   settings：当前模块配置字典；键、类型及数值范围由Configuration.validate约束。
    # ***
    def _stream_controls(self, settings):
        controls = settings.get("uvc_controls", {})
        if not controls:
            return
        device = settings["device"]
        subprocess.run(["v4l2-ctl", "-d", device, "--set-ctrl=auto_exposure=3"], check=True, capture_output=True, timeout=2)
        subprocess.run(["v4l2-ctl", "-d", device, "--set-ctrl=auto_exposure=1"], check=True, capture_output=True, timeout=2)
        values = ",".join(name+"="+str(value) for name, value in controls.items())
        subprocess.run(["v4l2-ctl", "-d", device, "--set-ctrl="+values], check=True, capture_output=True, timeout=2)
        readback = subprocess.run(["v4l2-ctl", "-d", device, "--get-ctrl="+",".join(controls)],
                                  check=True, capture_output=True, text=True, timeout=2)
        actual = {}
        for line in readback.stdout.splitlines():
            if ":" not in line:
                continue
            name, raw = line.split(":", 1)
            try:
                actual[name.strip()] = int(raw.split()[0])
            except (ValueError, IndexError) as error:
                raise RuntimeError("CAMERA_UVC_READBACK_INVALID:" + name.strip()) from error
        if any(actual.get(name) != value for name,value in controls.items()):
            raise RuntimeError("CAMERA_UVC_READBACK_MISMATCH")

    # ***
    # 函数名字：_switch_object_mode
    # 函数作用：切换物块/圆盘/标靶档位，圆盘保持0x61业务状态，不丢确认与待回复事件。
    # 参数意义与范围：
    #   self：当前CameraModule实例，调用者必须是唯一相机线程。
    #   cap：当前已打开的VideoCapture对象；重开失败时由外层异常处理终止采集。
    #   current：当前生效的采集参数字典。
    #   mode：目标采集档位，block、disc或target；disc属于block业务功能。
    # ***
    def _switch_object_mode(self, cap, current, mode):
        desired = object_capture_settings(self.configuration.data["cameras"]["object"], mode)
        reopened = capture_signature(current) != capture_signature(desired)
        with self.lock:
            self.latest = None
            self.latest_yolo = None
        business_mode = 'target' if mode == 'target' else 'block'
        changed = business_mode != self.recognition_mode
        if changed:
            self._set_recognition_mode(None)
        self.telemetry.update("camera", state="switching", focus_delta=None)
        if reopened:
            self.telemetry.clear_image(self.role)
            cap.release()
            cap = self._open_capture(desired)
        try:
            self._set_recognition_mode(business_mode)
        except Exception:
            if reopened:
                cap.release()
            raise
        self.capture_profile = mode
        self.telemetry.event("camera", "recognition_mode", mode=business_mode, capture_profile=mode, reopened=reopened)
        self.telemetry.update("camera", state="ready", capture_width=desired["width"],
                              capture_height=desired["height"], capture_fps=desired["fps"],
                              capture_buffer_count=desired.get('buffer_count', 1), capture_profile=mode)
        return cap, desired, reopened

    # ***
    # 函数名字：switch
    # 函数作用：关闭旧相机角色并启动新角色的唯一采集线程。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   role：相机角色字符串；范围为qr、object或prescan。
    # ***
    def switch(self, role):
        if self.role == role and self.thread and self.thread.is_alive():
            return
        self.close()
        with self.lock:
            self.latest = None
            self.error = None
        self.role = role
        self.finish_focus_event.clear()
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._capture, args=(role,), name="camera-" + role)
        self.thread.start()

    # ***
    # 函数名字：result
    # 函数作用：优先消费单槽圆盘或二维码事件，否则读取最新快照，并传播设备错误。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   include_mode_events：是否原子取出帧和待发送的模式事件；布尔值，默认False只返回帧。
    # ***
    def result(self, include_mode_events=False):
        with self.lock:
            if self.error:
                raise RuntimeError(self.error)
            if self.pending_disc is not None:
                result = self.pending_disc
                self.pending_disc = None
            elif self.pending_qr is not None:
                result = self.pending_qr
                self.pending_qr = None
            else:
                result = self.latest
            if include_mode_events:
                events = tuple(self.mode_events)
                self.mode_events.clear()
                return result, events
            return result

    # ***
    # 函数名字：_decode_qr
    # 函数作用：默认整帧扫码；跳过含帧尾]或非法UTF-8的候选，保留首个有效原文。
    # 参数意义与范围：
    #   self：当前CameraModule实例。
    #   detector：当前QR角色的OpenCV QRCodeDetector对象。
    #   image：当前采集的完整非空BGR图像；roi为null或缺省时直接传给解码器。
    #   zbar_decode：可选备用解码函数；None表示不可用，否则返回带type和data的候选序列。
    # ***
    def _decode_qr(self, detector, image, zbar_decode):
        import cv2

        roi = self.configuration.data["cameras"]["qr"].get("roi")
        if roi is not None:
            x, y, width, height = roi
            if (x < 0 or y < 0 or width <= 0 or height <= 0
                    or x + width > image.shape[1] or y + height > image.shape[0]):
                raise ValueError("QR_ROI_OUT_OF_FRAME")
            image = image[y:y + height, x:x + width]
        try:
            text, _, _ = detector.detectAndDecode(image)
        except UnicodeDecodeError:
            self.telemetry.event("camera", "qr_invalid_utf8", decoder="opencv")
            text = ""
        except cv2.error as error:
            if 'Invalid QR code source points' not in str(error):
                raise
            # 退化四边形只拒绝当前帧，允许备用解码及后续帧继续运行。
            self.telemetry.event("camera", "qr_invalid_geometry", decoder="opencv")
            text = ""
        if text and "]" not in text:
            return text
        if text:
            self.telemetry.event("camera", "qr_invalid_terminator", decoder="opencv")
        if zbar_decode is not None:
            for code in zbar_decode(image):
                if code.type != "QRCODE":
                    continue
                try:
                    text = code.data.decode("utf-8")
                except UnicodeDecodeError:
                    self.telemetry.event("camera", "qr_invalid_utf8", decoder="pyzbar")
                    continue
                if text and "]" not in text:
                    return text
                if text:
                    self.telemetry.event("camera", "qr_invalid_terminator", decoder="pyzbar")
        return ""

    # ***
    # 函数名字：_capture
    # 函数作用：唯一线程拥有相机，以单帧预取重叠BPU等待，并执行圆盘门控及标靶模式切换。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   role：相机角色字符串；范围为qr、object或prescan。
    # ***
    def _capture(self, role):
        self.role = role
        settings = self.configuration.data["cameras"][role]
        base_settings = settings
        if role == "object":
            settings = object_capture_settings(base_settings, "block")
        cap = writer = detector = None
        prefetched = None
        try:
            import cv2
            cap = self._open_capture(settings)
            controls_pending = True
            settle_frames = 0
            focus = self.configuration.data.get("target_focus", {})
            focus_enabled = role == "object" and focus.get("enabled", False)
            focus_trigger = focus.get('trigger', 'auto')
            disc_settings = self.configuration.data.get("disc_centering", {})
            disc_enabled = role == 'object' and disc_settings.get('enabled', False)
            active, armed, finder, seed = False, True, None, None
            circle_finders, circle_seeds = (), ()
            focus_exit_reason = None
            started = last_circle = 0.0
            transition_started = None
            transition_ms = None
            detector = None if role == "qr" else BpuDetector(
                self.configuration, self.configuration.data["models"][settings["model"]])
            qr = cv2.QRCodeDetector() if role == "qr" else None
            try:
                from pyzbar.pyzbar import decode as zbar_decode
            except (ImportError, OSError):
                zbar_decode = None
            self.telemetry.update("camera", state="ready", role=role, device=settings["device"],
                                  recognition_mode="block" if role == "object" else None,
                                  capture_width=settings["width"], capture_height=settings["height"],
                                  capture_fps=settings["fps"],
                                  capture_buffer_count=settings.get('buffer_count', 1),
                                  capture_profile='block' if role == 'object' else role,
                                  qr_roi=(settings.get("roi") or [0, 0, settings["width"], settings["height"]])
                                  if role == "qr" else None)
            if role == "object":
                self.capture_profile = 'block'
                self._set_recognition_mode("block")
            elif role == "qr":
                self._set_recognition_mode("qr")
            last_preview = 0.0
            recording_started = None
            recording_complete = False

            # ***
            # 函数名字：prefetch_next
            # 函数作用：在本相机线程读取唯一下一帧并保存真实read开始时间，不覆盖当前处理帧。
            # 参数意义与范围：无。
            # ***
            def prefetch_next():
                nonlocal prefetched
                if self.stop_event.is_set():
                    return
                if prefetched is not None:
                    raise RuntimeError('CAMERA_PREFETCH_SLOT_FULL')
                stamp = time.monotonic()
                ok, next_image = cap.read()
                if not ok:
                    raise RuntimeError('CAMERA_READ_FAILED:' + role)
                if next_image.shape[:2] != (settings['height'], settings['width']):
                    raise RuntimeError('CAMERA_DIMENSIONS_MISMATCH:' + role)
                prefetched = (next_image, stamp)

            # ***
            # 函数名字：enter_focus
            # 函数作用：用指定YOLO快照创建三个局部跟踪器，清空预取并切换标靶采集档位。
            # 参数意义与范围：marks为三个唯一有效标靶Detection；source为快照图像的正整数(width,height)。
            # ***
            def enter_focus(marks, source):
                nonlocal cap, settings, controls_pending, settle_frames, active, armed
                nonlocal started, last_circle, transition_started, writer, recording_complete
                nonlocal prefetched, circle_finders, circle_seeds, seed, focus_exit_reason
                target_settings = object_capture_settings(base_settings, 'target')
                destination = (target_settings['width'], target_settings['height'])
                same_stream = capture_signature(settings) == capture_signature(target_settings)
                transform = (1.0,1.0,0.0,0.0) if same_stream else focus['mode_transform']
                tracking_settings = dict(focus, mode_transform=transform)
                circle_finders = tuple(CircleFinder(self.configuration.path(focus['native_library']),tracking_settings) for _ in marks)
                for tracker, mark in zip(circle_finders, marks):
                    tracker.set_target_scale(mark, source, destination)
                circle_seeds = tuple(map_seed(mark, source, destination, transform) for mark in marks)
                seed = circle_seeds[1]
                active, armed, focus_exit_reason = True, False, None
                started, last_circle = time.monotonic(), None
                transition_started = started
                if writer is not None:
                    writer.release()
                    writer, recording_complete = None, True
                with self.lock:
                    self.latest = self.latest_yolo = None
                prefetched = None
                cap, settings, reopened = self._switch_object_mode(cap, settings, 'target')
                controls_pending = reopened
                settle_frames = focus['settle_frames'] if reopened else 0
                self.telemetry.event('camera', 'focus_enter', seed=seed, source=source)
                self.telemetry.update('camera', vision_mode='circle', focus_delta=None,
                                      disc=None, disc_detected=False, disc_state='inactive',disc_aligned=False)

            while not self.stop_event.is_set():
                begin = time.monotonic()
                if active and (focus_exit_reason is not None or self.finish_focus_event.is_set() or begin-started >= focus["maximum_duration_s"]
                               or (last_circle is not None and begin-last_circle >= focus["lost_timeout_s"])):
                    reason = ("mcu" if self.finish_focus_event.is_set() else
                              focus_exit_reason if focus_exit_reason else
                              "timeout" if begin-started >= focus["maximum_duration_s"] else "circle_lost")
                    self.finish_focus_event.clear()
                    active, armed = False, False
                    focus_exit_reason = None
                    transition_started = time.monotonic()
                    with self.lock:
                        self.latest = None
                    self.telemetry.event("camera", "focus_exit", reason=reason)
                    prefetched = None
                    cap, settings, reopened = self._switch_object_mode(cap, settings, "block")
                    controls_pending = reopened
                    settle_frames = focus["settle_frames"] if reopened else 0
                    self.telemetry.update("camera", vision_mode="yolo", focus_delta=None)
                    begin = time.monotonic()
                if role == 'object' and not active:
                    with self.lock:
                        request = self.focus_request
                    if request is not None:
                        snapshot = request[2]
                        age = time.monotonic()-snapshot[1]
                        angle, marks = target_geometry(snapshot[4])
                        if (focus_enabled and focus_trigger == 'command'
                                and not self.finish_focus_event.is_set()
                                and self.disc_request is None and self.pending_disc is None
                                and 0 <= age <= focus.get('seed_max_age_s', 1.0)
                                and angle is not None and len(marks) == 3):
                            self.telemetry.event('camera','focus_seed',request_id=request[0],
                                                 yolo_sequence=snapshot[0], yolo_timestamp=snapshot[1])
                            enter_focus(marks, (snapshot[2], snapshot[3]))
                            continue
                        with self.lock:
                            if self.focus_request is request:
                                self.focus_request = None
                        self.telemetry.event('camera','focus_request_rejected',reason='stale_busy_or_cancelled')
                    desired_profile = 'disc' if self.disc_request is not None else 'block'
                    if self.capture_profile != desired_profile:
                        prefetched = None
                        cap, settings, reopened = self._switch_object_mode(cap, settings, desired_profile)
                        controls_pending = reopened
                        settle_frames = focus.get('settle_frames', 3) if reopened else 0
                        begin = time.monotonic()
                # A confirmation must consume a new post-request frame; a slot
                # captured before that request is discarded rather than relabeled.
                if self.disc_request is not None or self.pending_disc is not None:
                    prefetched = None
                if prefetched is None:
                    captured = time.monotonic()
                    ok, image = cap.read()
                else:
                    image, captured = prefetched
                    prefetched = None
                    ok = True
                if not ok:
                    raise RuntimeError("CAMERA_READ_FAILED:" + role)
                height, width = image.shape[:2]
                if (width, height) != (settings["width"], settings["height"]):
                    raise RuntimeError("CAMERA_DIMENSIONS_MISMATCH:" + role)
                if controls_pending:
                    self._stream_controls(settings)
                    controls_pending = False
                    if settings.get("uvc_controls"):
                        settle_frames = max(settle_frames, 3)
                if settle_frames:
                    settle_frames -= 1
                    continue
                capture_image = image
                disc_result,disc_ms = None,0.0
                disc_request_id, disc_delta = 0, None
                disc_size = (width,height)
                if role == "object" and not active:
                    image = cv2.resize(image, (base_settings["width"], base_settings["height"]),
                                       interpolation=cv2.INTER_AREA)
                    height, width = image.shape[:2]
                text, detections = "", ()
                delta = circle = alpha = None
                if qr:
                    text = self._decode_qr(qr, image, zbar_decode)
                elif active:
                    if last_circle is None:
                        last_circle = begin
                    circles = [tracker.find(image, point) for tracker,point in zip(circle_finders,circle_seeds)]
                    circle_seeds = tuple(value[:2] if value is not None else old for value,old in zip(circles,circle_seeds))
                    circle = circles[1]
                    if all(value is not None for value in circles):
                        alpha = line_angle([value[:2] for value in circles])
                    if alpha is not None and abs(alpha) > focus["angle_threshold_deg"] + 1e-9:
                        focus_exit_reason = "angle_outside_threshold"
                        continue
                    if alpha is not None and circle is not None:
                        seed = circle[:2]
                        last_circle = time.monotonic()
                        delta = (int(round(seed[0]-width/2)), int(round(seed[1]-height/2)))
                else:
                    if focus_trigger == 'auto':
                        self.finish_focus_event.clear()
                    can_prefetch = (role == 'object' and getattr(detector, 'supports_prefetch', False) is True
                                    and self.disc_request is None and self.pending_disc is None)
                    detections = detector.detect(image, prefetch=prefetch_next) if can_prefetch else detector.detect(image)
                    if role == "object":
                        detections = filter_objects(detections, height,
                                                    self.configuration.data["camera_processing"])
                        if disc_enabled:
                            disc_request_id, disc_delta, disc_result, disc_ms = self._measure_disc(capture_image, captured)
                    if role == "object":
                        alpha, marks = target_geometry(detections)
                        with self.lock:
                            if self.capture_profile == 'block' and self.disc_request is None and self.pending_disc is None:
                                self.latest_yolo = (self.sequence+1,captured,width,height,tuple(detections))
                    target = alignment_target(detections, (width,height), focus["angle_threshold_deg"]) if focus_enabled and focus_trigger == 'auto' else None
                    if target is None:
                        armed = True
                    elif armed and not disc_request_id and self.disc_request is None and self.pending_disc is None:
                        enter_focus(marks, (width, height))
                        continue
                if active and self.finish_focus_event.is_set():
                    continue
                self.sequence += 1
                result = CameraFrame(role, self.sequence, captured, width, height, detections, text, active, delta, alpha,
                                     disc_request_id, disc_delta)
                with self.lock:
                    self.latest = result
                    if disc_request_id:
                        self.pending_disc = result
                        self.disc_request = None
                    # 扫码事件独立保留原帧序号和时间戳，后续视频帧不能覆盖。
                    if role == "qr" and text and self.pending_qr is None:
                        self.pending_qr = result
                recording = self.configuration.data["recording"]
                if recording["enabled"] and not recording_complete and not active:
                    if writer is None:
                        directory = self.configuration.path(recording["directory"])
                        directory.mkdir(parents=True, exist_ok=True)
                        file = directory / (time.strftime("%Y%m%d-%H%M%S") + "-" + role + ".avi")
                        recording_started = time.monotonic()
                        writer = cv2.VideoWriter(str(file), cv2.VideoWriter_fourcc(*"MJPG"),
                                                 settings["fps"], (width, height))
                        if not writer.isOpened():
                            raise RuntimeError("VIDEO_WRITER_OPEN_FAILED")
                    writer.write(image)
                    if time.monotonic() - recording_started >= recording["maximum_seconds"]:
                        writer.release()
                        writer = None
                        recording_complete = True
                        self.telemetry.event("camera", "recording_limit", role=role)
                now = time.monotonic()
                hmi = self.configuration.data["hmi"]
                if self.configuration.mode == 'debug' and hmi["enabled"] and now - last_preview >= 1 / hmi["preview_fps"]:
                    preview = image.copy()
                    for d in detections:
                        cv2.rectangle(preview, (d.x1, d.y1), (d.x2, d.y2), (0, 220, 0), 2)
                        cv2.putText(preview, d.label, (d.x1, max(15, d.y1)),
                                    cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 220, 0), 1)
                    if disc_request_id:
                        draw_disc(preview,disc_result,disc_size)
                    if circle is not None:
                        cv2.circle(preview, (round(circle[0]),round(circle[1])), round(circle[2]), (0,220,0), 2)
                        cv2.drawMarker(preview, (width//2,height//2), (0,0,255))
                        cv2.putText(preview, str(delta), (15,30), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,220,0), 2)
                    if alpha is not None:
                        cv2.putText(preview, "alpha=%.2f deg" % alpha, (15,60), cv2.FONT_HERSHEY_SIMPLEX, .6, (0,220,0), 2)
                    encoded, jpeg = cv2.imencode(".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, 75])
                    if encoded:
                        self.telemetry.image(role, jpeg.tobytes())
                    last_preview = now
                duration = time.monotonic() - begin
                if transition_started is not None and (not active or delta is not None):
                    transition_ms = round((time.monotonic()-transition_started)*1000, 2)
                    self.telemetry.event("camera", "focus_ready" if active else "yolo_ready",
                                         transition_ms=transition_ms, valid=delta is not None if active else True)
                    transition_started = None
                inference = getattr(detector, 'metrics', {})
                if not isinstance(inference, dict):
                    inference = {}
                self.telemetry.update("camera", role=role, sequence=self.sequence,
                                      frame_timestamp=captured, processing_ms=round(duration * 1000, 2),
                                      inference_overlap=inference.get('overlapped', False) if not active else False,
                                      inference_forward_ms=inference.get('forward_ms') if not active else None,
                                      inference_outputs_ms=inference.get('outputs_ms') if not active else None,
                                      inference_wait_ms=inference.get('wait_ms') if not active else None,
                                      detections=len(detections), width=width, height=height,
                                      vision_mode="qr" if role == "qr" else "circle" if active else "yolo",
                                      transition_ms=transition_ms,
                                      focus_delta=delta,target_alpha=alpha,
                                      disc=disc_result,disc_detected=disc_result is not None,
                                      disc_image_size=disc_size if disc_request_id else None,
                                      disc_aligned=False,
                                      disc_state=('inactive' if active else 'measured' if disc_delta is not None
                                                  else 'searching' if self.disc_request is not None else 'waiting_request')
                                      if disc_enabled else 'disabled',
                                      disc_request_id=disc_request_id,
                                      disc_processed_frames=self.disc_processed_frames,
                                      disc_processing_ms=round(disc_ms,2))
                self.log.debug("role=%s sequence=%s processing_ms=%.2f", role, self.sequence, duration * 1000)
        except Exception as error:
            with self.lock:
                self.error = str(error)
            self.telemetry.update("camera", state="failed", error=str(error),disc=None,disc_detected=False)
            self.log.exception("camera failed")
        finally:
            if detector is not None and callable(getattr(detector, 'close', None)):
                try:
                    detector.close()
                except Exception as error:
                    with self.lock:
                        if self.error is None:
                            self.error = 'INFERENCE_CLOSE_FAILED:' + str(error)
                    self.telemetry.update('camera', state='failed', error=self.error)
                    self.log.exception('inference close failed')
            if writer is not None:
                writer.release()
            if cap is not None:
                cap.release()
            if role in ("object", "qr"):
                self._set_recognition_mode(None)

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
                raise RuntimeError("CAMERA_STOP_TIMEOUT")
        self.thread = None
        self.telemetry.clear_image(self.role)
        self.role = None
        with self.lock:
            self.latest = None
            self.pending_qr = None
            self.disc_request = self.pending_disc = None
            self.focus_request = self.latest_yolo = None
            self.capture_profile = None
        self.telemetry.update("camera", state="stopped",disc=None,disc_detected=False,disc_aligned=False,disc_state='inactive')
