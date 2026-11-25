# Purpose: keep resettable QR-preview results separate from mission execution and MCU transmission.
import threading
import time


class QrPreviewState:
    # ***
    # 函数名字：__init__
    # 函数作用：创建扫码预览结果容器，记录初始帧时间门限。
    # 参数意义与范围：
    #   self：当前QrPreviewState实例。
    #   telemetry：本预览会话所属线程安全Telemetry对象。
    # ***
    def __init__(self, telemetry):
        self.telemetry = telemetry
        self.lock = threading.Lock()
        self.cutoff = time.monotonic()
        self.generation = 0
        self.count = 0
        self.text = None
        self._publish(None)

    # ***
    # 函数名字：_publish
    # 函数作用：发布当前识别结果和重置代次，调用者须持有结果锁或尚未共享实例。
    # 参数意义与范围：
    #   self：当前QrPreviewState实例。
    #   seen_at：最近识别的Unix秒时间；None表示本轮尚无结果。
    # ***
    def _publish(self, seen_at):
        self.telemetry.update('mission', state='QR_PREVIEW', qr_text=self.text,
                              recognized_frames=self.count, qr_seen_at=seen_at,
                              qr_reset_enabled=True, reset_generation=self.generation, uart_enabled=False)

    # ***
    # 函数名字：reset
    # 函数作用：原子清空识别结果并拒绝重置之前开始采集的帧，不停止相机。
    # 参数意义与范围：self：当前QrPreviewState实例。
    # ***
    def reset(self):
        with self.lock:
            self.cutoff = time.monotonic()
            self.generation += 1
            self.count, self.text = 0, None
            self._publish(None)
            return {'status': 'reset', 'generation': self.generation}

    # ***
    # 函数名字：observe
    # 函数作用：累计当前重置代次的新鲜扫码内容，返回内容是否发生变化供诊断输出。
    # 参数意义与范围：
    #   self：当前QrPreviewState实例。
    #   frame：CameraFrame；仅接受qr角色、非空文本且采集时间晚于当前重置门限。
    # ***
    def observe(self, frame):
        with self.lock:
            if frame.role != 'qr' or not frame.qr_text or frame.timestamp <= self.cutoff:
                return False
            changed = frame.qr_text != self.text
            self.text = frame.qr_text
            self.count += 1
            self._publish(time.time())
            return changed
