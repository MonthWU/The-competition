"""USB Camera wrapper for RDK_25_E_006
关键修正：只使用显式候选，不再自动扫 /dev/video*，避免反复误碰 /dev/video1 元数据节点
"""
import os, re, time
import cv2

class Camera:
    def __init__(self, cfg):
        self.cfg = cfg
        self.cap = None
        self.opened_source = None
        self.width = int(cfg.get('width', 640))
        self.height = int(cfg.get('height', 480))
        self.fps = int(cfg.get('fps', 30))
        self.fourcc_name = cfg.get('fourcc', 'MJPG')
        self.open()

    @staticmethod
    def _dev_to_index(source):
        if isinstance(source, int):
            return source, str(source)
        if isinstance(source, str) and source.isdigit():
            return int(source), source
        if isinstance(source, str):
            real = os.path.realpath(source)
            m = re.search(r'/dev/video(\d+)$', real)
            if m:
                return int(m.group(1)), f'{source} -> {real}'
        return source, str(source)

    def _candidate_list(self):
        raw = []
        cam_id = self.cfg.get('id', None)
        if cam_id is not None:
            raw.append(cam_id)
        raw.extend(self.cfg.get('candidates', ['/dev/video0']))
        candidates, seen = [], set()
        for src in raw:
            ns, label = self._dev_to_index(src)
            key = str(ns)
            if key in seen:
                continue
            seen.add(key)
            candidates.append((ns, label))
        return candidates

    def _apply_props(self, cap):
        if self.fourcc_name:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc_name))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        bs = int(self.cfg.get('buffer_size', 1))
        if bs > 0:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, bs)

    def open(self):
        retry = int(self.cfg.get('open_retry', 30))
        interval = float(self.cfg.get('retry_interval_sec', 1.0))
        for i in range(retry):
            for src, label in self._candidate_list():
                cap = cv2.VideoCapture(src, cv2.CAP_V4L2)
                if not cap.isOpened():
                    cap.release()
                    continue
                self._apply_props(cap)
                time.sleep(0.5)
                ok, frame = False, None
                warmup = max(1, int(self.cfg.get('warmup_frames', 5)))
                for _ in range(warmup):
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        ok = True
                        break
                    time.sleep(0.01)
                if ok and frame is not None:
                    self.cap = cap
                    self.opened_source = src
                    print(f'[CAMERA] opened {label}, shape={frame.shape}')
                    return True
                cap.release()
            print(f'[CAMERA] waiting for camera... retry {i+1}/{retry}')
            time.sleep(interval)
        raise RuntimeError('Camera open failed on explicit candidates')

    def read(self):
        if self.cap is None or not self.cap.isOpened():
            return False, None
        ok, frame = self.cap.read()
        if not ok or frame is None:
            return False, None
        return True, frame

    def reopen_if_needed(self):
        self.release()
        try:
            return self.open()
        except Exception:
            return False

    def release(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None
