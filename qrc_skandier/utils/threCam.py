"""
filename: ThreadingCam.py
author: Neolux Lee
created: 2024-08-09
last modified: 2026-09-28
descrip: ThreadCap with 首帧强制写入（修"静止场景 Get None Pic"）
version: 1.1
"""

import cv2 as cv
import threading
import numpy as np
from datetime import datetime


class Frame:
    def __init__(self, image, timestamp):
        self.image = image
        self.timestamp = timestamp


class ThreadCap:
    def __init__(self, camera_index=0, width=640, height=400, fps=240):
        self.cap = cv.VideoCapture(camera_index)
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc(*"MJPG"))
        self.cap.set(cv.CAP_PROP_FPS, fps)

        self.frame = None
        self.last_frame = None
        self.lock = threading.Lock()
        self.stop_flag = False

        self.thread = threading.Thread(target=self._update_frame, daemon=True)
        self.thread.start()

    def _update_frame(self):
        # 2026-09-28 修：首帧无条件写入。静止/纯色场景下两张连续帧 MSE 极小，
        # 会被 _are_frames_similar 判为重复，导致 self.frame 永远为 None，
        # 外部 cam.read() 一直拿到 None（qrc_cam 报"Get None Pic"）。
        primed = False
        while not self.stop_flag:
            ret, frame_gray = self.cap.read()
            if not ret:
                continue
            if not primed or not self.last_frame or not self._are_frames_similar(
                    frame_gray, self.last_frame.image
            ):
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                with self.lock:
                    self.frame = Frame(frame_gray, timestamp)
                    self.last_frame = Frame(frame_gray, timestamp)
                primed = True

    def _compute_mse(self, imageA, imageB):
        err = np.sum((imageA.astype("float") - imageB.astype("float")) ** 2)
        err /= float(imageA.shape[0] * imageA.shape[1])
        return err

    def _are_frames_similar(self, frame1, frame2, threshold=10):
        return self._compute_mse(frame1, frame2) < threshold

    def read(self):
        with self.lock:
            frame_copy = self.frame.image.copy() if self.frame is not None else None
            timestamp = self.frame.timestamp if self.frame is not None else None
        return (timestamp, frame_copy)

    def isOpened(self):
        return self.cap.isOpened()

    def release(self):
        """顺序：先 cap.release() 断流，让阻塞在 cap.read() 里的线程立即返回失败，
        否则 thread.join() 可能无限等待。join 另加超时兜底。"""
        self.stop_flag = True
        try:
            self.cap.release()
        except Exception:
            pass
        if self.thread.is_alive():
            self.thread.join(timeout=2.0)


def main():
    cam = ThreadCap(camera_index=0, width=640, height=400)
    fourcc = cv.VideoWriter.fourcc(*"XVID")
    out = cv.VideoWriter("output.avi", fourcc, 30.0, (640, 400))
    try:
        while True:
            timestamp, frame = cam.read()
            if frame is not None:
                cv.imshow("Camera Frame", frame)
                print(f"Timestamp: {timestamp}")
                if cv.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        out.release()
        cam.release()
        cv.destroyAllWindows()


if __name__ == "__main__":
    main()