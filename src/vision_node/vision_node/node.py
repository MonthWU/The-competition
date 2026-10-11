import os
import time
from datetime import datetime
import threading
from http import server
from socketserver import ThreadingMixIn

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .Camera import Camera
from .config_from_001 import CAMERA, YOLO, CV, TARGET, TASK, LASER_SWITCH, FIXED_POINT_A
from .visionlib.VisionPipeline import VisionPipeline


class MJPEGHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path in ("/", "/index.html"):
            body = (
                "<html><head><title>RDK Camera</title></head>"
                "<body style='margin:0;background:#111;color:#eee;font-family:Arial;'>"
                "<div style='padding:8px 12px'>RDK Camera Stream</div>"
                "<img src='/stream.mjpg' style='width:100%;max-width:1280px;display:block;margin:0 auto;' />"
                "</body></html>"
            ).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path != "/stream.mjpg":
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
        self.end_headers()

        try:
            while True:
                frame = self.server.frame_provider()
                if frame is None:
                    time.sleep(0.03)
                    continue
                ok, jpg = cv2.imencode('.jpg', frame)
                if not ok:
                    time.sleep(0.03)
                    continue
                data = jpg.tobytes()
                self.wfile.write(b'--FRAME\r\n')
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                self.wfile.write(b'\r\n')
                time.sleep(0.03)
        except Exception:
            pass

    def log_message(self, fmt, *args):
        return


class ThreadingHTTPServer(ThreadingMixIn, server.HTTPServer):
    daemon_threads = True


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        cam_cfg = {
            'id': CAMERA.get('id', '/dev/video0'),
            'candidates': CAMERA.get('candidates', ['/dev/video0']),
            'width': int(CAMERA.get('width', 640)),
            'height': int(CAMERA.get('height', 480)),
            'fps': int(CAMERA.get('fps', 30)),
            'fourcc': CAMERA.get('fourcc', 'MJPG'),
            'buffer_size': int(CAMERA.get('buffer_size', 1)),
            'open_retry': int(CAMERA.get('open_retry', 60)),
            'retry_interval_sec': float(CAMERA.get('retry_interval_sec', 1.0)),
            'warmup_frames': int(CAMERA.get('warmup_frames', 10)),
            'force_resize': bool(CAMERA.get('force_resize', False)),
        }
        self.cam = Camera(cam_cfg)
        self.pipeline = VisionPipeline(
            yolo_cfg=YOLO,
            cv_cfg=CV,
            target_cfg=TARGET,
            task_cfg=TASK,
        )

        self.err_pub = self.create_publisher(String, '/vision/error', 10)
        self.timer = self.create_timer(1.0 / 120.0, self.loop)
        self.frame_count = 0
        self.last_t = time.time()
        self.current_fps = 0.0
        self.last_ax = 0.0
        self.last_ay = 0.0
        self.last_vis_frame = None
        self.last_vis_lock = threading.Lock()
        self.http_server = None
        self.http_thread = None
        self._laser_backend = None
        self._laser_pin = None
        self.get_logger().info('Vision node started')
        self._laser_set(True)
        self._start_http(int(CAMERA.get('web_port', 8000)))
        self._last_debug_save_ts = 0.0

    def _laser_set(self, enable: bool):
        if not LASER_SWITCH.get('enabled', False):
            return
        try:
            import Hobot.GPIO as GPIO
            pin = int(LASER_SWITCH.get('pin', 31))
            active_high = bool(LASER_SWITCH.get('active_high', True))
            level = GPIO.HIGH if (enable == active_high) else GPIO.LOW
            if self._laser_backend is None:
                GPIO.setwarnings(False)
                GPIO.setmode(GPIO.BOARD)
                GPIO.setup(pin, GPIO.OUT, initial=level)
                self._laser_backend = GPIO
                self._laser_pin = pin
            else:
                self._laser_backend.output(self._laser_pin, level)
        except Exception as e:
            self.get_logger().warn(f'laser gpio init failed: {e}')

    def _start_http(self, port):
        def provider():
            with self.last_vis_lock:
                return None if self.last_vis_frame is None else self.last_vis_frame.copy()

        self.http_server = ThreadingHTTPServer(('0.0.0.0', port), MJPEGHandler)
        self.http_server.frame_provider = provider
        self.http_thread = threading.Thread(target=self.http_server.serve_forever, daemon=True)
        self.http_thread.start()
        self.get_logger().info(f'Web stream started: http://0.0.0.0:{port}/')


    def _force_save_debug_frame(self, frame):
        from .config_from_001 import DEBUG
        if frame is None:
            return
        if not DEBUG.get('enable', False):
            return
        if not DEBUG.get('save_visualize', False):
            return
        now = time.time()
        interval = float(DEBUG.get('save_interval_sec', 1.0))
        if now - self._last_debug_save_ts < interval:
            return
        dbg_dir = os.path.join('/root/Monthwu/RDK_vision/RDK_25_E_Done', DEBUG.get('dir', 'debug_img'))
        os.makedirs(dbg_dir, exist_ok=True)
        tag = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        out = os.path.join(dbg_dir, f'{tag}_frame.jpg')
        cv2.imwrite(out, frame)
        self._last_debug_save_ts = now
        if DEBUG.get('print_save', False):
            self.get_logger().info(f'[DEBUG] saved frame -> {out}')

    def _calc_error(self, result, frame_shape):
        h, w = frame_shape[:2]
        if not result or not result.get('frame_found', False):
            return self.last_ax, self.last_ay, 0, 0

        ref = result.get('white_frame_center', result.get('frame_center'))
        if ref is None:
            return self.last_ax, self.last_ay, 0, 0

        cx = int(ref[0])
        cy = int(ref[1])
        img_cx = float(w) / 2.0
        img_cy = float(h) / 2.0
        scale_w = float(result.get('scale_w', result.get('scale', 1.0)))
        scale_h = float(result.get('scale_h', result.get('scale', 1.0)))

        dx_pix = img_cx - float(cx)
        dy_pix = img_cy - float(cy)
        ax = dx_pix * scale_w
        ay = dy_pix * scale_h

        if bool(FIXED_POINT_A.get('invert_x', False)):
            ax = -ax
        if bool(FIXED_POINT_A.get('invert_y', False)):
            ay = -ay
        self.last_ax, self.last_ay = ax, ay
        return ax, ay, cx, cy

    def loop(self):
        ok, frame = self.cam.read()
        if not ok or frame is None:
            self.cam.reopen_if_needed()
            ax, ay, cx, cy = self.last_ax, self.last_ay, 0, 0
            result = None
            vis_frame = None
        else:
            result, vis_frame = self.pipeline.process(frame)
            ax, ay, cx, cy = self._calc_error(result, frame.shape)

        base_frame = vis_frame if vis_frame is not None else (frame if ok and frame is not None else None)
        self._force_save_debug_frame(base_frame)
        with self.last_vis_lock:
            self.last_vis_frame = base_frame

        msg = String()
        msg.data = f'{ax:.4f},{ay:.4f}'
        self.err_pub.publish(msg)

        self.frame_count += 1
        now = time.time()
        if now - self.last_t >= 1.0:
            fps = self.frame_count / (now - self.last_t)
            self.current_fps = fps
            self.get_logger().info(
                f"VISION: fps={fps:.1f}, res={CAMERA.get('width', 0)}x{CAMERA.get('height', 0)}"
            )
            self.frame_count = 0
            self.last_t = now

    def destroy_node(self):
        try:
            self._laser_set(False)
        except Exception:
            pass
        try:
            if self.http_server is not None:
                self.http_server.shutdown()
                self.http_server.server_close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
