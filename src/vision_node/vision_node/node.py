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
from .config_from_001 import CAMERA, CV, LASER_SWITCH


class MJPEGHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path in ("/", "/index.html"):
            body = (
                "<html><head><title>V02 Vision</title></head>"
                "<body style='margin:0;background:#111;color:#eee;font-family:Arial;'>"
                "<div style='padding:8px 12px'>V02 Vision Stream</div>"
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
            'fps': int(CAMERA.get('fps', 120)),
            'fourcc': CAMERA.get('fourcc', 'MJPG'),
            'buffer_size': int(CAMERA.get('buffer_size', 1)),
            'open_retry': int(CAMERA.get('open_retry', 60)),
            'retry_interval_sec': float(CAMERA.get('retry_interval_sec', 1.0)),
            'warmup_frames': int(CAMERA.get('warmup_frames', 10)),
            'force_resize': bool(CAMERA.get('force_resize', False)),
        }
        self.cam = Camera(cam_cfg)

        self.target_pub = self.create_publisher(String, '/vision/target', 10)
        self.timer = self.create_timer(1.0 / 120.0, self.loop)
        self.frame_count = 0
        self.last_t = time.time()
        self.current_fps = 0.0
        self.last_d = 0.0
        self.last_ax = 0.0
        self.last_vis_frame = None
        self.last_vis_lock = threading.Lock()
        self.http_server = None
        self.http_thread = None
        self._laser_backend = None
        self._laser_pin = None
        self.get_logger().info('V02 Vision node started (YOLO disabled, target=d,a_x)')
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
        if frame is None:
            return
        dbg_dir = os.path.join('/root/Monthwu/RDK_vision/V02', 'debug_img')
        os.makedirs(dbg_dir, exist_ok=True)

    def _detect_red_circle(self, frame):
        if frame is None or frame.size == 0:
            return None

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red1 = (0, 90, 70)
        upper_red1 = (12, 255, 255)
        lower_red2 = (165, 90, 70)
        upper_red2 = (180, 255, 255)

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        best = None
        best_score = -1.0
        h, w = frame.shape[:2]
        img_cx = w / 2.0
        ref_distance_cm = 250.0
        ref_radius_px = 40.0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 80:
                continue
            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            if radius < 6:
                continue
            peri = cv2.arcLength(cnt, True)
            circularity = 0.0 if peri <= 1e-6 else (4.0 * 3.1415926 * area / (peri * peri))
            score = area + radius * 5.0 + circularity * 80.0
            if score > best_score:
                best_score = score
                a_x = float(cx - img_cx)
                d = ref_distance_cm * (ref_radius_px / max(radius, 1e-6))
                best = {
                    'cx': float(cx),
                    'cy': float(cy),
                    'radius': float(radius),
                    'd': float(d),
                    'a_x': float(a_x),
                }

        return best

    def _annotate(self, frame, target):
        if frame is None:
            return frame
        vis = frame.copy()
        h, w = vis.shape[:2]

        # 更醒目的右上角 HUD
        title = 'V02 HUD'
        text1 = f'FPS: {self.current_fps:.1f}'
        text2 = f'RES: {w}x{h}'
        font = cv2.FONT_HERSHEY_SIMPLEX
        title_scale = 0.8
        body_scale = 0.8
        thick = 2
        (ttw, tth), _ = cv2.getTextSize(title, font, title_scale, thick)
        (tw1, th1), _ = cv2.getTextSize(text1, font, body_scale, thick)
        (tw2, th2), _ = cv2.getTextSize(text2, font, body_scale, thick)
        panel_w = max(ttw, tw1, tw2) + 36
        panel_h = tth + th1 + th2 + 42
        x0 = max(0, w - panel_w - 8)
        y0 = 8
        x1 = w - 8
        y1 = y0 + panel_h

        overlay = vis.copy()
        cv2.rectangle(overlay, (x0, y0), (x1, y1), (10, 10, 10), -1)
        vis = cv2.addWeighted(overlay, 0.68, vis, 0.32, 0)
        cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 255, 255), 3)
        cv2.putText(vis, title, (x0 + 12, y0 + tth + 6), font, title_scale, (0, 255, 255), thick)
        cv2.putText(vis, text1, (x0 + 12, y0 + tth + th1 + 16), font, body_scale, (255, 255, 255), thick)
        cv2.putText(vis, text2, (x0 + 12, y0 + tth + th1 + th2 + 28), font, body_scale, (255, 255, 255), thick)

        if target is not None:
            cx = int(target['cx'])
            cy = int(target['cy'])
            radius = int(target['radius'])
            cv2.circle(vis, (cx, cy), radius, (0, 0, 255), 2)
            cv2.drawMarker(vis, (cx, cy), (255, 0, 255), cv2.MARKER_CROSS, 18, 2)
            cv2.putText(
                vis,
                f"d={target['d']:.1f}cm ax={target['a_x']:.1f}",
                (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 255, 0),
                2,
            )
        return vis

    def loop(self):
        ok, frame = self.cam.read()
        target = None
        if not ok or frame is None:
            self.cam.reopen_if_needed()
            d, a_x = self.last_d, self.last_ax
            vis_frame = None
        else:
            target = self._detect_red_circle(frame)
            if target is None:
                d, a_x = self.last_d, self.last_ax
            else:
                d, a_x = target['d'], target['a_x']
                self.last_d, self.last_ax = d, a_x
            vis_frame = self._annotate(frame, target)

        self._force_save_debug_frame(vis_frame)
        with self.last_vis_lock:
            self.last_vis_frame = vis_frame

        msg = String()
        msg.data = f'{d:.4f},{a_x:.4f}'
        self.target_pub.publish(msg)

        self.frame_count += 1
        now = time.time()
        if now - self.last_t >= 1.0:
            self.current_fps = self.frame_count / (now - self.last_t)
            self.get_logger().info(
                f"V02: fps={self.current_fps:.1f}, res={CAMERA.get('width', 0)}x{CAMERA.get('height', 0)}, d={d:.1f}, ax={a_x:.1f}"
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
