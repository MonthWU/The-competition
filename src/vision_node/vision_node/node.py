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
from .config_from_001 import CAMERA, YOLO, CV, LASER_SWITCH, RED_CIRCLE
from .visionlib.VisionPipeline import VisionPipeline


class MJPEGHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path in ("/", "/index.html"):
            body = (
                "<html><head><title>RDK_CJLU_26_V11</title></head>"
                "<body style='margin:0;background:#111;color:#eee;font-family:Arial;'>"
                "<div style='padding:8px 12px'>RDK_CJLU_26_V11 Stream</div>"
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
            target_cfg={},
            task_cfg={},
        )

        self.loop_hz = float(CAMERA.get('loop_hz', CAMERA.get('fps', 30)))
        self.loop_hz = max(1.0, min(self.loop_hz, 120.0))
        self.debug_save_enable = bool(CAMERA.get('debug_save_enable', False))
        self.debug_save_interval_sec = float(CAMERA.get('debug_save_interval_sec', 0.5))
        self.project_root = '/root/Monthwu/RDK_vision/RDK_CJLU_26_V11'
        self.stream_host = CAMERA.get('stream_host', '192.168.1.125')

        self.target_pub = self.create_publisher(String, '/vision/target', 10)
        self.timer = self.create_timer(1.0 / self.loop_hz, self.loop)
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
        self.last_frame_shape = (cam_cfg['height'], cam_cfg['width'])
        self._last_debug_save_ts = 0.0

        self.get_logger().info(
            f'RDK_CJLU_26_V11 vision node started '
            f'(YOLO enabled={self.pipeline.yolo.enabled}, target=d,a_x, loop_hz={self.loop_hz:.1f})'
        )
        self._laser_set(True)
        self._start_http(int(CAMERA.get('web_port', 8000)))

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
        self.get_logger().info(f'Web stream started: http://{self.stream_host}:{port}/')

    def _force_save_debug_frame(self, frame):
        if frame is None or not self.debug_save_enable:
            return
        now = time.time()
        if now - self._last_debug_save_ts < self.debug_save_interval_sec:
            return
        dbg_dir = os.path.join(self.project_root, 'debug_img')
        os.makedirs(dbg_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        path = os.path.join(dbg_dir, f'{ts}.jpg')
        if cv2.imwrite(path, frame):
            self._last_debug_save_ts = now

    def _detect_red_circle_in_roi(self, frame, roi):
        if frame is None or frame.size == 0:
            return None
        h, w = frame.shape[:2]
        if roi is None:
            x1, y1, x2, y2 = 0, 0, w, h
        else:
            x1, y1, x2, y2 = roi
            x1 = max(0, min(w - 1, int(x1)))
            y1 = max(0, min(h - 1, int(y1)))
            x2 = max(x1 + 1, min(w, int(x2)))
            y2 = max(y1 + 1, min(h, int(y2)))

        roi_img = frame[y1:y2, x1:x2]
        if roi_img is None or roi_img.size == 0:
            return None

        hsv = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
        lower_red1 = (0, 90, 70)
        upper_red1 = (12, 255, 255)
        lower_red2 = (165, 90, 70)
        upper_red2 = (180, 255, 255)
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        k = int(RED_CIRCLE.get('morph_kernel', 5))
        if k % 2 == 0:
            k += 1
        k = max(3, k)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        best = None
        best_score = -1.0
        img_cx = w / 2.0
        ref_distance_cm = float(RED_CIRCLE.get('reference_distance_cm', 250.0))
        ref_diameter_px = float(RED_CIRCLE.get('reference_diameter_px', 80.0))
        real_diameter_cm = float(RED_CIRCLE.get('real_diameter_cm', 6.0))
        focal_length_mm = float(RED_CIRCLE.get('focal_length_mm', 2.8))
        pixel_size_um = float(RED_CIRCLE.get('pixel_size_um', 3.0))
        distance_model = str(RED_CIRCLE.get('distance_model', 'ellipse_minor_reference'))
        use_minor_axis_as_diameter = bool(RED_CIRCLE.get('use_minor_axis_as_diameter', True))
        min_area = float(RED_CIRCLE.get('min_area', 120))
        min_radius_px = float(RED_CIRCLE.get('min_radius_px', 6.0))
        ellipse_min_points = int(RED_CIRCLE.get('ellipse_min_points', 5))
        use_fit_ellipse = bool(RED_CIRCLE.get('use_fit_ellipse', True))

        focal_px = 0.0
        if pixel_size_um > 1e-6:
            focal_px = focal_length_mm / (pixel_size_um / 1000.0)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue

            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            if radius < min_radius_px:
                continue

            peri = cv2.arcLength(cnt, True)
            circularity = 0.0 if peri <= 1e-6 else (4.0 * 3.1415926 * area / (peri * peri))

            major = radius * 2.0
            minor = radius * 2.0
            angle = 0.0
            if use_fit_ellipse and len(cnt) >= ellipse_min_points:
                try:
                    (ecx, ecy), (ew, eh), eangle = cv2.fitEllipse(cnt)
                    cx, cy = ecx, ecy
                    major = max(float(ew), float(eh))
                    minor = min(float(ew), float(eh))
                    angle = float(eangle)
                except Exception:
                    pass

            effective_diameter_px = minor if use_minor_axis_as_diameter else major
            if effective_diameter_px <= 1e-6:
                continue

            score = area + effective_diameter_px * 6.0 + circularity * 80.0
            if score > best_score:
                best_score = score
                gx = cx + x1
                gy = cy + y1
                vision_x = float(gx - img_cx)
                if distance_model == 'pinhole_focal' and focal_px > 0.0 and real_diameter_cm > 0.0:
                    distance = (real_diameter_cm * focal_px) / effective_diameter_px
                else:
                    distance = ref_distance_cm * (ref_diameter_px / effective_diameter_px)
                best = {
                    'cx': float(gx),
                    'cy': float(gy),
                    'radius': float(radius),
                    'ellipse_major': float(major),
                    'ellipse_minor': float(minor),
                    'ellipse_angle': float(angle),
                    'effective_diameter_px': float(effective_diameter_px),
                    'distance': float(distance),
                    'vision_x': float(vision_x),
                    'roi': (x1, y1, x2, y2),
                }
        return best

    def _annotate(self, frame, target, result):
        if frame is None:
            return frame
        vis = frame.copy()
        h, w = vis.shape[:2]
        center_x = w // 2
        cv2.line(vis, (center_x, 0), (center_x, h - 1), (255, 255, 0), 1)

        cv2.putText(
            vis,
            f'FPS: {self.current_fps:.1f}',
            (12, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )

        if target is not None:
            cx = int(target['cx'])
            cy = int(target['cy'])
            if target.get('ellipse_major', 0.0) > 0 and target.get('ellipse_minor', 0.0) > 0:
                axes = (
                    max(1, int(target['ellipse_major'] / 2.0)),
                    max(1, int(target['ellipse_minor'] / 2.0)),
                )
                cv2.ellipse(vis, (cx, cy), axes, target.get('ellipse_angle', 0.0), 0, 360, (0, 0, 255), 2)
            else:
                radius = int(target['radius'])
                cv2.circle(vis, (cx, cy), radius, (0, 0, 255), 2)

            cv2.putText(
                vis,
                f'distance={target["distance"]:.1f}cm vision_x={target["vision_x"]:.1f}',
                (12, 54),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 0),
                2,
            )
            cv2.putText(
                vis,
                f'diam_px={target.get("effective_diameter_px", 0.0):.1f}',
                (12, 82),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )
        return vis

    def loop(self):
        ok, frame = self.cam.read()
        result = None
        target = None
        if not ok or frame is None:
            self.cam.reopen_if_needed()
            d, a_x = self.last_d, self.last_ax
            vis_frame = None
        else:
            self.last_frame_shape = frame.shape[:2]
            result, vis_frame = self.pipeline.process(frame)
            roi = result.get('yolo_roi') if result else None
            target = self._detect_red_circle_in_roi(frame, roi)
            if target is None:
                d, a_x = self.last_d, self.last_ax
            else:
                d, a_x = target['distance'], target['vision_x']
                self.last_d, self.last_ax = d, a_x
            vis_frame = self._annotate(vis_frame if vis_frame is not None else frame, target, result)

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
            h, w = self.last_frame_shape
            self.get_logger().info(
                f"V11: fps={self.current_fps:.1f}, res={w}x{h}, distance={d:.1f}, vision_x={a_x:.1f}, yolo={self.pipeline.yolo.enabled}"
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
