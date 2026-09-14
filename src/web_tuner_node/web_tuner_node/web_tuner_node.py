"""ROS2 web tuner and camera-preview node.

The HTTP server is intentionally an operations surface only. Camera capture and
inference stay in their dedicated real-time ROS nodes.
"""

import json
import threading
import time
from typing import Any, Dict, Iterator, List, Tuple

import cv2
import numpy as np
import rclpy
from flask import Flask, Response, jsonify, request
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from werkzeug.serving import BaseWSGIServer, make_server


PAGE_HTML = """<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Jetson Vision Tuner</title><style>
body{font-family:system-ui,sans-serif;background:#10151c;color:#e8edf2;margin:0;padding:24px;max-width:1100px;margin:auto}
h1{margin-top:0}.grid{display:grid;grid-template-columns:minmax(0,2fr) minmax(260px,1fr);gap:20px}.card{background:#1a222d;border-radius:10px;padding:16px}
img{width:100%;background:#06090d;border-radius:6px}.row{display:flex;gap:10px;align-items:center;margin:10px 0}label{min-width:120px}input{width:90px;padding:6px}button{padding:7px 12px}pre{white-space:pre-wrap;word-break:break-word;max-height:260px;overflow:auto}
</style></head><body><h1>Jetson Vision Tuner</h1><div class="grid"><section class="card"><img src="/stream.mjpg" alt="等待相机图像"></section>
<section class="card"><div id="state">正在读取状态…</div><div class="row"><label>置信度</label><input id="confidence" type="number" min="0" max="1" step="0.01"></div>
<div class="row"><label>IoU 阈值</label><input id="iou" type="number" min="0" max="1" step="0.01"></div><button onclick="save()">应用参数</button><h3>最近检测结果</h3><pre id="result">暂无</pre></section></div>
<script>
async function refresh(){const s=await fetch('/api/status').then(r=>r.json());const y=s.yolo_status||{};const o=s.opencv_status||{};const u=s.serial_status||{};document.querySelector('#state').textContent=`图像: ${s.image_available?'可用':'等待中'} | 图像 FPS: ${s.image_fps.toFixed(1)} | 推理: ${(y.last_inference_ms||0).toFixed(1)} ms | ROI: ${(o.last_processing_ms||0).toFixed(1)} ms | 丢弃: ${y.queue_drops||0} | 串口: ${u.enabled?(u.mcu_connected?'已连接':'等待连接'):'安全关闭'}`;document.querySelector('#confidence').value=s.confidence_threshold;document.querySelector('#iou').value=s.iou_threshold;document.querySelector('#result').textContent=s.last_detection||'暂无';}
async function save(){const body={confidence_threshold:+document.querySelector('#confidence').value,iou_threshold:+document.querySelector('#iou').value};const r=await fetch('/api/settings',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});if(!r.ok)alert((await r.json()).error);refresh();}
refresh();setInterval(refresh,2000);
</script></body></html>"""


PAGE_HTML = """<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Jetson Vision Debug</title><style>
:root{color-scheme:dark}*{box-sizing:border-box}body{font-family:system-ui,sans-serif;background:#111820;color:#e9eef3;margin:0;padding:12px}
.shell{max-width:1760px;margin:0 auto}.top{display:flex;align-items:baseline;justify-content:space-between;gap:16px;margin-bottom:10px}
h1{font-size:22px;line-height:1.2;margin:0}.muted{color:#8ea0b3;font-size:13px}.grid{display:grid;grid-template-columns:minmax(0,4fr) minmax(300px,1fr);gap:12px}
.panel{background:#1a242f;border:1px solid #2b3948;border-radius:8px;padding:10px}.video-panel{min-width:0}.video-viewport{position:relative;width:100%;aspect-ratio:16/9;overflow:hidden;background:#05080c;border-radius:6px}
.video{width:100%;height:100%;display:block;object-fit:contain}.center-line{position:absolute;z-index:1;pointer-events:none;background:rgba(255,56,56,.95);box-shadow:0 0 2px rgba(255,255,255,.9)}.center-line.horizontal{height:1px}.center-line.vertical{width:1px}.video-tools{position:absolute;top:8px;right:8px;display:flex;gap:6px;z-index:2}.video-tools button{background:rgba(10,17,24,.82);color:#e9eef3;border:1px solid rgba(155,173,191,.65);border-radius:5px;padding:6px 9px;cursor:pointer;backdrop-filter:blur(4px)}.video-tools button:hover,.video-tools button.active{background:#31506a}
.video-hud{position:absolute;z-index:2;left:8px;right:8px;bottom:8px;display:flex;flex-wrap:wrap;gap:6px;pointer-events:none}.hud-item{background:rgba(8,14,20,.82);border:1px solid rgba(155,173,191,.55);border-radius:5px;padding:5px 8px;line-height:1.25;backdrop-filter:blur(4px);text-shadow:0 1px 1px #000}.hud-key{color:#9badbf;margin-right:5px}.video-note{color:#8ea0b3;font-size:12px;margin:7px 2px 0}.video-viewport.native{height:calc(100vh - 112px);aspect-ratio:auto;overflow:auto}.video-viewport.native .video{width:auto;height:auto;max-width:none;max-height:none;object-fit:none}
.video-viewport:fullscreen{width:100vw;height:100vh;aspect-ratio:auto;border-radius:0;background:#000}.video-viewport:fullscreen .video{width:100%;height:100%;object-fit:contain}.video-viewport:fullscreen .video-note{display:none}
.stats{display:grid;grid-template-columns:1fr 1fr;gap:8px}.stat{background:#121a23;border:1px solid #283746;border-radius:6px;padding:10px;min-height:58px}.stat.wide{grid-column:1/-1}
.label{color:#9badbf;font-size:12px}.value{font-size:20px;line-height:1.25;margin-top:3px;word-break:break-word}.ok{color:#76d391}.warn{color:#ffcf70}.bad{color:#ff8a8a}
h2{font-size:15px;margin:14px 0 8px}.frames{margin:0;padding:0;list-style:none;max-height:280px;overflow:auto}.frames li{font-family:ui-monospace,Consolas,monospace;background:#0d141b;border:1px solid #263440;border-radius:5px;padding:7px 8px;margin-bottom:6px;word-break:break-all}
.task-state{background:#121a23;border:1px solid #283746;border-radius:6px;padding:9px 10px}.tasks{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-top:8px}.tasks button{background:#26394b;color:#e9eef3;border:1px solid #3d566d;border-radius:5px;padding:8px;cursor:pointer}.tasks button:hover{background:#31506a}
@media(max-width:1100px){.grid{grid-template-columns:1fr}.stats{grid-template-columns:repeat(3,1fr)}}
@media(max-width:680px){body{padding:8px}.stats{grid-template-columns:1fr 1fr}.top{display:block}.video-tools{top:5px;right:5px}.video-tools button{padding:5px 7px}.video-hud{left:5px;right:5px;bottom:5px}.hud-item{font-size:12px;padding:4px 6px}}
</style></head><body><main class="shell"><div class="top"><h1>Jetson Vision Debug</h1><div class="muted" id="updated">waiting</div></div>
<div class="grid"><section class="panel video-panel"><div class="video-viewport" id="video_viewport"><img class="video" id="debug_image" src="/stream.mjpg" alt="waiting for OpenCV debug image" draggable="false">
<div class="center-line horizontal" id="center_horizontal" aria-hidden="true"></div><div class="center-line vertical" id="center_vertical" aria-hidden="true"></div>
<div class="video-tools"><button type="button" id="auto_source_button" class="active" onclick="setStreamMode('auto')">自动</button><button type="button" id="live_source_button" onclick="setStreamMode('live')">现场</button><button type="button" id="plan_source_button" onclick="setStreamMode('plan')">拼图</button><button type="button" id="fit_button" class="active" onclick="setViewMode('fit')">适应</button><button type="button" id="native_button" onclick="setViewMode('native')">1:1</button><button type="button" onclick="toggleFullscreen()">全屏</button></div>
<div class="video-hud"><div class="hud-item"><span class="hud-key">任务</span><span id="hud_task">--</span></div><div class="hud-item"><span class="hud-key">场景</span><span id="hud_scene">--</span></div><div class="hud-item"><span class="hud-key">拼图</span><span id="hud_pieces">--</span></div><div class="hud-item"><span class="hud-key">图源</span><span id="hud_source">--</span></div><div class="hud-item"><span class="hud-key">显示</span><span id="hud_view">适应</span></div></div></div>
<div class="video-note">双击画面进入全屏；“1:1”按服务端预览像素显示，可拖动滚动条检查细节。</div></section>
<aside class="panel"><div class="stats">
<div class="stat"><div class="label">FPS</div><div class="value" id="fps">--</div></div>
<div class="stat wide"><div class="label">Scene</div><div class="value" id="yolo">--</div></div>
<div class="stat"><div class="label">Pieces / solve</div><div class="value" id="opencv">--</div></div>
<div class="stat"><div class="label">Resolution</div><div class="value" id="resolution">--</div></div>
<div class="stat"><div class="label">Serial Hz</div><div class="value" id="serial_hz">--</div></div>
<div class="stat"><div class="label">Serial link</div><div class="value" id="serial_link">--</div></div>
</div><h2>任务命令</h2><div class="task-state">当前任务：<span id="task_current">无任务</span></div><div class="tasks"><button onclick="sendTask(1)">基础</button><button onclick="sendTask(2)">发挥 1</button><button onclick="sendTask(3)">发挥 2</button></div><h2>Last 10 serial vision frames</h2><ul class="frames" id="serial_frames"><li>waiting</li></ul></aside></div></main>
<script>
const fmt = (v, d=1) => Number.isFinite(Number(v)) ? Number(v).toFixed(d) : '--';
let streamMode = 'auto';
function setText(id, text, cls=''){const el=document.getElementById(id);el.textContent=text;el.className='value '+cls;}
function updateCenterLines(){
  const image = document.getElementById('debug_image');
  const horizontal = document.getElementById('center_horizontal');
  const vertical = document.getElementById('center_vertical');
  let left = image.offsetLeft, top = image.offsetTop;
  let width = image.clientWidth, height = image.clientHeight;
  if(getComputedStyle(image).objectFit === 'contain' && image.naturalWidth && image.naturalHeight && width && height){
    const scale = Math.min(width / image.naturalWidth, height / image.naturalHeight);
    const renderedWidth = image.naturalWidth * scale;
    const renderedHeight = image.naturalHeight * scale;
    left += (width - renderedWidth) / 2;
    top += (height - renderedHeight) / 2;
    width = renderedWidth;
    height = renderedHeight;
  }
  horizontal.style.left = `${left}px`; horizontal.style.top = `${top + height / 2}px`; horizontal.style.width = `${width}px`;
  vertical.style.left = `${left + width / 2}px`; vertical.style.top = `${top}px`; vertical.style.height = `${height}px`;
}
function setStreamMode(mode){
  streamMode = mode;
  document.getElementById('auto_source_button').classList.toggle('active', mode === 'auto');
  document.getElementById('live_source_button').classList.toggle('active', mode === 'live');
  document.getElementById('plan_source_button').classList.toggle('active', mode === 'plan');
  document.getElementById('debug_image').src = `/stream.mjpg?source=${mode}&t=${Date.now()}`;
  requestAnimationFrame(updateCenterLines);
}
function setViewMode(mode){
  const native = mode === 'native';
  document.getElementById('video_viewport').classList.toggle('native', native);
  document.getElementById('fit_button').classList.toggle('active', !native);
  document.getElementById('native_button').classList.toggle('active', native);
  document.getElementById('hud_view').textContent = native ? '1:1' : '适应';
  requestAnimationFrame(updateCenterLines);
}
async function toggleFullscreen(){
  const viewport = document.getElementById('video_viewport');
  if(document.fullscreenElement){await document.exitFullscreen();return;}
  await viewport.requestFullscreen();
}
async function refresh(){
  const s = await fetch('/api/status', {cache:'no-store'}).then(r => r.json());
  const camera = s.camera_status || {};
  const yolo = s.yolo_status || {};
  const opencv = s.opencv_status || {};
  const serial = s.serial_status || {};
  const task = s.task_status || {};
  const fps = Number(camera.measured_fps ?? s.image_fps);
  let sceneStatus = String(camera.status || opencv.status || '');
  if(!sceneStatus){sceneStatus = task.green_a4_detected === true ? 'GREEN_A4_READY' : (task.green_a4_detected === false ? 'GREEN_A4_NOT_FOUND' : '--');}
  const pieceCount = Number(yolo.piece_count ?? opencv.piece_count ?? camera.piece_count ?? 0);
  const solveMs = Number(yolo.solve_ms ?? opencv.solve_ms ?? 0);
  const serialHz = Number(serial.vision_observed_hz ?? serial.observed_hz);
  const linkReady = serial.enabled ? Boolean(serial.mcu_connected) : false;
  const resolution = s.image_resolution || {};
  setText('fps', fmt(fps), fps >= 45 ? 'ok' : (fps > 0 ? 'warn' : 'bad'));
  const sceneOk = sceneStatus === 'SCENE_VALID' || sceneStatus === 'SCENE_VALID_LOCAL_ONLY';
  setText('yolo', sceneStatus, sceneOk ? 'ok' : 'warn');
  setText('opencv', `${pieceCount} / ${fmt(solveMs)} ms`, pieceCount > 0 ? 'ok' : 'warn');
  setText('resolution', resolution.width && resolution.height ? `${resolution.width}x${resolution.height}` : '--');
  setText('serial_hz', fmt(serialHz), serialHz > 0 ? 'ok' : 'warn');
  setText('serial_link', serial.enabled ? (linkReady ? 'connected' : 'waiting') : 'disabled', linkReady ? 'ok' : 'warn');
  const activeTask = Number(task.active_task || 0);
  const taskText = activeTask > 0 ? `${activeTask} (${task.state || 'selected'})` : '无任务';
  document.getElementById('task_current').textContent = taskText;
  document.getElementById('hud_task').textContent = taskText;
  document.getElementById('hud_scene').textContent = sceneStatus;
  document.getElementById('hud_pieces').textContent = `${pieceCount} / ${fmt(solveMs)} ms`;
  const sourceText = streamMode === 'auto' ? `自动:${s.display_source || '--'}` : streamMode;
  document.getElementById('hud_source').textContent = sourceText;
  const frames = (serial.recent_sent_vision_frames || s.serial_sent_frames || []).slice(-10);
  const list = document.getElementById('serial_frames');
  list.replaceChildren(...(frames.length ? frames : ['waiting']).map(frame => {
    const li = document.createElement('li');
    li.textContent = frame;
    return li;
  }));
  document.getElementById('updated').textContent = `updated ${new Date().toLocaleTimeString()}`;
}
async function sendTask(task){const r=await fetch('/api/task',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({task})});if(!r.ok){const e=await r.json();alert(e.error||'任务命令发送失败');return;}setTimeout(refresh,100);}
document.getElementById('debug_image').addEventListener('dblclick', toggleFullscreen);
document.getElementById('debug_image').addEventListener('load', updateCenterLines);
window.addEventListener('resize', updateCenterLines);
document.addEventListener('fullscreenchange', () => requestAnimationFrame(updateCenterLines));
if(window.ResizeObserver){const observer=new ResizeObserver(updateCenterLines);observer.observe(document.getElementById('video_viewport'));observer.observe(document.getElementById('debug_image'));}
refresh();setInterval(refresh,1000);
</script></body></html>"""


class WebTunerNode(Node):
    """Serves a LAN preview and lightweight tuning API for the vision system."""

    def __init__(self) -> None:
        super().__init__("web_tuner_node")
        self.declare_parameter("result_topic", "vision/detections")
        self.declare_parameter("display_detection_source", "vision_inference_node")
        self.declare_parameter("image_topic", "vision/image_raw")
        self.declare_parameter("plan_image_topic", "puzzle/solver_image_debug")
        self.declare_parameter("plan_image_hold_sec", 60.0)
        self.declare_parameter("camera_status_topic", "vision/camera_status")
        self.declare_parameter("yolo_status_topic", "vision/yolo_status")
        self.declare_parameter("opencv_status_topic", "vision/opencv_status")
        self.declare_parameter("serial_status_topic", "vision/serial_bridge_status")
        self.declare_parameter("task_command_topic", "puzzle/task_command")
        self.declare_parameter("task_status_topic", "puzzle/coordinator_status")
        self.declare_parameter("preview_enabled", True)
        self.declare_parameter("preview_fps", 30.0)
        self.declare_parameter("preview_width", 960)
        self.declare_parameter("confidence_threshold", 0.25)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("status_log_interval_sec", 5.0)
        self.declare_parameter("web_enabled", True)
        self.declare_parameter("web_bind_address", "0.0.0.0")
        self.declare_parameter("web_port", 5000)
        self.declare_parameter("web_debug_mode", False)

        self._result_topic = str(self.get_parameter("result_topic").value)
        self._display_detection_source = str(self.get_parameter("display_detection_source").value)
        self._image_topic = str(self.get_parameter("image_topic").value)
        self._plan_image_topic = str(self.get_parameter("plan_image_topic").value)
        self._plan_image_hold_sec = max(0.0, float(self.get_parameter("plan_image_hold_sec").value))
        self._camera_status_topic = str(self.get_parameter("camera_status_topic").value)
        self._yolo_status_topic = str(self.get_parameter("yolo_status_topic").value)
        self._opencv_status_topic = str(self.get_parameter("opencv_status_topic").value)
        self._serial_status_topic = str(self.get_parameter("serial_status_topic").value)
        self._task_command_topic = str(self.get_parameter("task_command_topic").value)
        self._task_status_topic = str(self.get_parameter("task_status_topic").value)
        self._preview_enabled = bool(self.get_parameter("preview_enabled").value)
        self._preview_fps = min(30.0, float(self.get_parameter("preview_fps").value))
        self._preview_width = int(self.get_parameter("preview_width").value)
        self._confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self._iou_threshold = float(self.get_parameter("iou_threshold").value)
        self._web_debug_mode = bool(self.get_parameter("web_debug_mode").value)
        self._state_lock = threading.Lock()
        self._image_condition = threading.Condition(self._state_lock)
        self._latest_jpeg = b""
        self._latest_plan_jpeg = b""
        self._latest_plan_time = 0.0
        self._display_source = "live"
        self._last_payload = ""
        self._camera_status: Dict[str, Any] = {}
        self._yolo_status: Dict[str, Any] = {}
        self._opencv_status: Dict[str, Any] = {}
        self._serial_status: Dict[str, Any] = {}
        self._task_status: Dict[str, Any] = {"active_task": 0, "state": "IDLE"}
        self._serial_sent_frames: List[str] = []
        self._image_resolution: Dict[str, int] = {}
        self._plan_image_resolution: Dict[str, int] = {}
        self._preview_frames = 0
        self._image_messages = 0
        self._plan_image_messages = 0
        self._image_fps = 0.0
        self._image_fps_window_count = 0
        self._image_fps_window_start = time.monotonic()
        self._detection_messages = 0
        self._last_preview_time = 0.0
        self._last_encoding_warning_time = 0.0
        self._server: BaseWSGIServer | None = None
        self._server_thread: threading.Thread | None = None

        self.add_on_set_parameters_callback(self._validate_and_apply_parameters)
        self._result_subscription = self.create_subscription(
            String, self._result_topic, self._on_detection, 10
        )
        self._image_subscription = self.create_subscription(
            Image, self._image_topic,
            lambda message: self._on_image("live", message), qos_profile_sensor_data
        )
        self._plan_image_subscription = self.create_subscription(
            Image, self._plan_image_topic,
            lambda message: self._on_image("plan", message), qos_profile_sensor_data
        )
        self._camera_status_subscription = self.create_subscription(
            String, self._camera_status_topic,
            lambda message: self._on_status("camera", message), 10
        )
        self._yolo_status_subscription = self.create_subscription(
            String, self._yolo_status_topic,
            lambda message: self._on_status("yolo", message), 10
        )
        self._opencv_status_subscription = self.create_subscription(
            String, self._opencv_status_topic,
            lambda message: self._on_status("opencv", message), 10
        )
        self._serial_status_subscription = self.create_subscription(
            String, self._serial_status_topic,
            lambda message: self._on_status("serial", message), 10
        )
        self._task_status_subscription = self.create_subscription(
            String, self._task_status_topic,
            lambda message: self._on_status("task", message), 10
        )
        self._task_command_publisher = self.create_publisher(
            String, self._task_command_topic, 10
        )
        self._timer = self.create_timer(
            max(0.5, float(self.get_parameter("status_log_interval_sec").value)),
            self._log_status,
        )
        if bool(self.get_parameter("web_enabled").value):
            self._start_web_server()

        self.get_logger().info(
            "web_tuner_node ready: "
            f"image_topic={self._image_topic} plan_image_topic={self._plan_image_topic} "
            f"result_topic={self._result_topic} "
            f"preview={self._preview_enabled}"
        )

    def _start_web_server(self) -> None:
        bind_address = str(self.get_parameter("web_bind_address").value)
        port = int(self.get_parameter("web_port").value)
        app = Flask(__name__)
        app.logger.disabled = True

        @app.get("/")
        def index() -> str:
            return PAGE_HTML

        @app.get("/api/status")
        def status() -> Response:
            return jsonify(self._current_settings())

        @app.post("/api/settings")
        def update_settings() -> Tuple[Response, int] | Response:
            payload = request.get_json(silent=True)
            if not isinstance(payload, dict):
                return jsonify(error="JSON object required"), 400
            parameters: List[Parameter] = []
            for name in ("confidence_threshold", "iou_threshold", "preview_enabled", "preview_fps"):
                if name not in payload:
                    continue
                value = payload[name]
                parameter_type = Parameter.Type.BOOL if name == "preview_enabled" else Parameter.Type.DOUBLE
                if parameter_type == Parameter.Type.DOUBLE and (not isinstance(value, (int, float)) or isinstance(value, bool)):
                    return jsonify(error=f"{name} must be numeric"), 400
                if parameter_type == Parameter.Type.BOOL and not isinstance(value, bool):
                    return jsonify(error=f"{name} must be boolean"), 400
                parameters.append(Parameter(name, parameter_type, value))
            if not parameters:
                return jsonify(error="No supported settings supplied"), 400
            results = self.set_parameters(parameters)
            failure = next((result.reason for result in results if not result.successful), None)
            if failure:
                return jsonify(error=failure), 400
            return jsonify(self._current_settings())

        @app.post("/api/task")
        def select_task() -> Tuple[Response, int] | Response:
            payload = request.get_json(silent=True)
            task_number = payload.get("task") if isinstance(payload, dict) else None
            if isinstance(task_number, bool) or not isinstance(task_number, int) or task_number not in (1, 2, 3):
                return jsonify(error="task must be integer 1, 2, or 3"), 400
            message = String()
            message.data = f"[task,{task_number}]"
            self._task_command_publisher.publish(message)
            return jsonify(accepted=True, command=message.data)

        @app.get("/stream.mjpg")
        def stream() -> Response:
            source = request.args.get("source", "auto")
            if source not in ("auto", "live", "plan"):
                source = "auto"
            return Response(
                self._mjpeg_frames(source),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )

        # BUG_POINT:WEB_BIND -- A port/address collision prevents the operations page
        # from starting. Keep the failure explicit instead of silently disabling web access.
        try:
            self._server = make_server(bind_address, port, app, threaded=True)
        except OSError as error:
            self.get_logger().error(
                f"BUG_POINT:WEB_BIND unable to listen on {bind_address}:{port}: {error}"
            )
            return
        self._server_thread = threading.Thread(
            target=self._server.serve_forever,
            name="web_tuner_http",
            daemon=True,
        )
        self._server_thread.start()
        self.get_logger().info(f"Web tuner available at http://{bind_address}:{port}/")

    def _validate_and_apply_parameters(self, parameters: List[Parameter]) -> SetParametersResult:
        updated: Dict[str, Any] = {}
        for parameter in parameters:
            if parameter.name in ("confidence_threshold", "iou_threshold"):
                if not 0.0 <= float(parameter.value) <= 1.0:
                    return SetParametersResult(successful=False, reason=f"{parameter.name} must be in [0, 1]")
                updated[parameter.name] = float(parameter.value)
            elif parameter.name == "preview_fps":
                if not 0.1 <= float(parameter.value) <= 30.0:
                    return SetParametersResult(successful=False, reason="preview_fps must be in [0.1, 30]")
                updated[parameter.name] = float(parameter.value)
            elif parameter.name == "preview_enabled":
                updated[parameter.name] = bool(parameter.value)

        with self._state_lock:
            self._confidence_threshold = updated.get("confidence_threshold", self._confidence_threshold)
            self._iou_threshold = updated.get("iou_threshold", self._iou_threshold)
            self._preview_fps = updated.get("preview_fps", self._preview_fps)
            self._preview_enabled = updated.get("preview_enabled", self._preview_enabled)
        return SetParametersResult(successful=True)

    def _on_detection(self, message: String) -> None:
        source_marker = f'"source":"{self._display_detection_source}"'
        if self._display_detection_source and source_marker not in message.data:
            return
        with self._state_lock:
            self._last_payload = message.data
            self._detection_messages += 1
        if self._web_debug_mode:
            self.get_logger().debug(f"Web preview received detection: {message.data}")

    def _on_status(self, source: str, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if not isinstance(payload, dict):
            return
        with self._state_lock:
            if source == "camera":
                self._camera_status = payload
            elif source == "yolo":
                self._yolo_status = payload
            elif source == "opencv":
                self._opencv_status = payload
            elif source == "serial":
                self._serial_status = payload
                recent_frames = payload.get("recent_sent_vision_frames")
                if isinstance(recent_frames, list):
                    self._serial_sent_frames = [
                        str(frame) for frame in recent_frames[-10:]
                    ]
                else:
                    last_frame = payload.get("last_vision_frame")
                    if isinstance(last_frame, str) and last_frame:
                        if not self._serial_sent_frames or self._serial_sent_frames[-1] != last_frame:
                            self._serial_sent_frames.append(last_frame)
                            self._serial_sent_frames = self._serial_sent_frames[-10:]
            elif source == "task":
                self._task_status = payload

    def _on_image(self, source: str, message: Image) -> None:
        now = time.monotonic()
        with self._state_lock:
            if source == "plan":
                self._plan_image_messages += 1
                self._plan_image_resolution = {
                    "width": int(message.width),
                    "height": int(message.height),
                }
            else:
                self._image_messages += 1
                self._image_resolution = {
                    "width": int(message.width),
                    "height": int(message.height),
                }
                self._image_fps_window_count += 1
                elapsed = now - self._image_fps_window_start
                if elapsed >= 1.0:
                    self._image_fps = self._image_fps_window_count / elapsed
                    self._image_fps_window_count = 0
                    self._image_fps_window_start = now
            if not self._preview_enabled:
                return
            if source != "plan" and now - self._last_preview_time < 1.0 / self._preview_fps:
                return
            if source != "plan":
                self._last_preview_time = now
            preview_width = self._preview_width

        try:
            frame = self._image_to_bgr(message)
            if frame.shape[1] > preview_width:
                scale = preview_width / frame.shape[1]
                frame = cv2.resize(frame, (preview_width, int(frame.shape[0] * scale)))
            success, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not success:
                raise ValueError("cv2.imencode returned false")
        except (ValueError, cv2.error) as error:
            # BUG_POINT:IMAGE_ENCODING -- Unsupported ROS image encodings or malformed
            # step/data sizes must not crash the web node. Debug mode identifies the source.
            if now - self._last_encoding_warning_time > 5.0:
                self._last_encoding_warning_time = now
                self.get_logger().warning(
                    f"BUG_POINT:IMAGE_ENCODING ignored {message.encoding} image: {error}"
                )
            return

        with self._image_condition:
            if source == "plan":
                self._latest_plan_jpeg = encoded.tobytes()
                self._latest_plan_time = now
            else:
                self._latest_jpeg = encoded.tobytes()
            self._preview_frames += 1
            self._image_condition.notify_all()

    @staticmethod
    def _image_to_bgr(message: Image) -> np.ndarray:
        layout = {
            "bgr8": (3, None),
            "rgb8": (3, cv2.COLOR_RGB2BGR),
            "mono8": (1, cv2.COLOR_GRAY2BGR),
            "bgra8": (4, cv2.COLOR_BGRA2BGR),
            "rgba8": (4, cv2.COLOR_RGBA2BGR),
            "yuyv": (2, cv2.COLOR_YUV2BGR_YUY2),
        }
        if message.encoding not in layout:
            raise ValueError(f"unsupported encoding '{message.encoding}'")
        channels, conversion = layout[message.encoding]
        required_step = message.width * channels
        if message.step < required_step or len(message.data) < message.step * message.height:
            raise ValueError("image data is shorter than height * step")
        rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
        frame = rows[:, :required_step].reshape(message.height, message.width, channels)
        return frame if conversion is None else cv2.cvtColor(frame, conversion)

    def _select_jpeg_locked(self, requested_source: str) -> Tuple[bytes, str]:
        if requested_source == "live":
            self._display_source = "live"
            return self._latest_jpeg, "live"
        if requested_source == "plan":
            self._display_source = "plan" if self._latest_plan_jpeg else "plan_waiting"
            return self._latest_plan_jpeg, self._display_source
        live_has_current_failure = self._live_view_has_priority_locked()
        if live_has_current_failure and self._latest_jpeg:
            self._display_source = "live_current"
            return self._latest_jpeg, "live_current"
        active_task = int(self._task_status.get("active_task", 0) or 0)
        active_generation = int(self._task_status.get("generation", 0) or 0)
        solver_task = int(self._yolo_status.get("active_task", 0) or 0)
        solver_generation = int(self._yolo_status.get("generation", 0) or 0)
        plan_is_recent = (
            self._latest_plan_jpeg and
            time.monotonic() - self._latest_plan_time <= self._plan_image_hold_sec
        )
        plan_matches_active_task = (
            active_task > 0 and
            solver_task == active_task and
            (active_generation == 0 or solver_generation == active_generation)
        )
        if self._latest_plan_jpeg and (plan_matches_active_task or plan_is_recent):
            self._display_source = "plan"
            return self._latest_plan_jpeg, "plan"
        self._display_source = "live"
        return self._latest_jpeg, "live"

    def _live_view_has_priority_locked(self) -> bool:
        camera_status = str(self._camera_status.get("status", ""))
        opencv_status = str(self._opencv_status.get("status", ""))
        task_state = str(self._task_status.get("state", ""))
        green_a4_detected = self._task_status.get("green_a4_detected")
        current_failures = {
            "GREEN_A4_NOT_FOUND",
            "CAMERA_OPEN_FAILED",
            "FRAME_READ_FAILED",
            "SCAN_CANCELLED_BY_TASK_SWITCH",
        }
        if camera_status in current_failures or opencv_status in current_failures:
            return True
        return task_state == "WAIT_GREEN_A4" and green_a4_detected is False

    def _mjpeg_frames(self, requested_source: str = "auto") -> Iterator[bytes]:
        next_emit = time.monotonic()
        while rclpy.ok():
            with self._state_lock:
                period = 1.0 / max(1.0, self._preview_fps)
            delay = next_emit - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            with self._image_condition:
                image, _ = self._select_jpeg_locked(requested_source)
            if image:
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + image + b"\r\n"
            next_emit += period
            if next_emit < time.monotonic():
                next_emit = time.monotonic()

    def _current_settings(self) -> Dict[str, Any]:
        with self._state_lock:
            selected_image, selected_source = self._select_jpeg_locked("auto")
            plan_age_sec = (
                time.monotonic() - self._latest_plan_time
                if self._latest_plan_jpeg else None
            )
            return {
                "preview_enabled": self._preview_enabled,
                "preview_fps": self._preview_fps,
                "confidence_threshold": self._confidence_threshold,
                "iou_threshold": self._iou_threshold,
                "result_topic": self._result_topic,
                "display_detection_source": self._display_detection_source,
                "image_topic": self._image_topic,
                "plan_image_topic": self._plan_image_topic,
                "display_source": selected_source,
                "image_available": bool(selected_image),
                "live_image_available": bool(self._latest_jpeg),
                "plan_image_available": bool(self._latest_plan_jpeg),
                "image_resolution": dict(self._image_resolution),
                "plan_image_resolution": dict(self._plan_image_resolution),
                "plan_image_age_sec": plan_age_sec,
                "plan_image_hold_sec": self._plan_image_hold_sec,
                "image_fps": self._image_fps,
                "image_messages": self._image_messages,
                "plan_image_messages": self._plan_image_messages,
                "preview_frames": self._preview_frames,
                "detection_messages": self._detection_messages,
                "last_detection": self._last_payload,
                "serial_sent_frames": list(self._serial_sent_frames),
                "camera_status": dict(self._camera_status),
                "yolo_status": dict(self._yolo_status),
                "opencv_status": dict(self._opencv_status),
                "serial_status": dict(self._serial_status),
                "task_status": dict(self._task_status),
                "active_task": int(self._task_status.get("active_task", 0)),
            }

    def _log_status(self) -> None:
        self.get_logger().info(f"web_tuner_node status={json.dumps(self._current_settings())}")

    def destroy_node(self) -> bool:
        if self._server is not None:
            self._server.shutdown()
        if self._server_thread is not None:
            self._server_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebTunerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
