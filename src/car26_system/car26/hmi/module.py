# Purpose: serve telemetry, camera previews and explicit queued application controls.
import json
import select
import socket
import threading
from urllib.parse import urlsplit
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


class HmiModule:
    # ***
    # 函数名字：__init__
    # 函数作用：初始化当前类的依赖、配置和内部状态。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   settings：当前模块配置字典；键、类型及数值范围由Configuration.validate约束。
    #   telemetry：共享Telemetry对象；范围为当前应用初始化的线程安全状态容器。
    #   qr_reset：可选无参数回调，返回JSON兼容重置结果；None不提供重置端点。
    #   task_switch：可选无参数回调，将任务切换提交给应用主线程；None不提供该端点。
    #   mode：work或debug；只有debug可创建HTTP服务与线程，默认work。
    # ***
    def __init__(self, settings, telemetry, qr_reset=None, task_switch=None, mode='work'):
        self.settings, self.telemetry = settings, telemetry
        self.server = self.thread = None
        self.stop_event = threading.Event()
        self.qr_reset = qr_reset
        self.task_switch = task_switch
        self.mode = mode
        self.stream_slots = threading.BoundedSemaphore(settings["max_stream_clients"])

    # ***
    # 函数名字：start
    # 函数作用：仅在debug且HMI启用时创建HTTP服务和线程，其余模式保持关闭。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    # ***
    def start(self):
        if self.mode != 'debug' or not self.settings.get('enabled', True):
            self.telemetry.update('hmi', state='disabled')
            return
        owner = self

        class Handler(BaseHTTPRequestHandler):
            # ***
            # 函数名字：log_message
            # 函数作用：抑制HTTP服务默认的逐请求日志。
            # 参数意义与范围：
            #   self：当前所属类实例；范围为该类已初始化的对象。
            #   _：HTTP框架附带的日志参数；任意位置参数，本函数只忽略其内容。
            # ***
            def log_message(self, *_):
                pass

            # ***
            # 函数名字：respond
            # 函数作用：向HTTP客户端发送带长度及类型的完整响应。
            # 参数意义与范围：
            #   self：当前所属类实例；范围为该类已初始化的对象。
            #   data：HTTP响应正文；范围为bytes，长度须可由Content-Length表达。
            #   kind：事件类型或HTTP内容类型；范围为调用方明确给定的非空字符串。
            # ***
            def respond(self, data, kind):
                self.send_response(200)
                self.send_header("Content-Type", kind)
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(data)

            # ***
            # 函数名字：do_GET
            # 函数作用：处理状态、页面及相机预览的HTTP读取请求。
            # 参数意义与范围：
            #   self：当前所属类实例；范围为该类已初始化的对象。
            # ***
            def do_GET(self):
                try:
                    self.connection.settimeout(2)
                    if self.path == "/":
                        self.respond(Path(__file__).with_name("index.html").read_bytes(), "text/html; charset=utf-8")
                    elif self.path == "/api/status":
                        self.respond(json.dumps(owner.telemetry.snapshot(), ensure_ascii=False).encode(), "application/json; charset=utf-8")
                    elif self.path.startswith("/camera/") and self.path[8:] in ("qr", "object", "prescan"):
                        if not owner.stream_slots.acquire(blocking=False):
                            self.send_error(503, "preview client limit")
                            return
                        try:
                            self.send_response(200)
                            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                            self.end_headers()
                            role = self.path[8:]
                            previous = None
                            while not owner.stop_event.is_set():
                                ready, _, _ = select.select([self.connection], [], [], 0)
                                if ready and not self.connection.recv(1, socket.MSG_PEEK):
                                    break
                                frame = owner.telemetry.image(role)
                                if frame and frame[0] != previous:
                                    stamp, jpeg = frame
                                    previous = stamp
                                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                                     + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
                                owner.stop_event.wait(1 / owner.settings["preview_fps"])
                        finally:
                            owner.stream_slots.release()
                    else:
                        self.send_error(404)
                except (BrokenPipeError, ConnectionError, TimeoutError):
                    pass

            # ***
            # 函数名字：do_POST
            # 函数作用：校验同源空正文请求并提交已启用的重置或任务切换动作。
            # 参数意义与范围：self：当前HTTP请求Handler实例。
            # ***
            def do_POST(self):
                callback = {'/api/qr/reset': owner.qr_reset,
                            '/api/task/switch': owner.task_switch}.get(self.path)
                if callback is None:
                    self.send_error(404)
                    return
                origin = self.headers.get('Origin')
                if origin and urlsplit(origin).netloc != self.headers.get('Host'):
                    self.send_error(403)
                    return
                if self.headers.get('Content-Length', '0') != '0' or self.headers.get('Transfer-Encoding'):
                    self.close_connection = True
                    self.send_error(400)
                    return
                try:
                    result = callback()
                    self.respond(json.dumps(result, ensure_ascii=False).encode('utf-8'),
                                 'application/json; charset=utf-8')
                except (BrokenPipeError, ConnectionError, TimeoutError):
                    pass
                except ValueError as error:
                    self.send_error(409, str(error))
                except Exception:
                    self.send_error(500, 'HMI_CONTROL_FAILED')

        self.server = ThreadingHTTPServer((self.settings["host"], self.settings["port"]), Handler)
        self.server.daemon_threads = True
        self.thread = threading.Thread(target=lambda: self.server.serve_forever(poll_interval=.1), name="hmi")
        self.thread.start()
        self.telemetry.update("hmi", state="ready", port=self.settings["port"])

    # ***
    # 函数名字：close
    # 函数作用：关闭当前模块持有的线程、连接或设备资源。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    # ***
    def close(self):
        self.stop_event.set()
        if self.server:
            self.server.shutdown()
            self.server.server_close()
        if self.thread:
            self.thread.join(timeout=3)
            if self.thread.is_alive():
                raise RuntimeError("HMI_STOP_TIMEOUT")
        self.telemetry.update("hmi", state="stopped")
