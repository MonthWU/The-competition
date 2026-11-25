# Purpose: verify debug HMI stream cleanup without opening a real HTTP listener.
import unittest
from unittest.mock import Mock, patch

from car26.hmi.module import HmiModule
from car26.telemetry import Telemetry


class HmiTests(unittest.TestCase):
    # ***
    # 函数名字：test_closed_inactive_camera_stream_frees_client_slot
    # 函数作用：检查debug网页预览连接断开时释放并发名额。
    # 参数意义与范围：self：当前HmiTests实例。
    # ***
    def test_closed_inactive_camera_stream_frees_client_slot(self):
        hmi = HmiModule({"host": "127.0.0.1", "port": 8080,
                         "preview_fps": 10, "max_stream_clients": 1}, Telemetry(), mode='debug')
        with patch("car26.hmi.module.ThreadingHTTPServer") as server, patch("car26.hmi.module.threading.Thread"):
            hmi.start()
        handler_type = server.call_args.args[1]
        handler = handler_type.__new__(handler_type)
        handler.path = "/camera/qr"
        handler.connection = Mock()
        handler.connection.recv.return_value = b""
        handler.send_response = Mock()
        handler.send_header = Mock()
        handler.end_headers = Mock()
        with patch("car26.hmi.module.select.select", return_value=([handler.connection], [], [])):
            handler.do_GET()
        self.assertTrue(hmi.stream_slots.acquire(blocking=False))
        hmi.stream_slots.release()
