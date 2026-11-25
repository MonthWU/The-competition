# Purpose: verify QR-preview reset state, in-flight frame rejection and the optional HTTP reset endpoint.
import io
import json
import unittest
from unittest.mock import Mock, patch

from car26.camera.preview import QrPreviewState
from car26.hmi.module import HmiModule
from car26.telemetry import Telemetry
from car26.types import CameraFrame


class QrResetTests(unittest.TestCase):
    # ***
    # 函数名字：state
    # 函数作用：创建具有确定初始帧时间门限的独立扫码预览状态。
    # 参数意义与范围：self：当前QrResetTests实例。
    # ***
    def state(self):
        telemetry = Telemetry()
        with patch('car26.camera.preview.time.monotonic', return_value=10):
            state = QrPreviewState(telemetry)
        return state, telemetry

    # ***
    # 函数名字：handler
    # 函数作用：创建无网络连接的HTTP处理器测试实例。
    # 参数意义与范围：
    #   self：当前QrResetTests实例。
    #   callback：重置回调或None，None代表正式应用不提供重置。
    #   path：请求路径，默认/api/qr/reset。
    # ***
    def handler(self, callback, path='/api/qr/reset'):
        hmi = HmiModule({'host': '127.0.0.1', 'port': 8080, 'preview_fps': 10,
                         'max_stream_clients': 1}, Telemetry(), qr_reset=callback, mode='debug')
        with patch('car26.hmi.module.ThreadingHTTPServer') as server, \
                patch('car26.hmi.module.threading.Thread'):
            hmi.start()
        handler = server.call_args.args[1].__new__(server.call_args.args[1])
        handler.path = path
        handler.headers = {'Host': '127.0.0.1:8080', 'Content-Length': '0'}
        handler.connection = Mock()
        handler.wfile = io.BytesIO()
        handler.send_response, handler.send_header, handler.end_headers, handler.send_error = Mock(), Mock(), Mock(), Mock()
        return handler

    # ***
    # 函数名字：test_reset_clears_text_count_and_time
    # 函数作用：验证重置清空后台结果、次数和时间并递增会话代次。
    # 参数意义与范围：self：当前QrResetTests实例。
    # ***
    def test_reset_clears_text_count_and_time(self):
        state, telemetry = self.state()
        self.assertTrue(state.observe(CameraFrame('qr', 1, 11, 640, 400, qr_text='first')))
        self.assertFalse(state.observe(CameraFrame('qr', 2, 12, 640, 400, qr_text='first')))
        self.assertEqual(telemetry.snapshot()['mission']['recognized_frames'], 2)
        with patch('car26.camera.preview.time.monotonic', return_value=20):
            self.assertEqual(state.reset(), {'status': 'reset', 'generation': 1})
        result = telemetry.snapshot()['mission']
        self.assertIsNone(result['qr_text'])
        self.assertIsNone(result['qr_seen_at'])
        self.assertEqual(result['recognized_frames'], 0)
        self.assertTrue(result['qr_reset_enabled'])
        self.assertFalse(result['uart_enabled'])

    # ***
    # 函数名字：test_in_flight_old_frame_cannot_restore_result
    # 函数作用：验证重置前开始采集的帧及边界帧不会恢复旧识别值，新帧可正常识别。
    # 参数意义与范围：self：当前QrResetTests实例。
    # ***
    def test_in_flight_old_frame_cannot_restore_result(self):
        state, telemetry = self.state()
        with patch('car26.camera.preview.time.monotonic', return_value=20):
            state.reset()
        for timestamp in (19, 20):
            self.assertFalse(state.observe(CameraFrame('qr', 1, timestamp, 640, 400, qr_text='old')))
        self.assertEqual(telemetry.snapshot()['mission']['recognized_frames'], 0)
        self.assertTrue(state.observe(CameraFrame('qr', 2, 21, 640, 400, qr_text='new')))
        self.assertEqual(telemetry.snapshot()['mission']['qr_text'], 'new')

    # ***
    # 函数名字：test_empty_and_other_role_do_not_create_success
    # 函数作用：验证空结果及物块角色不改变扫码预览结果。
    # 参数意义与范围：self：当前QrResetTests实例。
    # ***
    def test_empty_and_other_role_do_not_create_success(self):
        state, telemetry = self.state()
        for frame in (CameraFrame('qr', 1, 11, 640, 400),
                      CameraFrame('object', 2, 12, 640, 360, qr_text='wrong role')):
            self.assertFalse(state.observe(frame))
        self.assertIsNone(telemetry.snapshot()['mission']['qr_text'])

    # ***
    # 函数名字：test_post_reset_returns_confirmation
    # 函数作用：验证同源空正文POST调用重置并返回后台代次确认。
    # 参数意义与范围：self：当前QrResetTests实例。
    # ***
    def test_post_reset_returns_confirmation(self):
        state, telemetry = self.state()
        state.observe(CameraFrame('qr', 1, 11, 640, 400, qr_text='before'))
        handler = self.handler(state.reset)
        handler.headers['Origin'] = 'http://127.0.0.1:8080'
        handler.do_POST()
        handler.send_response.assert_called_once_with(200)
        self.assertEqual(json.loads(handler.wfile.getvalue()), {'status': 'reset', 'generation': 1})
        self.assertIsNone(telemetry.snapshot()['mission']['qr_text'])

    # ***
    # 函数名字：test_formal_application_and_wrong_path_have_no_reset
    # 函数作用：验证未注册回调的正式应用及其它路径没有重置端点。
    # 参数意义与范围：self：当前QrResetTests实例。
    # ***
    def test_formal_application_and_wrong_path_have_no_reset(self):
        callback = Mock()
        for handler in (self.handler(None), self.handler(callback, '/api/mission/reset')):
            handler.do_POST()
            handler.send_error.assert_called_once_with(404)
        callback.assert_not_called()

    # ***
    # 函数名字：test_get_does_not_reset
    # 函数作用：验证读取URL不会触发状态修改。
    # 参数意义与范围：self：当前QrResetTests实例。
    # ***
    def test_get_does_not_reset(self):
        callback = Mock()
        handler = self.handler(callback)
        handler.do_GET()
        handler.send_error.assert_called_once_with(404)
        callback.assert_not_called()

    # ***
    # 函数名字：test_foreign_origin_and_nonempty_body_are_rejected
    # 函数作用：验证跨站或非空正文请求不会触发重置。
    # 参数意义与范围：self：当前QrResetTests实例。
    # ***
    def test_foreign_origin_and_nonempty_body_are_rejected(self):
        for headers, code in (({'Origin': 'http://another-host'}, 403),
                              ({'Content-Length': '3'}, 400),
                              ({'Transfer-Encoding': 'chunked'}, 400)):
            callback = Mock()
            handler = self.handler(callback)
            handler.headers.update(headers)
            handler.do_POST()
            handler.send_error.assert_called_once_with(code)
            callback.assert_not_called()
