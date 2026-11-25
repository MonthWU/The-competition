# Purpose: verify queued HMI controls and raw serial history without opening hardware.
import io
import json
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from car26.application import Application
from car26.communication.module import CommunicationModule
from car26.config import load_config
from car26.hmi.module import HmiModule
from car26.telemetry import Telemetry
from car26.types import CameraFrame

ROOT = Path(__file__).resolve().parents[1]


class HmiControlTests(unittest.TestCase):
    # ***
    # 函数名字：app
    # 函数作用：创建运行态应用并替换真实相机。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def app(self):
        app = Application(load_config(ROOT / 'config/board-object.json', 'debug'))
        app.camera = Mock()
        app.telemetry.update('application', state='running')
        return app

    # ***
    # 函数名字：test_switch_is_queued_and_round_trip_keeps_result
    # 函数作用：验证HTTP请求不触碰设备，主线程可往返切换且保留扫码结果。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_switch_is_queued_and_round_trip_keeps_result(self):
        app = self.app()
        app.telemetry.update('mission', qr_text='saved')
        for state, role in [('QR', 'qr'), ('OBJECT', 'object')]:
            app.camera.reset_mock()
            result = app.request_task_switch()
            self.assertEqual(result['status'], 'accepted')
            app.camera.switch.assert_not_called()
            with self.assertRaisesRegex(ValueError, 'CONTROL_BUSY'):
                app.request_task_switch()
            app._process_controls()
            app.camera.switch.assert_called_once_with(role)
            self.assertEqual(app.mission.state, state)
            self.assertEqual(app.telemetry.snapshot()['control']['status'], 'done')
            self.assertEqual(app.telemetry.snapshot()['mission']['qr_text'], 'saved')

    # ***
    # 函数名字：test_reset_clears_result_without_changing_task
    # 函数作用：验证清空结果设置帧时间门限，且不切换任务或发送报文。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_reset_clears_result_without_changing_task(self):
        app = self.app()
        app.telemetry.update('mission', qr_text='old', qr_seen_at=1)
        app.request_qr_reset()
        with patch('car26.application.time.monotonic', return_value=20):
            app._process_controls()
        self.assertEqual(app.qr_cutoff, 20)
        self.assertIsNone(app.telemetry.snapshot()['mission']['qr_text'])
        self.assertEqual(app.mission.state, 'OBJECT')
        app.camera.switch.assert_not_called()

    # ***
    # 函数名字：test_reset_rejects_old_frame_and_accepts_new_qr
    # 函数作用：验证完整主循环拒绝重置前的二维码，仍转发之后采集的新二维码。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_reset_rejects_old_frame_and_accepts_new_qr(self):
        app = self.app()
        app.mission.state = 'QR'
        app.mission.settings['object_enabled'] = False
        app.configuration.data['hmi']['enabled'] = False
        app.configuration.data['radar']['enabled'] = False
        app.communication = Mock()
        app.communication.poll.return_value = []
        app.camera.drain_mode_events.return_value = ()
        app.camera.role = 'qr'
        app.camera.result.side_effect = [
            (CameraFrame('qr', 1, 19.99, 640, 400, qr_text='old'), ()),
            (CameraFrame('qr', 2, 20.01, 640, 400, qr_text='new'), ())]
        app.request_qr_reset()
        with patch('car26.application.time.monotonic', return_value=20), patch('car26.application.signal.signal'):
            self.assertEqual(app.run(), 0)
        self.assertEqual([c.args[0] for c in app.communication.send.call_args_list], [b'[\x70new]']*4)
        self.assertEqual(app.telemetry.snapshot()['mission']['qr_text'], 'new')

    # ***
    # 函数名字：test_unavailable_and_failed_controls
    # 函数作用：验证任务未就绪时拒绝，切换失败时发布错误并释放控制锁。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_unavailable_and_failed_controls(self):
        app = self.app()
        app.mission.state = 'WAIT_START'
        with self.assertRaisesRegex(ValueError, 'TASK_NOT_READY'):
            app.request_task_switch()
        app.mission.state = 'OBJECT'
        app.request_task_switch()
        app.camera.switch.side_effect = RuntimeError('CAMERA_STOP_TIMEOUT')
        with self.assertRaisesRegex(RuntimeError, 'CAMERA_STOP_TIMEOUT'):
            app._process_controls()
        self.assertEqual(app.mission.state, 'OBJECT')
        self.assertEqual(app.telemetry.snapshot()['control']['status'], 'failed')
        self.assertTrue(app.control_lock.acquire(blocking=False))
        app.control_lock.release()

    # ***
    # 函数名字：test_raw_rx_and_partial_tx_are_retained_separately
    # 函数作用：验证未解析RX和部分写入也记录实际字节，两个方向容量独立。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_raw_rx_and_partial_tx_are_retained_separately(self):
        telemetry, transport = Telemetry(), Mock()
        transport.read.return_value = b'junk['
        transport.write.side_effect = [2, 1]
        comm = CommunicationModule({'write_timeout_s': 1}, telemetry, transport)
        comm.poll()
        comm.send(b'abc')
        serial = telemetry.snapshot()['serial']
        self.assertEqual(serial['rx'][0]['hex'], '6a 75 6e 6b 5b')
        self.assertEqual([r['hex'] for r in serial['tx']], ['61 62', '63'])
        for _ in range(110):
            telemetry.serial('tx', b'x')
        self.assertEqual(len(telemetry.snapshot()['serial']['tx']), 100)
        self.assertEqual(len(telemetry.snapshot()['serial']['rx']), 1)

    # ***
    # 函数名字：test_image_cleared_and_snapshot_is_independent
    # 函数作用：验证预览关闭清除旧图，前端可取得画面新鲜度。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_image_cleared_and_snapshot_is_independent(self):
        telemetry = Telemetry()
        telemetry.image('qr', b'jpeg')
        self.assertIn('qr', telemetry.snapshot()['previews'])
        telemetry.clear_image('qr')
        self.assertNotIn('qr', telemetry.snapshot()['previews'])
        self.assertIsNone(telemetry.image('qr'))

    # ***
    # 函数名字：test_switch_http_and_conflict
    # 函数作用：验证POST任务端点返回排队确认，重复点击返回409。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_switch_http_and_conflict(self):
        app = self.app()
        hmi = HmiModule({'host': '127.0.0.1', 'port': 0, 'preview_fps': 10,
                         'max_stream_clients': 4}, app.telemetry, task_switch=app.request_task_switch, mode='debug')
        with patch('car26.hmi.module.ThreadingHTTPServer') as server, patch('car26.hmi.module.threading.Thread'):
            hmi.start()
        handler_type = server.call_args.args[1]
        handler = handler_type.__new__(handler_type)
        handler.path = '/api/task/switch'
        handler.headers = {'Host': 'localhost', 'Origin': 'http://localhost', 'Content-Length': '0'}
        handler.wfile = io.BytesIO()
        handler.send_response, handler.send_header = Mock(), Mock()
        handler.end_headers, handler.send_error = Mock(), Mock()
        handler.do_POST()
        self.assertEqual(json.loads(handler.wfile.getvalue())['status'], 'accepted')
        handler.do_POST()
        handler.send_error.assert_called_with(409, 'CONTROL_BUSY')
