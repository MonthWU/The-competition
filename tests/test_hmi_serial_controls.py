# Purpose: verify debug-only HTTP and common UART/web control execution with fragmented input and no hardware.
import unittest
from unittest.mock import Mock, patch
from pathlib import Path

from car26.application import Application
from car26.communication.module import CommunicationModule
from car26.communication.protocol import CommandParser, control_frame
from car26.config import load_config
from car26.hmi.module import HmiModule
from car26.camera.module import CameraModule
from car26.telemetry import Telemetry

ROOT = Path(__file__).resolve().parents[1]


class HmiSerialControlTests(unittest.TestCase):
    # ***
    # 函数名字：test_work_skips_jpeg_preview_but_debug_keeps_it
    # 函数作用：用一帧模拟扫码采集，确认work不编码网页JPEG，debug仍发布图像。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_work_skips_jpeg_preview_but_debug_keeps_it(self):
        import numpy as np
        for mode in ('work', 'debug'):
            camera = CameraModule(load_config(ROOT / 'config/board.json', mode), Telemetry())
            capture = Mock()

            # ***
            # 函数名字：read
            # 函数作用：返回单张人工图像后请求结束采集。
            # 参数意义与范围：无。
            # ***
            def read():
                camera.stop_event.set()
                return True, np.zeros((400, 640, 3), dtype=np.uint8)

            capture.read.side_effect = read
            with patch.object(camera, '_open_capture', return_value=capture), \
                    patch.object(camera, '_decode_qr', return_value=''), \
                    patch('cv2.imencode', return_value=(True, np.array([1, 2], dtype=np.uint8))) as encode:
                camera._capture('qr')
            self.assertIsNone(camera.error)
            self.assertEqual(encode.called, mode == 'debug')

    # ***
    # 函数名字：app
    # 函数作用：创建无真实设备的运行态应用，保留原有命令解析和状态机。
    # 参数意义与范围：
    #   self：当前测试实例。
    #   mode：work或debug。
    # ***
    def app(self, mode):
        app = Application(load_config(ROOT / 'config/board-object.json', mode))
        app.camera = Mock()
        app.camera.role = None
        app.camera.result.return_value = (None, ())
        app.camera.drain_mode_events.return_value = ()
        app.telemetry.update('application', state='running')
        app.telemetry.update('mission', qr_text='saved')
        return app

    # ***
    # 函数名字：run_uart
    # 函数作用：通过真实串口模块解析假传输输入，运行完整主循环后有界退出。
    # 参数意义与范围：
    #   self：当前测试实例。
    #   app：由app方法创建、尚未运行的Application。
    #   chunks：有界bytes序列，支持半帧、噪声及粘包。
    # ***
    def run_uart(self, app, chunks):
        transport = Mock()
        pending = iter(chunks)

        # ***
        # 函数名字：read
        # 函数作用：返回下一片串口输入，耗尽后停止主循环。
        # 参数意义与范围：无。
        # ***
        def read():
            try:
                return next(pending)
            except StopIteration:
                app.stop_event.set()
                return b''

        transport.read.side_effect = read
        app.communication = CommunicationModule(app.configuration.data['communication'], app.telemetry, transport)
        app.hmi = Mock()
        with patch('car26.application.signal.signal'):
            self.assertEqual(app.run(), 0)
        return transport

    # ***
    # 函数名字：test_hmi_never_binds_or_starts_threads_in_work
    # 函数作用：验证模块本身仅debug启用HTTP，配置关闭时debug也不监听。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_hmi_never_binds_or_starts_threads_in_work(self):
        for mode, enabled, expected in [('work', True, False), ('debug', False, False), ('debug', True, True)]:
            telemetry = Telemetry()
            hmi = HmiModule(dict(host='127.0.0.1', port=0, preview_fps=10,
                                max_stream_clients=1, enabled=enabled), telemetry, mode=mode)
            with patch('car26.hmi.module.ThreadingHTTPServer') as server, patch('car26.hmi.module.threading.Thread') as thread:
                hmi.start()
                self.assertEqual(server.called, expected)
                self.assertEqual(thread.called, expected)

    # ***
    # 函数名字：test_uart_controls_work_without_web_in_both_modes
    # 函数作用：验证两种模式均可接收半帧/粘包重置与任务选择，重复目标不重开设备。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_uart_controls_work_without_web_in_both_modes(self):
        for mode in ('work', 'debug'):
            app = self.app(mode)
            transport = self.run_uart(app, [b'noise[qr_re', b'set][task_qr]',
                                           b'[task_qr][task_object]'])
            self.assertIsNone(app.telemetry.snapshot()['mission']['qr_text'])
            self.assertEqual([call.args[0] for call in app.camera.switch.call_args_list], ['object', 'qr', 'object'])
            self.assertEqual(app.mission.state, 'OBJECT')
            self.assertEqual(app.telemetry.snapshot()['control']['source'], 'uart')
            self.assertEqual(app.hmi.start.called, mode == 'debug')
            transport.write.assert_not_called()

    # ***
    # 函数名字：test_web_encodes_same_frames_and_does_not_forge_rx
    # 函数作用：验证网页入队真实协议帧、主线程执行且不伪造串口历史或等待MCU回传。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_web_encodes_same_frames_and_does_not_forge_rx(self):
        app = self.app('debug')
        app.request_task_switch()
        frame, _, source = app.control_queue.queue[0]
        self.assertEqual(frame, b'[task_qr]')
        self.assertEqual(source, 'web')
        self.assertEqual(CommandParser().feed(frame), ['task_qr'])
        app.camera.switch.assert_not_called()
        app._process_controls()
        self.assertEqual(app.mission.state, 'QR')
        app.request_qr_reset()
        self.assertEqual(app.control_queue.queue[0][0], b'[qr_reset]')
        app._process_controls()
        self.assertIsNone(app.telemetry.snapshot()['mission']['qr_text'])
        self.assertEqual(app.telemetry.snapshot()['serial']['rx'], [])
        self.assertEqual(app.telemetry.snapshot()['serial']['tx'], [])

    # ***
    # 函数名字：test_uart_does_not_erase_web_completion_and_history_is_bounded
    # 函数作用：验证网页完成后紧接串口请求仍可按ID查到结果，且结果历史有界。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_uart_does_not_erase_web_completion_and_history_is_bounded(self):
        app = self.app('debug')
        result = app.request_qr_reset()
        app._process_controls()
        app._request_control('task_qr', source='uart')
        app._process_controls()
        history = app.telemetry.snapshot()['control']['results']
        self.assertEqual(next(row['status'] for row in history if row['request_id'] == result['request_id']), 'done')
        for _ in range(40):
            app._request_control('task_qr', source='uart')
            app._process_controls()
        self.assertEqual(len(app.telemetry.snapshot()['control']['results']), 32)

    # ***
    # 函数名字：test_work_rejects_web_and_unknown_commands_have_no_effect
    # 函数作用：验证work拒绝网页回调，未知命令不触发操作，非法任务阶段拒绝控制。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_work_rejects_web_and_unknown_commands_have_no_effect(self):
        app = self.app('work')
        with self.assertRaisesRegex(ValueError, 'WEB_REQUIRES_DEBUG'):
            app.request_qr_reset()
        with self.assertRaisesRegex(ValueError, 'UNKNOWN_CONTROL_COMMAND'):
            control_frame('unknown')
        self.run_uart(app, [b'[unknown]'])
        self.assertEqual(app.qr_cutoff, float('-inf'))
        self.assertEqual([call.args[0] for call in app.camera.switch.call_args_list], ['object'])
        app.mission.state = 'WAIT_START'
        with self.assertRaisesRegex(ValueError, 'TASK_NOT_READY'):
            app._request_control('qr_reset', source='uart')


if __name__ == '__main__':
    unittest.main()
