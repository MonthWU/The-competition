# Purpose: verify that the interactive QR web page stays available when its camera reports an error.
import importlib.util
import time
import unittest
from unittest.mock import Mock, patch

from car26.config import load_config
from car26.telemetry import Telemetry
from car26.types import CameraFrame
from test_application import ROOT


class QrPreviewRuntimeTests(unittest.TestCase):
    # ***
    # 函数名字：scenario
    # 函数作用：用模拟相机运行两轮预览，验证相机错误不提前关闭网页。
    # 参数意义与范围：
    #   self：当前QrPreviewRuntimeTests实例。
    #   recovered：布尔值；True时第二轮提供有效帧，否则保持没有新帧。
    # ***
    def scenario(self, recovered):
        spec = importlib.util.spec_from_file_location('qr_preview_test', ROOT/'artifacts/qr_preview_20261118.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        telemetry = Telemetry()
        camera, hmi, stopped = Mock(), Mock(), Mock()
        stopped.is_set.side_effect = [False, False, True]
        frame = CameraFrame('qr', 1, time.monotonic(), 640, 400) if recovered else None
        camera.result.side_effect = [RuntimeError('simulated camera failure'), frame]

        # ***
        # 函数名字：during_wait
        # 函数作用：核对错误后的等待发生时网页仍未关闭。
        # 参数意义与范围：seconds：非负等待秒数，本测试忽略实际等待。
        # ***
        def during_wait(seconds):
            hmi.close.assert_not_called()

        stopped.wait.side_effect = during_wait
        with patch.object(module, 'load_config', return_value=load_config(ROOT/'config/board.json')), \
                patch.object(module, 'Telemetry', return_value=telemetry), \
                patch.object(module, 'CameraModule', return_value=camera), \
                patch.object(module, 'HmiModule', return_value=hmi), \
                patch.object(module.threading, 'Event', return_value=stopped), \
                patch.object(module.signal, 'signal'), patch('builtins.print'):
            module.main()
        self.assertEqual(camera.result.call_count, 2)
        camera.close.assert_called_once()
        hmi.close.assert_called_once()
        return telemetry.snapshot()['application']

    # ***
    # 函数名字：test_camera_failure_keeps_web_running_until_requested_stop
    # 函数作用：验证相机异常仅写入诊断状态，预览循环仍继续。
    # 参数意义与范围：self：当前QrPreviewRuntimeTests实例。
    # ***
    def test_camera_failure_keeps_web_running_until_requested_stop(self):
        state = self.scenario(False)
        self.assertEqual(state['state'], 'qr_preview_camera_error')
        self.assertIn('simulated camera failure', state['error'])

    # ***
    # 函数名字：test_new_frame_clears_temporary_error_state
    # 函数作用：验证后续有效帧到达后恢复预览状态并清除旧错误。
    # 参数意义与范围：self：当前QrPreviewRuntimeTests实例。
    # ***
    def test_new_frame_clears_temporary_error_state(self):
        state = self.scenario(True)
        self.assertEqual(state['state'], 'qr_preview')
        self.assertIsNone(state['error'])
