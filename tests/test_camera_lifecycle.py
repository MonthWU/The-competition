# Purpose: verify single camera ownership, bounded buffers, fail-closed startup and resource cleanup.
import unittest
from unittest.mock import Mock, patch

import numpy as np

from car26.camera.module import CameraModule
from car26.camera.modes import capture_signature, object_capture_settings
from car26.config import load_config
from car26.telemetry import Telemetry
from test_application import ROOT


class CameraLifecycleTests(unittest.TestCase):
    # ***
    # 函数名字：test_explicit_buffers_are_verified_and_old_settings_keep_one
    # 函数作用：核对显式双缓冲读回及旧配置单缓冲，避免驱动静默忽略配置。
    # 参数意义与范围：self：当前CameraLifecycleTests实例。
    # ***
    def test_explicit_buffers_are_verified_and_old_settings_keep_one(self):
        import cv2
        config=load_config(ROOT/'config/board-object.json')
        module=CameraModule(config,Telemetry());module.role='object'
        settings=object_capture_settings(config.data['cameras']['object'],'block')
        for explicit in (True,False):
            current=dict(settings)
            if not explicit:current.pop('buffer_count')
            capture=Mock();capture.get.return_value=2
            with patch('cv2.VideoCapture',return_value=capture):
                self.assertIs(module._open_capture(current),capture)
            self.assertIn(((cv2.CAP_PROP_BUFFERSIZE,2 if explicit else 1),{}),capture.set.call_args_list)
            capture.release.assert_not_called()

    # ***
    # 函数名字：test_rejected_buffers_release_device_before_failure
    # 函数作用：验证缓冲设置失败或回读不符时释放已打开设备，不进入识别。
    # 参数意义与范围：self：当前CameraLifecycleTests实例。
    # ***
    def test_rejected_buffers_release_device_before_failure(self):
        import cv2
        config=load_config(ROOT/'config/board-object.json')
        module=CameraModule(config,Telemetry());module.role='object'
        settings=object_capture_settings(config.data['cameras']['object'],'block')
        for rejected in (True,False):
            capture=Mock();capture.get.return_value=1
            # ***
            # 函数名字：set_value
            # 函数作用：只模拟驱动对缓冲设置的拒绝，其它属性均接受。
            # 参数意义与范围：key为OpenCV属性整数；value为对应设置数值。
            # ***
            def set_value(key,value):
                return not (rejected and key==cv2.CAP_PROP_BUFFERSIZE)
            capture.set.side_effect=set_value
            with patch('cv2.VideoCapture',return_value=capture):
                with self.assertRaisesRegex(RuntimeError,'CAMERA_BUFFER_COUNT_REJECTED:object'):
                    module._open_capture(settings)
            capture.release.assert_called_once()

    # ***
    # 函数名字：test_invalid_buffer_count_and_signature_change
    # 函数作用：验证无界或非法缓冲配置被拒绝，缓冲数量改变会要求重开设备。
    # 参数意义与范围：self：当前CameraLifecycleTests实例。
    # ***
    def test_invalid_buffer_count_and_signature_change(self):
        for value in (0,5,True,2.5):
            config=load_config(ROOT/'config/board-object.json')
            config.data['cameras']['object']['buffer_count']=value
            self.assertIn('INVALID_CAMERA_BUFFER_COUNT:object',config.validate())
        config=load_config(ROOT/'config/board-object.json')
        settings=object_capture_settings(config.data['cameras']['object'],'block')
        self.assertNotEqual(capture_signature(settings),capture_signature(dict(settings,buffer_count=1)))
    # ***
    # 函数名字：test_uvc_menu_readback_and_real_mismatch
    # 函数作用：验证UVC菜单说明不触发误报且真实曝光值不符仍被拒绝。
    # 参数意义与范围：
    #   self：当前CameraLifecycleTests实例。
    # ***
    def test_uvc_menu_readback_and_real_mismatch(self):
        config = load_config(ROOT / "config/board-object.json")
        module = CameraModule(config, Telemetry())
        for exposure in (50, 51):
            readback = Mock(stdout=f"brightness: 128\ngain: 16\nauto_exposure: 1 (Manual Mode)\nexposure_time_absolute: {exposure}\n")
            with patch("car26.camera.module.subprocess.run", return_value=readback):
                if exposure == 50:
                    module._stream_controls(config.data['cameras']['object'])
                else:
                    with self.assertRaisesRegex(RuntimeError, 'CAMERA_UVC_READBACK_MISMATCH'):
                        module._stream_controls(config.data['cameras']['object'])

    def module(self):
        config = load_config(ROOT / "config/legacy_camera.json")
        return CameraModule(config, Telemetry())

    def test_failed_open_releases_device_and_reports_failure(self):
        module = self.module()
        capture = Mock()
        capture.isOpened.return_value = False
        with patch("cv2.VideoCapture", return_value=capture):
            with self.assertLogs("car26.camera", level="ERROR"):
                module._capture("qr")
        capture.release.assert_called_once()
        with self.assertRaisesRegex(RuntimeError, "CAMERA_OPEN_FAILED:qr"):
            module.result()

    def test_dimensions_mismatch_does_not_publish_or_write(self):
        module = self.module()
        capture = Mock()
        capture.read.return_value = (True, np.zeros((100, 100, 3), dtype=np.uint8))
        with patch("cv2.VideoCapture", return_value=capture), patch("cv2.VideoWriter") as writer:
            with self.assertLogs("car26.camera", level="ERROR"):
                module._capture("qr")
        self.assertIsNone(module.latest)
        writer.assert_not_called()
        capture.release.assert_called_once()
        self.assertEqual(module.error, "CAMERA_DIMENSIONS_MISMATCH:qr")

    def test_failed_join_prevents_opening_next_role(self):
        module = self.module()
        module.role = "qr"
        module.thread = Mock()
        module.thread.is_alive.return_value = True
        with patch("car26.camera.module.threading.Thread") as thread:
            with self.assertRaisesRegex(RuntimeError, "CAMERA_STOP_TIMEOUT"):
                module.switch("object")
        thread.assert_not_called()
        self.assertEqual(module.role, "qr")
