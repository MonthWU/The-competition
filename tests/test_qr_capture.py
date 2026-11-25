# Purpose: verify QR delivery and decoding with synthetic frames and mocked devices.
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

from car26.camera.module import CameraModule
from car26.config import load_config
from car26.telemetry import Telemetry
from test_application import ROOT


class QrCaptureTests(unittest.TestCase):
    # ***
    # 函数名字：test_terminator_is_rejected_but_open_bracket_is_payload
    # 函数作用：验证含]的二维码被跳过，包含[的下一帧仍有效，并报告二维码状态进入/退出。
    # 参数意义与范围：
    #   self：当前QrCaptureTests实例。
    # ***
    def test_terminator_is_rejected_but_open_bracket_is_payload(self):
        module=self.capture(['bad]payload','valid[payload'])
        self.assertEqual(module.result().qr_text,'valid[payload')
        self.assertEqual(module.drain_mode_events(),(('qr',True),('qr',False)))

    # ***
    # 函数名字：test_invalid_primary_and_fallback_do_not_hide_valid_qr
    # 函数作用：验证主解码和备用候选的]都被拒绝，备用解码继续选择有效内容。
    # 参数意义与范围：
    #   self：当前QrCaptureTests实例。
    # ***
    def test_invalid_primary_and_fallback_do_not_hide_valid_qr(self):
        module=self.capture(['bad]'],[SimpleNamespace(type='QRCODE',data=b'bad]'),
                                    SimpleNamespace(type='QRCODE',data=b'valid[qr')])
        self.assertEqual(module.result().qr_text,'valid[qr')

    # ***
    # 函数名字：capture
    # 函数作用：使用人工帧和模拟解码器运行有限次采集，不打开设备。
    # 参数意义与范围：
    #   self：当前QrCaptureTests实例。
    #   texts：非空列表；元素为OpenCV返回的字符串或抛出的异常。
    #   codes：pyzbar候选列表；None表示没有候选，元素包含type和bytes类型data。
    # ***
    def capture(self, texts, codes=None):
        config = load_config(ROOT / 'config/board.json')
        config.data['hmi']['enabled'] = False
        module = CameraModule(config, Telemetry())
        image = np.zeros((400, 640, 3), dtype=np.uint8)
        capture = Mock()
        count = 0

        # ***
        # 函数名字：read
        # 函数作用：返回人工图像并在最后一帧请求退出采集。
        # 参数意义与范围：无。
        # ***
        def read():
            nonlocal count
            count += 1
            if count == len(texts):
                module.stop_event.set()
            return True, image

        capture.read.side_effect = read
        detector = Mock()
        detector.detectAndDecode.side_effect = [
            value if isinstance(value, Exception) else (value, None, None) for value in texts
        ]
        fallback = Mock(return_value=codes or [])
        with patch.object(module, '_open_capture', return_value=capture), \
                patch('cv2.QRCodeDetector', return_value=detector), \
                patch.dict('sys.modules', {'pyzbar.pyzbar': SimpleNamespace(decode=fallback)}):
            module._capture('qr')
        self.assertIsNone(module.error)
        capture.release.assert_called_once()
        return module

    # ***
    # 函数名字：test_single_success_survives_empty_frames_and_is_consumed
    # 函数作用：验证成功事件不被空帧覆盖且消费后仍能读取最新视频帧。
    # 参数意义与范围：self：当前QrCaptureTests实例。
    # ***
    def test_single_success_survives_empty_frames_and_is_consumed(self):
        module = self.capture([' 456+123+564+231 ', '', ''])
        result = module.result()
        self.assertEqual(result.qr_text, ' 456+123+564+231 ')
        self.assertEqual(result.sequence, 1)
        self.assertLessEqual(result.timestamp, module.latest.timestamp)
        self.assertEqual(module.result().sequence, 3)
        self.assertEqual(module.result().qr_text, '')

    # ***
    # 函数名字：test_first_success_survives_different_later_code
    # 函数作用：验证后出现的其他二维码不能覆盖首次待消费结果。
    # 参数意义与范围：self：当前QrCaptureTests实例。
    # ***
    def test_first_success_survives_different_later_code(self):
        module = self.capture(['first', 'second'])
        self.assertEqual(module.result().qr_text, 'first')

    # ***
    # 函数名字：test_invalid_utf8_candidate_does_not_hide_valid_candidate
    # 函数作用：验证备用解码器跳过非法UTF-8后仍能选择同帧有效码。
    # 参数意义与范围：self：当前QrCaptureTests实例。
    # ***
    def test_invalid_utf8_candidate_does_not_hide_valid_candidate(self):
        module = self.capture([''], [SimpleNamespace(type='QRCODE', data=b'\xff'),
                                     SimpleNamespace(type='QRCODE', data=' 中文 '.encode())])
        self.assertEqual(module.result().qr_text, ' 中文 ')

    # ***
    # 函数名字：test_empty_and_other_barcode_candidates_are_skipped
    # 函数作用：验证空二维码及其他码制不会遮蔽后面的有效二维码。
    # 参数意义与范围：self：当前QrCaptureTests实例。
    # ***
    def test_empty_and_other_barcode_candidates_are_skipped(self):
        module = self.capture([''], [SimpleNamespace(type='CODE128', data=b'barcode'),
                                     SimpleNamespace(type='QRCODE', data=b''),
                                     SimpleNamespace(type='QRCODE', data=b'valid')])
        self.assertEqual(module.result().qr_text, 'valid')

    # ***
    # 函数名字：test_opencv_unicode_error_uses_fallback
    # 函数作用：验证OpenCV解码异常仅拒绝当前内容并继续备用解码。
    # 参数意义与范围：self：当前QrCaptureTests实例。
    # ***
    def test_opencv_unicode_error_uses_fallback(self):
        error = UnicodeDecodeError('utf-8', b'\xff', 0, 1, 'invalid start byte')
        module = self.capture([error], [SimpleNamespace(type='QRCODE', data=b'fallback')])
        self.assertEqual(module.result().qr_text, 'fallback')

    # ***
    # 函数名字：test_degenerate_opencv_geometry_continues_to_next_frame
    # 函数作用：验证OpenCV退化二维码轮廓异常不关闭采集，下一帧仍可成功。
    # 参数意义与范围：self：当前QrCaptureTests实例。
    # ***
    def test_degenerate_opencv_geometry_continues_to_next_frame(self):
        error = cv2.error('Invalid QR code source points: contourArea(src_points) is 0')
        module = self.capture([error, 'next valid code'])
        self.assertEqual(module.result().qr_text, 'next valid code')
        self.assertTrue(any(event['kind'] == 'qr_invalid_geometry'
                            for event in module.telemetry.snapshot()['events']))

    # ***
    # 函数名字：test_degenerate_opencv_geometry_uses_fallback
    # 函数作用：验证当前帧主解码器轮廓异常时备用解码器仍能返回有效结果。
    # 参数意义与范围：self：当前QrCaptureTests实例。
    # ***
    def test_degenerate_opencv_geometry_uses_fallback(self):
        error = cv2.error('Invalid QR code source points')
        module = self.capture([error], [SimpleNamespace(type='QRCODE', data=b'fallback')])
        self.assertEqual(module.result().qr_text, 'fallback')

    # ***
    # 函数名字：test_unrelated_opencv_error_is_not_hidden
    # 函数作用：验证未归类的OpenCV错误仍上报，不将资源故障伪装成无二维码。
    # 参数意义与范围：self：当前QrCaptureTests实例。
    # ***
    def test_unrelated_opencv_error_is_not_hidden(self):
        module = CameraModule(load_config(ROOT / 'config/board.json'), Telemetry())
        detector = Mock()
        detector.detectAndDecode.side_effect = cv2.error('out of memory')
        with self.assertRaises(cv2.error):
            module._decode_qr(detector, np.zeros((400, 640, 3), dtype=np.uint8), None)

    # ***
    # 函数名字：test_invalid_payload_does_not_stop_next_frame
    # 函数作用：验证非法内容后继续采集并交付下一帧有效内容。
    # 参数意义与范围：self：当前QrCaptureTests实例。
    # ***
    def test_invalid_payload_does_not_stop_next_frame(self):
        module = self.capture(['', 'next'], [SimpleNamespace(type='QRCODE', data=b'\xff')])
        self.assertEqual(module.result().qr_text, 'next')

    # ***
    # 函数名字：test_close_discards_pending_qr
    # 函数作用：验证关闭相机后不会向后续角色泄漏旧扫码事件。
    # 参数意义与范围：self：当前QrCaptureTests实例。
    # ***
    def test_close_discards_pending_qr(self):
        module = self.capture(['first', ''])
        module.close()
        self.assertIsNone(module.result())

    # ***
    # 函数名字：test_role_switch_discards_pending_qr
    # 函数作用：验证切换物块角色时清空待消费事件且不返回旧二维码帧。
    # 参数意义与范围：self：当前QrCaptureTests实例。
    # ***
    def test_role_switch_discards_pending_qr(self):
        module = self.capture(['first', ''])
        with patch('car26.camera.module.threading.Thread') as thread:
            module.switch('object')
        thread.return_value.start.assert_called_once()
        self.assertEqual(module.role, 'object')
        self.assertIsNone(module.result())

    # ***
    # 函数名字：test_optional_fallback_is_not_required
    # 函数作用：验证备用解码器不可用时正常返回OpenCV结果或继续等待。
    # 参数意义与范围：self：当前QrCaptureTests实例。
    # ***
    def test_optional_fallback_is_not_required(self):
        module = CameraModule(load_config(ROOT / 'config/board.json'), Telemetry())
        detector = Mock()
        detector.detectAndDecode.side_effect = [(' 原文 ', None, None), ('', None, None)]
        image = np.zeros((400, 640, 3), dtype=np.uint8)
        self.assertEqual(module._decode_qr(detector, image, None), ' 原文 ')
        self.assertEqual(module._decode_qr(detector, image, None), '')

    # ***
    # 函数名字：test_default_and_missing_roi_pass_entire_image_to_both_decoders
    # 函数作用：验证默认及缺省扫码ROI均把完整原图传给两种解码器，不受物块ROI影响。
    # 参数意义与范围：self：当前QrCaptureTests实例。
    # ***
    def test_default_and_missing_roi_pass_entire_image_to_both_decoders(self):
        config = load_config(ROOT / 'projects/without_radar/config/board.json')
        self.assertIsNone(config.data['cameras']['qr']['roi'])
        config.data['camera_processing']['block_roi'] = [200, 400, 300]
        image = np.zeros((400, 640, 3), dtype=np.uint8)
        module = CameraModule(config, Telemetry())
        for missing in (False, True):
            if missing:
                config.data['cameras']['qr'].pop('roi')
            detector, fallback = Mock(), Mock(return_value=[])
            detector.detectAndDecode.return_value = ('', None, None)
            module._decode_qr(detector, image, fallback)
            self.assertIs(detector.detectAndDecode.call_args.args[0], image)
            self.assertIs(fallback.call_args.args[0], image)

    # ***
    # 函数名字：test_real_qr_decode_at_four_corners_and_center
    # 函数作用：用人工二维码验证OpenCV整帧扫码覆盖四角与中心，不使用相机。
    # 参数意义与范围：self：当前QrCaptureTests实例。
    # ***
    def test_real_qr_decode_at_four_corners_and_center(self):
        config = load_config(ROOT / 'projects/without_radar/config/board.json')
        module = CameraModule(config, Telemetry())
        text = '134+213+314+321'
        code = cv2.QRCodeEncoder_create().encode(text)
        code = cv2.resize(np.pad(code, 2, constant_values=255), None, fx=5, fy=5, interpolation=cv2.INTER_NEAREST)
        size = code.shape[0]
        for x, y in ((8, 8), (640-size-8, 8), (8, 400-size-8), (640-size-8, 400-size-8),
                     ((640-size)//2, (400-size)//2)):
            with self.subTest(position=(x, y)):
                image = np.full((400, 640, 3), 255, dtype=np.uint8)
                image[y:y+size, x:x+size] = code[:, :, None]
                self.assertEqual(module._decode_qr(cv2.QRCodeDetector(), image, None), text)

    # ***
    # 函数名字：test_explicit_roi_is_the_only_way_to_crop
    # 函数作用：验证仅显式指定ROI才裁剪且两种解码器使用相同区域。
    # 参数意义与范围：self：当前QrCaptureTests实例。
    # ***
    def test_explicit_roi_is_the_only_way_to_crop(self):
        config = load_config(ROOT / 'config/board.json')
        config.data['cameras']['qr']['roi'] = [20, 30, 100, 80]
        self.assertEqual(config.validate(), [])
        image = np.arange(400*640*3, dtype=np.uint8).reshape((400, 640, 3))
        detector, fallback = Mock(), Mock(return_value=[])
        detector.detectAndDecode.return_value = ('', None, None)
        CameraModule(config, Telemetry())._decode_qr(detector, image, fallback)
        for actual in (detector.detectAndDecode.call_args.args[0], fallback.call_args.args[0]):
            np.testing.assert_array_equal(actual, image[30:110, 20:120])

    # ***
    # 函数名字：test_invalid_roi_is_rejected_without_silent_clipping
    # 函数作用：验证负数、空区域、越界或非整数ROI不能通过配置检查。
    # 参数意义与范围：self：当前QrCaptureTests实例。
    # ***
    def test_invalid_roi_is_rejected_without_silent_clipping(self):
        config = load_config(ROOT / 'config/board.json')
        for roi in ([], [0, 0, 0, 20], [-1, 0, 20, 20], [0, 0, 641, 400],
                    [0, 399, 20, 2], [True, 0, 20, 20], [0, 0, 20.5, 20]):
            with self.subTest(roi=roi):
                config.data['cameras']['qr']['roi'] = roi
                self.assertIn('INVALID_QR_ROI', config.validate())
        config.data['cameras']['qr']['roi'] = [0, 0, 640, 400]
        module = CameraModule(config, Telemetry())
        with self.assertRaisesRegex(ValueError, 'QR_ROI_OUT_OF_FRAME'):
            module._decode_qr(Mock(), np.zeros((200, 320, 3), dtype=np.uint8), None)
