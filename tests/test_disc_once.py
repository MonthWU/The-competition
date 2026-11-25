# Purpose: verify confirmation-triggered disc search, single replies, signed encoding and phase gating.
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import Mock, patch

from car26.application import Application
from car26.camera.module import CameraModule
from car26.communication.module import CommunicationModule
from car26.communication.protocol import disc_offset_frame, state_frame, TARGET_FOCUS_REQUEST
from car26.config import load_config
from car26.telemetry import Telemetry
from car26.types import CameraFrame

ROOT = Path(__file__).resolve().parents[1]


class DiscOnceTests(unittest.TestCase):
    # ***
    # 函数名字：test_missing_disc_keeps_searching_until_phase_exit
    # 函数作用：验证连续未检出保持同一搜索请求，期间重复确认合并，退出阶段取消且不继续计算。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_missing_disc_keeps_searching_until_phase_exit(self):
        module=CameraModule(load_config(ROOT/'config/board-object.json'),Telemetry())
        module._set_recognition_mode('block')
        module.capture_profile='disc'
        with patch('car26.camera.module.time.monotonic',return_value=10):
            self.assertTrue(module.request_disc())
        request=module.disc_request
        module.disc_finder=Mock()
        module.disc_finder.find.return_value=None
        for stamp in range(11,21):
            completed,delta,result,duration=module._measure_disc(None,stamp)
            self.assertEqual(completed,0)
            self.assertIsNone(delta)
            self.assertEqual(module.disc_request,request)
            self.assertFalse(module.request_disc())
        self.assertEqual(module.disc_finder.find.call_count,10)
        module._set_recognition_mode('target')
        self.assertIsNone(module.disc_request)
        module._set_recognition_mode('block')
        self.assertEqual(module._measure_disc(None,21),(0,None,None,0.0))
        self.assertEqual(module.disc_finder.find.call_count,10)

    # ***
    # 函数名字：test_signed_nine_byte_frame_and_delimiters_inside_values
    # 函数作用：验证九字节精确格式、补码边界及载荷中的括号/逗号不会改变包长。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_signed_nine_byte_frame_and_delimiters_inside_values(self):
        self.assertEqual(disc_offset_frame(-12,8),bytes.fromhex('5b 61 2c f4 ff 2c 08 00 5d'))
        self.assertEqual(disc_offset_frame(-32768,32767),bytes.fromhex('5b 61 2c 00 80 2c ff 7f 5d'))
        self.assertEqual(disc_offset_frame(0x5d5b,0x2c),bytes.fromhex('5b 61 2c 5b 5d 2c 2c 00 5d'))
        for dx,dy in ((True,1),(1.2,0),(-32769,0),(0,32768)):
            with self.assertRaises(ValueError):
                disc_offset_frame(dx,dy)

    # ***
    # 函数名字：test_fragmented_confirmation_and_repeated_notifications
    # 函数作用：验证分片0x61确认与新增0x62进入请求独立产生动作，退出仍只记录状态。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_fragmented_confirmation_and_repeated_notifications(self):
        wire=state_frame(0x61,1)
        for split in range(1,len(wire)):
            transport=Mock()
            transport.read.side_effect=[wire[:split],wire[split:],wire+state_frame(0x61,0)+state_frame(0x62,1)]
            comm=CommunicationModule({},Telemetry(),transport)
            self.assertEqual(comm.poll(),[])
            self.assertEqual(comm.poll(),['disc_once'])
            self.assertEqual(comm.poll(),['disc_once',TARGET_FOCUS_REQUEST])
            transport.write.assert_not_called()

    # ***
    # 函数名字：test_camera_gating_and_no_unrequested_native_load
    # 函数作用：验证二维码/标靶/关闭态与禁用配置拒绝请求，未收到请求不加载原生算法。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_camera_gating_and_no_unrequested_native_load(self):
        module=CameraModule(load_config(ROOT/'config/board-object.json'),Telemetry())
        for mode in (None,'qr','target','block'):
            module._set_recognition_mode(mode)
            if mode=='block':
                with patch('car26.camera.module.time.monotonic',return_value=10):
                    self.assertTrue(module.request_disc())
                self.assertFalse(module.request_disc())
            else:
                self.assertFalse(module.request_disc())
        with patch('car26.camera.module.DiscFinder') as finder:
            self.assertEqual(module._measure_disc(None,10),(0,None,None,0.0))
            finder.assert_not_called()
            module._set_recognition_mode('target')
            self.assertIsNone(module.disc_request)
            module._set_recognition_mode('block')
            self.assertEqual(module._measure_disc(None,11),(0,None,None,0.0))
            finder.assert_not_called()
        module.configuration.data['disc_centering']['enabled']=False
        self.assertFalse(module.request_disc())

    # ***
    # 函数名字：test_application_only_accepts_confirmations_during_object_task
    # 函数作用：验证真实串口解析经应用主循环门控，二维码任务不会将确认带到后续物料任务。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_application_only_accepts_confirmations_during_object_task(self):
        for entry in ('qr','object'):
            config=load_config(ROOT/'config/board-object.json')
            config.data['mission']['entry']=entry
            config.data['hmi']['enabled']=False
            app=Application(config)
            transport=Mock()
            transport.read.return_value=state_frame(0x61,1)
            app.communication=CommunicationModule(config.data['communication'],app.telemetry,transport)
            # ***
            # 函数名字：stop
            # 函数作用：完成一轮主循环后请求退出，不实际等待。
            # 参数意义与范围：timeout：主循环计划等待的非负秒数。
            # ***
            def stop(timeout):
                app.stop_event.set()
            with patch.object(app.camera,'switch'),patch.object(app.camera,'result',return_value=(None,())), \
                 patch.object(app.camera,'request_disc',return_value=True) as request, \
                 patch.object(app.stop_event,'wait',side_effect=stop), \
                 patch('car26.application.signal.signal'):
                self.assertEqual(app.run(),0)
            self.assertEqual(request.call_count,1 if entry=='object' else 0)
            transport.write.assert_not_called()

    # ***
    # 函数名字：test_measurement_sends_once_and_missing_does_not_send_zero
    # 函数作用：验证圆盘结果只发送一次，未检出不发送零，物块YOLO结果继续发送。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_measurement_sends_once_and_missing_does_not_send_zero(self):
        app=Application(load_config(ROOT/'config/board-object.json'))
        app.communication=Mock()
        frame=CameraFrame('object',1,10,640,360,disc_request_id=1,disc_delta=(-12,8))
        app._object_frame(frame)
        app._object_frame(frame)
        app._object_frame(replace(frame,disc_request_id=2,disc_delta=None))
        app._object_frame(replace(frame,disc_request_id=3,disc_delta=(0,0)))
        self.assertEqual([c.args[0] for c in app.communication.send_disc_offset.call_args_list],[(-12,8),(0,0)])
        self.assertEqual(app.communication.send_detections.call_count,4)

    # ***
    # 函数名字：test_native_failure_does_not_create_valid_reply
    # 函数作用：验证原生检测异常不会留下可发送的旧偏差。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_native_failure_does_not_create_valid_reply(self):
        module=CameraModule(load_config(ROOT/'config/board-object.json'),Telemetry())
        module._set_recognition_mode('block')
        module.capture_profile='disc'
        with patch('car26.camera.module.time.monotonic',return_value=10):
            module.request_disc()
        module.disc_finder=Mock()
        module.disc_finder.find.side_effect=RuntimeError('DISC_NATIVE_ERROR')
        with self.assertRaisesRegex(RuntimeError,'DISC_NATIVE_ERROR'):
            module._measure_disc(None,11)
        self.assertIsNone(module.pending_disc)
