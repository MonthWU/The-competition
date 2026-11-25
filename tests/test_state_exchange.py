# Purpose: verify binary state notifications, phase ordering and the explicit one-shot disc request exception.
import unittest
from unittest.mock import Mock, patch

from car26.application import Application
from car26.camera.module import CameraModule
from car26.camera.modes import object_capture_settings
from car26.communication.module import CommunicationModule
from car26.communication.protocol import CommandParser, StateUpdate, TARGET_FOCUS_REQUEST, state_frame, target_frame, target_pose_frame
from car26.config import load_config
from car26.telemetry import Telemetry
from test_application import ROOT


class StateExchangeTests(unittest.TestCase):
    # ***
    # 函数名字：test_binary_state_bytes_and_invalid_values
    # 函数作用：验证状态包原始字节、三状态编号及严格的BOOL取值。
    # 参数意义与范围：
    #   self：当前StateExchangeTests实例。
    # ***
    def test_binary_state_bytes_and_invalid_values(self):
        for flag in (0x60,0x61,0x62):
            self.assertEqual(state_frame(flag,True),bytes((0x5b,flag,0x2c,1,0x5d)))
            self.assertEqual(state_frame(flag,0),bytes((0x5b,flag,0x2c,0,0x5d)))
        for flag,active in ((0x5f,1),(True,1),(0x61,2),(0x62,'1'),(0x61,1.0)):
            with self.assertRaises(ValueError): state_frame(flag,active)

    # ***
    # 函数名字：test_fragmented_state_and_commands_in_order
    # 函数作用：验证所有分片位置、连续状态包、噪声及既有控制命令的有序解析。
    # 参数意义与范围：
    #   self：当前StateExchangeTests实例。
    # ***
    def test_fragmented_state_and_commands_in_order(self):
        wire=b'noise'+state_frame(0x61,1)+b'[4]'+state_frame(0x62,1)+state_frame(0x61,0)+b'[align_done]'
        expected=[StateUpdate(0x61,True),'4',StateUpdate(0x62,True),StateUpdate(0x61,False),'align_done']
        for split in range(len(wire)+1):
            parser=CommandParser()
            self.assertEqual(parser.feed(wire[:split])+parser.feed(wire[split:]),expected)
        parser=CommandParser()
        self.assertEqual([item for value in wire for item in parser.feed(bytes((value,)))],expected)

    # ***
    # 函数名字：test_bad_state_cannot_become_command
    # 函数作用：验证非法BOOL、未知编号、额外载荷及字符BOOL被拒绝，随后有效帧仍能解析。
    # 参数意义与范围：
    #   self：当前StateExchangeTests实例。
    # ***
    def test_bad_state_cannot_become_command(self):
        parser=CommandParser()
        self.assertEqual(parser.feed(b'[b,1][c,0][b,\x02][c,\x01][b,\x01x][shot]'),['shot'])
        self.assertEqual(parser.dropped,5)
        self.assertEqual(parser.feed(state_frame(0x62,0)),[StateUpdate(0x62,False)])

    # ***
    # 函数名字：test_nested_peer_and_local_states_are_independent
    # 函数作用：验证嵌套状态独立，0x61和0x62进入产生请求且不直接回发。
    # 参数意义与范围：
    #   self：当前StateExchangeTests实例。
    # ***
    def test_nested_peer_and_local_states_are_independent(self):
        transport=Mock()
        transport.write.side_effect=len
        comm=CommunicationModule({'write_timeout_s':1},Telemetry(),transport)
        comm.send_state(0x61,1)
        comm.send_state(0x62,1)
        comm.send_state(0x61,0)
        self.assertEqual(comm.local_states,{0x62})
        transport.read.return_value=state_frame(0x61,1)+state_frame(0x62,1)+state_frame(0x62,1)+state_frame(0x61,0)
        self.assertEqual(comm.poll(),['disc_once',TARGET_FOCUS_REQUEST,TARGET_FOCUS_REQUEST])
        self.assertEqual(comm.peer_states,{0x62})
        transport.read.return_value=state_frame(0x62,0)+b'[skip]'
        self.assertEqual(comm.poll(),['skip'])
        self.assertEqual(comm.peer_states,set())
        self.assertEqual(comm.local_states,{0x62})
        self.assertEqual(transport.write.call_count,3)

    # ***
    # 函数名字：test_partial_state_write_and_failed_write_commit
    # 函数作用：验证状态包支持部分写入补齐，并且失败通知不会提交本机状态。
    # 参数意义与范围：
    #   self：当前StateExchangeTests实例。
    # ***
    def test_partial_state_write_and_failed_write_commit(self):
        transport=Mock()
        output=bytearray()
        # ***
        # 函数名字：write
        # 函数作用：每次最多接受两个字节，模拟UART部分写入。
        # 参数意义与范围：
        #   value：待发送的非空bytes后缀。
        # ***
        def write(value):
            output.extend(value[:2])
            return min(2,len(value))
        transport.write.side_effect=write
        comm=CommunicationModule({'write_timeout_s':1},Telemetry(),transport)
        comm.send_state(0x61,1)
        self.assertEqual(bytes(output),state_frame(0x61,1))
        transport.write.side_effect=None
        transport.write.return_value=0
        with self.assertRaises(OSError): comm.send_state(0x61,0)
        self.assertEqual(comm.local_states,{0x61})

    # ***
    # 函数名字：test_binary_payload_delimiters_are_preserved
    # 函数作用：验证坐标载荷中的方括号字节不被转义或改变固定帧长度。
    # 参数意义与范围：
    #   self：当前StateExchangeTests实例。
    # ***
    def test_binary_payload_delimiters_are_preserved(self):
        self.assertEqual(target_frame('redblock',0x5d5b,0x5b5d),bytes.fromhex('5b415b5d5d5b5d'))
        frame=target_pose_frame(0,0x5d5b,0x5b5d,1)
        self.assertEqual(frame,bytes.fromhex('5b720100005b5d5d5b5d'))

    # ***
    # 函数名字：test_atomic_mode_snapshot_and_bounded_events
    # 函数作用：验证状态事件与帧原子取出、重复模式不重复通知，满队列仍预留退出容量。
    # 参数意义与范围：
    #   self：当前StateExchangeTests实例。
    # ***
    def test_atomic_mode_snapshot_and_bounded_events(self):
        module=CameraModule(load_config(ROOT/'config/board-object.json'),Telemetry())
        module._set_recognition_mode('block')
        module._set_recognition_mode('block')
        frame,events=module.result(include_mode_events=True)
        self.assertIsNone(frame)
        self.assertEqual(events,(('block',True),))
        self.assertEqual(module.drain_mode_events(),())
        module._set_recognition_mode(None)
        module.drain_mode_events()
        for _ in range(31):
            module._set_recognition_mode('block')
            module._set_recognition_mode(None)
        module._set_recognition_mode('block')
        with self.assertRaisesRegex(RuntimeError,'CAMERA_STATE_QUEUE_FULL'):
            module._set_recognition_mode('target')
        module._set_recognition_mode(None)
        self.assertEqual(len(module.drain_mode_events()),64)

    # ***
    # 函数名字：test_failed_reopen_has_exit_without_false_target_entry
    # 函数作用：验证重开失败仅报告旧状态退出，不报告标靶已经进入。
    # 参数意义与范围：
    #   self：当前StateExchangeTests实例。
    # ***
    def test_failed_reopen_has_exit_without_false_target_entry(self):
        config=load_config(ROOT/'config/board-object.json')
        settings=config.data['cameras']['object']
        settings['recognition_modes']['target'].update(width=1920,height=1080)
        module=CameraModule(config,Telemetry())
        module._set_recognition_mode('block')
        old=Mock()
        with patch.object(module,'_open_capture',side_effect=RuntimeError('CAMERA_OPEN_FAILED')):
            with self.assertRaises(RuntimeError):
                module._switch_object_mode(old,object_capture_settings(settings,'block'),'target')
        self.assertEqual(module.drain_mode_events(),(('block',True),('block',False)))
        self.assertIsNone(module.recognition_mode)
        old.release.assert_called_once()

    # ***
    # 函数名字：test_queue_failure_releases_newly_opened_capture
    # 函数作用：验证模式事件容量耗尽时，新打开的相机也会释放，且不伪报新模式进入。
    # 参数意义与范围：
    #   self：当前StateExchangeTests实例。
    # ***
    def test_queue_failure_releases_newly_opened_capture(self):
        config=load_config(ROOT/'config/board-object.json')
        settings=config.data['cameras']['object']
        settings['recognition_modes']['target'].update(width=1920,height=1080)
        module=CameraModule(config,Telemetry())
        for _ in range(31):
            module._set_recognition_mode('block')
            module._set_recognition_mode(None)
        module._set_recognition_mode('block')
        old,new=Mock(),Mock()
        with patch.object(module,'_open_capture',return_value=new):
            with self.assertRaisesRegex(RuntimeError,'CAMERA_STATE_QUEUE_FULL'):
                module._switch_object_mode(old,object_capture_settings(settings,'block'),'target')
        old.release.assert_called_once()
        new.release.assert_called_once()
        self.assertIsNone(module.recognition_mode)
        self.assertEqual(module.drain_mode_events()[-1],('block',False))

    # ***
    # 函数名字：test_application_reports_real_mode_and_ignores_peer_as_control
    # 函数作用：验证应用发送真实状态，缺少YOLO快照时0x62请求被拒绝且不控制Mission。
    # 参数意义与范围：
    #   self：当前StateExchangeTests实例。
    # ***
    def test_application_reports_real_mode_and_ignores_peer_as_control(self):
        config=load_config(ROOT/'config/board-object.json')
        config.data['hmi']['enabled']=False
        app=Application(config)
        transport=Mock()
        transport.read.return_value=state_frame(0x61,1)+state_frame(0x62,1)
        transport.write.side_effect=len
        app.communication=CommunicationModule(config.data['communication'],app.telemetry,transport)
        app.mission.command=Mock(return_value=[])
        app.camera.role='object'
        # ***
        # 函数名字：open_mode
        # 函数作用：只记录识别状态进入，不打开真实相机。
        # 参数意义与范围：
        #   role：启动角色；此场景固定为object。
        # ***
        def open_mode(role):
            self.assertEqual(role,'object')
            app.camera._set_recognition_mode('block')
        # ***
        # 函数名字：close_mode
        # 函数作用：模拟相机成功关闭并记录退出。
        # 参数意义与范围：
        #   无。
        # ***
        def close_mode():
            app.camera._set_recognition_mode(None)
        # ***
        # 函数名字：stop
        # 函数作用：完成一轮主循环后请求退出。
        # 参数意义与范围：
        #   timeout：等待秒数；测试不实际等待。
        # ***
        def stop(timeout):
            app.stop_event.set()
        app.stop_event.wait=Mock(side_effect=stop)
        with patch.object(app.camera,'switch',side_effect=open_mode), \
             patch.object(app.camera,'close',side_effect=close_mode), \
             patch.object(app.camera,'finish_focus') as finish, patch('car26.application.signal.signal'):
            self.assertEqual(app.run(),0)
        app.mission.command.assert_not_called()
        finish.assert_not_called()
        self.assertFalse(app.focus_ending)
        self.assertEqual([call.args[0] for call in transport.write.call_args_list],
                         [state_frame(0x61,1),state_frame(0x61,0)])
        self.assertEqual(app.communication.peer_states,{0x61,0x62})
        self.assertEqual(app.communication.local_states,set())
