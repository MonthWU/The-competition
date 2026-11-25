# Purpose: verify the authoritative model order, complete three-state mapping and QR framing rule.
import unittest
from unittest.mock import Mock, patch
import numpy as np

from car26.application import Application
from car26.camera.detector import decode_heads
from car26.camera.classmap import load_model_labels
from car26.communication.module import CommunicationModule
from car26.communication.protocol import BLOCK_CODES, MARK_CODES, STATE_FLAGS, qr_frame, state_frame, target_pose_frame
from car26.config import load_config
from car26.mission import Mission
from car26.types import CameraFrame
from test_application import ROOT


class CurrentProtocolContractTests(unittest.TestCase):
    # ***
    # 函数名字：test_model_order_and_every_output_class
    # 函数作用：验证业务类别目录与真实模型输出顺序分离，按绑定表解码九个索引并维持串口语义。
    # 参数意义与范围：
    #   self：当前CurrentProtocolContractTests实例。
    # ***
    def test_model_order_and_every_output_class(self):
        catalog=['yellowblock','redblock','lightblueblock','greenblock','targetTwo',
                 'blackblock','darkblueblock','targetThree','targetOne']
        expected=['redblock','yellowblock','greenblock','blackblock','lightblueblock',
                  'darkblueblock','targetOne','targetTwo','targetThree']
        for name in ('config/board-object.json','projects/without_radar/config/board.json',
                     'projects/with_radar/config/board.json'):
            config=load_config(ROOT/name)
            path=config.path(config.data['models']['object']['classes'])
            self.assertEqual(path,ROOT/'assets/models/classes.name')
            self.assertEqual(path.read_text(encoding='utf-8').splitlines(),catalog)
            self.assertEqual(config.path(config.data['models']['object']['class_map']),ROOT/'assets/models/yolo11_x5.classes.json')
            self.assertEqual(load_model_labels(config,config.data['models']['object']),tuple(expected))
        self.assertEqual(set(expected),set(BLOCK_CODES)|set(MARK_CODES))
        self.assertEqual(BLOCK_CODES['lightblueblock'],0x45)
        self.assertEqual(BLOCK_CODES['darkblueblock'],0x46)
        for index,label in enumerate(expected):
            outputs=[]
            for side in (4,2,1):
                classes=np.full((1,side,side,9),-30,dtype=np.float32)
                boxes=np.full((1,side,side,64),-30,dtype=np.float32)
                boxes.reshape(1,side,side,4,16)[...,1]=30
                outputs.extend((classes,boxes))
            outputs[0][0,1,2,index]=30
            found=decode_heads(outputs,expected,64,32,.5,.5,input_size=32)
            self.assertEqual(len(found),1)
            self.assertEqual(found[0].label,label)

    # ***
    # 函数名字：test_three_states_match_payload_types
    # 函数作用：验证二维码、物块、标靶的三个状态码和三个业务类型没有错位。
    # 参数意义与范围：
    #   self：当前CurrentProtocolContractTests实例。
    # ***
    def test_three_states_match_payload_types(self):
        self.assertEqual(STATE_FLAGS,{'qr':0x60,'block':0x61,'target':0x62})
        self.assertEqual(qr_frame('abc'),bytes.fromhex('5b706162635d'))
        self.assertEqual(target_pose_frame(0,0,0,0),bytes.fromhex('5b72000000000000005d'))
        for flag in (0x60,0x61,0x62):
            self.assertEqual(state_frame(flag,1),bytes((0x5b,flag,0x2c,1,0x5d)))
        with self.assertRaises(ValueError): state_frame(0x63,1)

    # ***
    # 函数名字：test_qr_guard_does_not_advance_mission
    # 函数作用：验证含]内容不会进入发送状态，合法[内容仍可按原文转发四次。
    # 参数意义与范围：
    #   self：当前CurrentProtocolContractTests实例。
    # ***
    def test_qr_guard_does_not_advance_mission(self):
        config=load_config(ROOT/'config/board.json')
        mission=Mission(config.data['mission'],None)
        with self.assertRaisesRegex(ValueError,'QR_PAYLOAD_CONTAINS_TERMINATOR'):
            qr_frame('bad]')
        self.assertEqual(mission.qr('bad]'),[])
        self.assertEqual(mission.state,'QR')
        actions=mission.qr('valid[qr')
        self.assertEqual([wire for action,wire in actions if action=='send'],[b'[\x70valid[qr]']*4)

    # ***
    # 函数名字：test_qr_state_and_transfer_before_block_state
    # 函数作用：验证主线程先通知二维码进入，再发四包原文，交接时二维码退出、物块进入，关闭时物块退出。
    # 参数意义与范围：
    #   self：当前CurrentProtocolContractTests实例。
    # ***
    def test_qr_state_and_transfer_before_block_state(self):
        config=load_config(ROOT/'config/board.json')
        config.data['hmi']['enabled']=False
        app=Application(config)
        transport=Mock()
        transport.read.return_value=b''
        transport.write.side_effect=len
        app.communication=CommunicationModule(config.data['communication'],app.telemetry,transport)
        # ***
        # 函数名字：switch
        # 函数作用：模拟相机真实功能交接并只在二维码阶段提供一帧有效结果。
        # 参数意义与范围：
        #   role：相机角色qr或object，不访问真实设备。
        # ***
        def switch(role):
            app.camera.role=role
            app.camera._set_recognition_mode('qr' if role=='qr' else 'block')
            app.camera.latest=CameraFrame('qr',1,0,640,400,qr_text='valid[qr') if role=='qr' else None
        # ***
        # 函数名字：close
        # 函数作用：模拟成功关闭相机并产生真实退出事件。
        # 参数意义与范围：
        #   无。
        # ***
        def close():
            app.camera._set_recognition_mode(None)
        # ***
        # 函数名字：stop
        # 函数作用：主循环完成一轮交接后停止。
        # 参数意义与范围：
        #   timeout：等待时间秒数；测试忽略实际等待。
        # ***
        def stop(timeout):
            app.stop_event.set()
        app.stop_event.wait=Mock(side_effect=stop)
        with patch.object(app.camera,'switch',side_effect=switch),patch.object(app.camera,'close',side_effect=close), \
             patch('car26.application.signal.signal'),patch('car26.application.time.monotonic',return_value=0):
            self.assertEqual(app.run(),0)
        packets=[call.args[0] for call in transport.write.call_args_list]
        self.assertEqual(packets,[state_frame(0x60,1)]+[qr_frame('valid[qr')]*4+
                         [state_frame(0x60,0),state_frame(0x61,1),state_frame(0x61,0)])
