# Purpose: verify signed angles and bracket-framed coarse/fine pose and color payloads.
import math
import struct
import unittest
from unittest.mock import Mock
from car26.camera.focus import line_angle, alignment_target, target_geometry
from car26.communication.protocol import target_pose_frame, blocks_frame
from car26.communication.module import CommunicationModule
from car26.application import Application
from car26.config import load_config
from car26.telemetry import Telemetry
from car26.types import CameraFrame, Detection
from test_application import ROOT


class TargetPoseTests(unittest.TestCase):
    # ***
    # 函数名字：test_angle_sign_order_and_half_open_range
    # 函数作用：验证角度正负、方向无关性、竖直归一化及无效数据拒绝。
    # 参数意义与范围：
    #   self：当前TargetPoseTests实例。
    # ***
    def test_angle_sign_order_and_half_open_range(self):
        for slope in (-1,0,1):
            points=[(0,0),(10,10*slope),(20,20*slope)]
            expected=math.degrees(math.atan(slope))
            self.assertAlmostEqual(line_angle(points),expected)
            self.assertAlmostEqual(line_angle(points[::-1]),expected)
        self.assertEqual(line_angle([(0,0),(0,10),(0,20)]),-90)
        self.assertIsNone(line_angle([(0,0),(0,0),(1,1)]))
        self.assertIsNone(line_angle([(0,0),(1,1),(2,float('nan'))]))

    # ***
    # 函数名字：test_negative_tilt_does_not_pass_absolute_threshold
    # 函数作用：验证较大的负角度不会因小于正阈值而误入精定位。
    # 参数意义与范围：
    #   self：当前TargetPoseTests实例。
    # ***
    def test_negative_tilt_does_not_pass_absolute_threshold(self):
        marks=tuple(Detection(label,.9,x-25,y-25,x+25,y+25) for label,x,y in
            [('targetOne',100,180),('targetTwo',320,140),('targetThree',540,100)])
        alpha,_=target_geometry(marks)
        self.assertLess(alpha,-3)
        self.assertIsNone(alignment_target(marks,(640,360),3))

    # ***
    # 函数名字：test_pose_wire_precision_and_bounds
    # 函数作用：验证固定十字节帧、小端补码与角度单位及参数范围。
    # 参数意义与范围：
    #   self：当前TargetPoseTests实例。
    # ***
    def test_pose_wire_precision_and_bounds(self):
        frame=target_pose_frame(-12.34,-320,180,0)
        self.assertEqual(len(frame),10)
        self.assertEqual(frame[:3],b'[\x72\x00')
        self.assertEqual(struct.unpack('<hhh',frame[3:9]),(-1234,-320,180))
        self.assertEqual(frame[-1:],b']')
        self.assertEqual(struct.unpack('<h',target_pose_frame(89.999,0,0,1)[3:5])[0],8999)
        self.assertEqual(struct.unpack('<h',target_pose_frame(-90,0,0,1)[3:5])[0],-9000)
        for alpha,dx,dy,source in [(90,0,0,0),(float('nan'),0,0,0),(True,0,0,0),(0,32768,0,0),(0,0,0,2)]:
            with self.assertRaises(ValueError): target_pose_frame(alpha,dx,dy,source)

    # ***
    # 函数名字：test_coarse_pose_and_color_packets_are_independent
    # 函数作用：验证YOLO模式分别发送物块合并中心偏差包与原标靶角度偏差包。
    # 参数意义与范围：
    #   self：当前TargetPoseTests实例。
    # ***
    def test_coarse_pose_and_color_packets_are_independent(self):
        config=load_config(ROOT/'config/board-object.json')
        app=Application(config)
        transport=Mock()
        transport.write.side_effect=len
        app.communication=CommunicationModule(config.data['communication'],Telemetry(),transport)
        block=Detection('redblock',.9,300,150,340,190)
        target=Detection('targetTwo',.9,230,90,270,130)
        app._object_frame(CameraFrame('object',1,0,640,360,(block,target),target_alpha=-12.34))
        written=[call.args[0] for call in transport.write.call_args_list]
        self.assertEqual(written,[blocks_frame([('redblock',0,-10)]),target_pose_frame(-12.34,-70,-70,0)])

    # ***
    # 函数名字：test_fine_pose_requires_current_angle_and_circle
    # 函数作用：验证OpenCV报文包含当前角度且缺失任一测量时不发送旧值。
    # 参数意义与范围：
    #   self：当前TargetPoseTests实例。
    # ***
    def test_fine_pose_requires_current_angle_and_circle(self):
        app=Application(load_config(ROOT/'config/board-object.json'))
        app.communication=Mock()
        app._object_frame(CameraFrame('object',1,0,1920,1080,focus_active=True,focus_delta=(59,-138)))
        app.communication.send_alignment.assert_not_called()
        app._object_frame(CameraFrame('object',2,0,1920,1080,focus_active=True,target_alpha=1.25))
        app.communication.send_alignment.assert_not_called()
        app._object_frame(CameraFrame('object',3,0,1920,1080,focus_active=True,focus_delta=(59,-138),target_alpha=1.25))
        app.communication.send_alignment.assert_called_once_with(1.25,(59,-138),1)


if __name__=='__main__': unittest.main()
