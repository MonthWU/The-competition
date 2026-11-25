# Purpose: verify all detected block classes are batched once with a distinct-class count.
import unittest
import struct
from unittest.mock import Mock

from car26.application import Application
from car26.communication.module import CommunicationModule
from car26.communication.protocol import BLOCK_CODES, blocks_frame, select_targets, target_pose_frame
from car26.config import load_config
from car26.telemetry import Telemetry
from car26.types import CameraFrame, Detection
from test_application import ROOT


class AllBlocksTests(unittest.TestCase):
    # ***
    # 函数名字：test_all_classes_and_best_duplicate_are_retained
    # 函数作用：验证不同种类无论距离均保留，同类重复时取最高置信度，丢弃无效面积或未知类别。
    # 参数意义与范围：
    #   self：当前AllBlocksTests实例。
    # ***
    def test_all_classes_and_best_duplicate_are_retained(self):
        blocks=[Detection('redblock',.9,0,0,20,20),Detection('redblock',.8,600,300,620,320),
                Detection('darkblueblock',.7,300,170,340,190)]
        invalid=[Detection('greenblock',.9,1,1,1,1),Detection('unknown',.9,0,0,40,40)]
        self.assertEqual(select_targets(blocks+invalid,(320,180)),[blocks[0],blocks[2]])
        self.assertEqual(select_targets(blocks+invalid,(0,0)),[blocks[0],blocks[2]])

    # ***
    # 函数名字：test_explicit_roi_keeps_all_inside_and_does_not_filter_marks
    # 函数作用：验证显式ROI仍生效，但区域内各类物块均保留且标靶不受物块ROI影响。
    # 参数意义与范围：
    #   self：当前AllBlocksTests实例。
    # ***
    def test_explicit_roi_keeps_all_inside_and_does_not_filter_marks(self):
        inside=[Detection('redblock',.9,100,100,120,120),Detection('greenblock',.9,200,100,220,120)]
        outside=Detection('lightblueblock',.9,500,100,520,120)
        mark=Detection('targetOne',.9,500,100,520,120)
        self.assertEqual(select_targets(inside+[outside,mark],None,(100,300,200)),inside+[mark])

    # ***
    # 函数名字：test_batch_literal_bytes_count_and_signed_centers
    # 函数作用：验证合并包的原始字节、种类数量、负偏差小端编码和空包。
    # 参数意义与范围：
    #   self：当前AllBlocksTests实例。
    # ***
    def test_batch_literal_bytes_count_and_signed_centers(self):
        wire=blocks_frame([('redblock',-320,-180),('darkblueblock',319,179)])
        self.assertEqual(wire,bytes.fromhex('5b710241c0fe4cff463f01b3005d'))
        self.assertEqual(len(wire),4+5*wire[2])
        self.assertEqual(blocks_frame([]),bytes.fromhex('5b71005d'))

    # ***
    # 函数名字：test_batch_rejects_duplicate_class_and_bad_centers
    # 函数作用：验证重复类别不能抬高num，且无效类别或超出int16范围的中心偏差不能编码。
    # 参数意义与范围：
    #   self：当前AllBlocksTests实例。
    # ***
    def test_batch_rejects_duplicate_class_and_bad_centers(self):
        with self.assertRaisesRegex(ValueError,'DUPLICATE_BLOCK_CLASS'):
            blocks_frame([('redblock',0,0),('redblock',1,1)])
        for record in [('unknown',0,0),('redblock',32768,0),('redblock',0,-32769),('redblock',True,0)]:
            with self.subTest(record=record):
                with self.assertRaises(ValueError): blocks_frame([record])
        wire=blocks_frame([('redblock',-32768,32767)])
        self.assertEqual(struct.unpack('<hh',wire[4:8]),(-32768,32767))

    # ***
    # 函数名字：test_application_batches_all_six_classes_once_and_keeps_target_pose
    # 函数作用：验证应用将六种物块合为一个包，同类重复只取最可信中心，并独立发送标靶姿态。
    # 参数意义与范围：
    #   self：当前AllBlocksTests实例。
    # ***
    def test_application_batches_all_six_classes_once_and_keeps_target_pose(self):
        config=load_config(ROOT/'config/board-object.json')
        app=Application(config)
        transport=Mock()
        transport.write.side_effect=len
        app.communication=CommunicationModule(config.data['communication'],Telemetry(),transport)
        detections=tuple(Detection(label,.9,90+i*60,30+i*30,110+i*60,50+i*30)
                         for i,label in enumerate(BLOCK_CODES))
        duplicate=Detection('redblock',.2,310,170,330,190)
        target=Detection('targetTwo',.9,230,90,270,130)
        app._object_frame(CameraFrame('object',1,0,640,360,detections+(duplicate,target),target_alpha=1.25))
        written=[call.args[0] for call in transport.write.call_args_list]
        self.assertEqual(len(written),2)
        batch=written[0]
        self.assertEqual(batch[:3],bytes.fromhex('5b7106'))
        self.assertEqual(len(batch),34)
        records=[struct.unpack_from('<Bhh',batch,3+5*i) for i in range(batch[2])]
        self.assertEqual(records,[(0x41+i,-220+60*i,-140+30*i) for i in range(6)])
        self.assertEqual(batch[-1:],b']')
        self.assertEqual(written[1],target_pose_frame(1.25,-70,-70,0))

    # ***
    # 函数名字：test_each_frame_replaces_count_and_empty_frame_is_sent
    # 函数作用：验证num只统计当前帧，空帧明确发送0且后续单类帧不沿用旧坐标。
    # 参数意义与范围：
    #   self：当前AllBlocksTests实例。
    # ***
    def test_each_frame_replaces_count_and_empty_frame_is_sent(self):
        transport=Mock()
        transport.write.side_effect=len
        comm=CommunicationModule({'write_timeout_s':1},Telemetry(),transport)
        settings={'reference_px':[0,0],'block_roi':None}
        red=Detection('redblock',.9,0,0,20,20)
        green=Detection('greenblock',.9,300,170,340,190)
        comm.send_detections((red,green),settings,640,360)
        comm.send_detections((),settings,640,360)
        comm.send_detections((green,),settings,640,360)
        written=[call.args[0] for call in transport.write.call_args_list]
        self.assertEqual([wire[2] for wire in written],[2,0,1])
        self.assertEqual(written[1],bytes.fromhex('5b71005d'))
        self.assertEqual(written[2],bytes.fromhex('5b710143000000005d'))

    # ***
    # 函数名字：test_roi_and_center_follow_current_image_size
    # 函数作用：验证物块ROI在计数前生效，偏差使用当前处理图中心而非旧参考点。
    # 参数意义与范围：
    #   self：当前AllBlocksTests实例。
    # ***
    def test_roi_and_center_follow_current_image_size(self):
        transport=Mock()
        transport.write.side_effect=len
        comm=CommunicationModule({'write_timeout_s':1},Telemetry(),transport)
        settings={'reference_px':[100,100],'block_roi':[0,300,200]}
        red=Detection('redblock',.9,90,90,110,110)
        outside=Detection('greenblock',.99,500,500,520,520)
        comm.send_detections((red,outside),settings,1280,720)
        wire=transport.write.call_args.args[0]
        self.assertEqual(wire[:4],bytes.fromhex('5b710141'))
        self.assertEqual(struct.unpack('<hh',wire[4:8]),(-540,-260))
