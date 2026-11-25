# Purpose: verify legacy alignment utility and confirmation-triggered search until found with continuous YOLO.
import unittest
from unittest.mock import Mock, patch
from pathlib import Path

import numpy as np

from car26.camera.disc import DiscAlignment
from car26.camera.module import CameraModule
from car26.config import load_config
from car26.telemetry import Telemetry
from car26.types import Detection

ROOT = Path(__file__).resolve().parents[1]


class DiscCenteringTests(unittest.TestCase):
    # ***
    # 函数名字：test_tolerance_boundaries_and_latch
    # 函数作用：验证两轴同时满足含边界5像素才锁定，不提前四舍五入，锁定后不再执行检测。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_tolerance_boundaries_and_latch(self):
        finder = Mock()
        finder.find.side_effect = [None,{'offset':[5.001,0]}, {'offset':[0,-5.001]},
                                   {'offset':[-5,5]}]
        alignment = DiscAlignment(finder,(5,5))
        for _ in range(3):
            alignment.process(None)
            self.assertFalse(alignment.aligned)
        self.assertEqual(alignment.process(None)['offset'],[-5,5])
        self.assertTrue(alignment.aligned)
        self.assertIsNone(alignment.process(None))
        self.assertEqual(finder.find.call_count,4)
        self.assertEqual(alignment.processed_frames,4)

    # ***
    # 函数名字：test_reset_and_invalid_measurement
    # 函数作用：验证新阶段清零并恢复计算，非有限偏差不会锁定。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_reset_and_invalid_measurement(self):
        finder = Mock()
        finder.find.side_effect = [{'offset':[0,0]},{'offset':[float('nan'),0]},None]
        alignment = DiscAlignment(finder,(5,5))
        alignment.process(None)
        self.assertTrue(alignment.aligned)
        alignment.reset()
        self.assertFalse(alignment.aligned)
        self.assertEqual(alignment.processed_frames,0)
        self.assertIsNone(alignment.process(None))
        self.assertFalse(alignment.aligned)
        self.assertIsNone(alignment.process(None))
        self.assertEqual(finder.find.call_count,3)

    # ***
    # 函数名字：test_config_and_project_paths
    # 函数作用：验证容差及开关拒绝，三种实机配置均解析到同一参数文件和原生库。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_config_and_project_paths(self):
        for relative in ('config/board-object.json','projects/without_radar/config/board.json',
                         'projects/with_radar/config/board.json'):
            config = load_config(ROOT/relative)
            settings = config.data['disc_centering']
            self.assertTrue(settings['enabled'])
            self.assertEqual(config.path(settings['parameter_file']),ROOT/'config/disc.json')
            self.assertEqual(config.path(settings['native_library']),ROOT/'native/lib/libcar26_disc.so')
        config = load_config(ROOT/'config/board-object.json')
        for invalid in ([True,5],[5],[-1,5],[float('nan'),5]):
            config.data['disc_centering']['tolerance_px'] = invalid
            self.assertIn('INVALID_DISC_CENTERING_TOLERANCE',config.validate())

    # ***
    # 函数名字：test_yolo_continues_and_disc_searches_until_found
    # 函数作用：验证确认后未检出或无效偏差继续逐帧检测，成功后停算并保留唯一完成事件。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_yolo_continues_and_disc_searches_until_found(self):
        config = load_config(ROOT/'config/board-object.json')
        config.data['cameras']['object']['recognition_modes']['block'].update(width=1280,height=720)
        config.data['target_focus']['enabled'] = False
        config.data['hmi']['enabled'] = False
        config.data['cameras']['object']['uvc_controls'] = {}
        module = CameraModule(config,Telemetry())
        cap,detector,finder = Mock(),Mock(),Mock()
        sequence = 0
        yolo_frames,disc_frames = [],[]
        completed = []

        # ***
        # 函数名字：read_frame
        # 函数作用：提供八张720p帧，第一/六帧触发确认，第六/八帧消费唯一成功结果。
        # 参数意义与范围：无。
        # ***
        def read_frame():
            nonlocal sequence
            sequence += 1
            if sequence in (3,4):
                self.assertEqual(module.telemetry.snapshot()['camera']['disc_state'],'searching')
                self.assertIsNotNone(module.disc_request)
                self.assertIsNone(module.pending_disc)
            if sequence==5:
                self.assertIsNone(module.disc_request)
                self.assertIsNotNone(module.pending_disc)
            if sequence in (6,8):
                completed.append(module.result())
            if sequence in (1,6):
                self.assertTrue(module.request_disc())
                self.assertFalse(module.request_disc())
            if sequence==8:
                module.stop_event.set()
            return True,np.full((720,1280,3),sequence,np.uint8)

        # ***
        # 函数名字：detect
        # 函数作用：核对YOLO处理图尺寸与帧标识，并返回有效物块。
        # 参数意义与范围：image：640×360当前缩小BGR帧。
        # ***
        def detect(image):
            self.assertEqual(image.shape,(360,640,3))
            yolo_frames.append(int(image[0,0,0]))
            return (Detection('redblock',.9,250,100,350,200),)

        # ***
        # 函数名字：find
        # 函数作用：核对同帧720p原图，依次模拟未检出、无效偏差及两次请求的有效结果。
        # 参数意义与范围：image：1280×720当前原始BGR帧。
        # ***
        def find(image):
            self.assertEqual(image.shape,(720,1280,3))
            number = int(image[0,0,0])
            disc_frames.append(number)
            return {2:None,3:{'offset':[float('nan'),0]},4:{'offset':[15,-22]},
                    7:{'offset':[0,0]}}[number]

        cap.read.side_effect = read_frame
        detector.detect.side_effect = detect
        finder.find.side_effect = find
        with patch.object(module,'_open_capture',return_value=cap),patch.object(module,'_stream_controls'), \
             patch('car26.camera.module.BpuDetector',return_value=detector), \
             patch('car26.camera.module.DiscFinder',return_value=finder):
            module._capture('object')
        self.assertIsNone(module.error)
        self.assertEqual(yolo_frames,list(range(1,9)))
        self.assertEqual(disc_frames,[2,3,4,7])
        self.assertEqual([frame.disc_request_id for frame in completed],[1,2])
        self.assertEqual([frame.disc_delta for frame in completed],[(15,-22),(0,0)])
        self.assertEqual(len(module.latest.detections),1)
        self.assertFalse(module.latest.focus_active)
        camera = module.telemetry.snapshot()['camera']
        self.assertFalse(camera['disc_aligned'])
        self.assertEqual(camera['disc_state'],'waiting_request')
        self.assertEqual(camera['disc_processed_frames'],4)
        self.assertIsNone(camera['disc'])
        self.assertIsNone(camera['disc_image_size'])
        cap.release.assert_called_once()


if __name__=='__main__':
    unittest.main()
