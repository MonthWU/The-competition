# Purpose: prevent stretched model input, mismatched padding and lower-half target suppression.
import unittest

import cv2
import numpy as np

from car26.camera.detector import letterbox_geometry, prepare_nv12
from car26.camera.processing import filter_objects
from car26.types import Detection


class LetterboxTests(unittest.TestCase):
    # ***
    # 函数名字：test_landscape_portrait_and_odd_padding
    # 函数作用：验证横向、纵向和奇数补边的整数几何，防止反变换使用不同偏移。
    # 参数意义与范围：self：当前LetterboxTests测试实例。
    # ***
    def test_landscape_portrait_and_odd_padding(self):
        self.assertEqual(letterbox_geometry(640,360), (640,360,0,140))
        self.assertEqual(letterbox_geometry(360,640), (360,640,140,0))
        self.assertEqual(letterbox_geometry(640,361), (640,361,0,139))
        self.assertEqual(letterbox_geometry(1280,720), (640,360,0,140))

    # ***
    # 函数名字：test_nv12_retains_aspect_ratio_with_neutral_padding
    # 函数作用：从NV12反解图像验证横向输入保留2比1形状、上下补边为中性灰。
    # 参数意义与范围：self：当前LetterboxTests测试实例。
    # ***
    def test_nv12_retains_aspect_ratio_with_neutral_padding(self):
        image = np.full((2,4,3),255,dtype=np.uint8)
        nv12 = prepare_nv12(image,[1,1,1],8).reshape(12,8)
        decoded = cv2.cvtColor(nv12,cv2.COLOR_YUV2BGR_NV12)
        np.testing.assert_allclose(decoded[:2],114,atol=1)
        np.testing.assert_allclose(decoded[2:6],255,atol=1)
        np.testing.assert_allclose(decoded[6:],114,atol=1)

    # ***
    # 函数名字：test_blocks_and_all_marks_survive_below_midline
    # 函数作用：验证三种标靶和物块均可在下半幅保留，且旧高度配置不再影响筛选。
    # 参数意义与范围：self：当前LetterboxTests测试实例。
    # ***
    def test_blocks_and_all_marks_survive_below_midline(self):
        detections = tuple(Detection(label,.9,index*120,220,index*120+80,300)
                           for index,label in enumerate(('targetOne','targetTwo','targetThree','redblock')))
        settings = dict(minimum_area_px=2000,mark23_overlap=.8,mark_block_overlap=.5,max_center_y_ratio=0)
        self.assertEqual(filter_objects(detections,360,settings),detections)
