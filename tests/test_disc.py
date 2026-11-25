# Purpose: check large-disc fitting under clipping, occlusion and perspective, and reject holes or false shapes.
import json
import os
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from car26.camera.disc import DiscFinder, load_disc_settings

ROOT = Path(__file__).resolve().parents[1]


class DiscTests(unittest.TestCase):
    # ***
    # 函数名字：test_invalid_config
    # 函数作用：拒绝错误类型、非有限数、非法核大小及缺失参数。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_invalid_config(self):
        for key,value in [('iterations',True),('inlier_distance_px',float('nan')),
                          ('morphology_size',4),('minimum_axis_ratio',0),('minimum_enclosure_ratio',1.1)]:
            settings = load_disc_settings(ROOT/'config/disc.json')
            settings[key] = value
            with tempfile.TemporaryDirectory(dir=ROOT/'artifacts') as folder:
                config = Path(folder)/'disc.json'
                config.write_text(json.dumps(settings),encoding='utf-8')
                with self.assertRaises(ValueError):
                    load_disc_settings(config)

    # ***
    # 函数名字：finder
    # 函数作用：加载本机原生库进行真实图像算法测试。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def finder(self):
        library = os.environ.get('CAR26_DISC_LIBRARY')
        if not library:
            self.skipTest('CAR26_DISC_LIBRARY is required for native image tests')
        return DiscFinder(library,ROOT/'config/disc.json')

    # ***
    # 函数名字：test_clipped_disc_and_foreground_hole
    # 函数作用：恢复上缘出画且下方有夹具遮挡的大圆盘，避免选择夹具圆孔。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_clipped_disc_and_foreground_hole(self):
        finder = self.finder()
        image = np.full((1080,1920,3),(35,65,40),np.uint8)
        cv2.ellipse(image,(830,50),(730,705),8,0,360,(80,80,80),-1)
        cv2.rectangle(image,(760,600),(1050,1079),(100,100,100),-1)
        cv2.circle(image,(900,600),170,(40,40,40),-1)
        result = finder.find(image)
        self.assertIsNotNone(result)
        self.assertLess(np.linalg.norm(np.array(result['center'])-[830,50]),8)
        self.assertLess(result['offset'][0],0)
        self.assertLess(result['offset'][1],0)

    # ***
    # 函数名字：test_full_perspective_ellipse
    # 函数作用：检测画面内倾斜椭圆并输出正确符号的像素偏差。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_full_perspective_ellipse(self):
        finder = self.finder()
        image = np.full((1080,1920,3),(35,65,40),np.uint8)
        cv2.ellipse(image,(1020,570),(420,310),22,0,360,(85,85,85),-1)
        result = finder.find(image)
        self.assertIsNotNone(result)
        self.assertLess(np.linalg.norm(np.array(result['center'])-[1020,570]),5)
        self.assertGreater(result['offset'][0],0)
        self.assertGreater(result['offset'][1],0)

    # ***
    # 函数名字：test_reject_blank_rectangle_small_hole_and_short_arc
    # 函数作用：拒绝无盘、矩形、只有小孔以及可见弧不足的场景，连续调用不保留旧结果。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_reject_blank_rectangle_small_hole_and_short_arc(self):
        finder = self.finder()
        for kind in ('blank','rectangle','hole','short_arc'):
            with self.subTest(kind=kind):
                image = np.full((1080,1920,3),(35,65,40),np.uint8)
                if kind=='rectangle':
                    cv2.rectangle(image,(300,100),(1500,900),(80,80,80),-1)
                elif kind=='hole':
                    cv2.circle(image,(900,600),170,(100,100,100),-1)
                    cv2.circle(image,(900,600),140,(20,20,20),-1)
                elif kind=='short_arc':
                    cv2.circle(image,(960,-600),750,(80,80,80),-1)
                self.assertIsNone(finder.find(image))

    # ***
    # 函数名字：test_invalid_image
    # 函数作用：在进入C接口之前拒绝灰度、浮点、空数组及错误通道数。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_invalid_image(self):
        finder = self.finder()
        for image in (None,np.zeros((100,100),np.uint8),np.zeros((100,100,3),np.float32),np.zeros((0,0,3),np.uint8)):
            with self.assertRaises(ValueError):
                finder.find(image)

    # ***
    # 函数名字：test_real_clipped_disc_and_loss
    # 函数作用：验证实拍低照度且被夹具遮挡的外圆、镜像场景及成功后的目标丢失。
    # 参数意义与范围：self：当前测试实例；实拍参考范围为人工轮廓估计，不是物理标定真值。
    # ***
    def test_real_clipped_disc_and_loss(self):
        finder = self.finder()
        image = cv2.imread(str(ROOT/'tests/data/disc_clipped_scene.jpg'))
        self.assertIsNotNone(image)
        for mirrored in (False,True):
            frame = cv2.flip(image,1) if mirrored else image
            result = finder.find(frame)
            self.assertIsNotNone(result)
            expected_x = image.shape[1]-1-830 if mirrored else 830
            self.assertLess(abs(result['center'][0]-expected_x),40)
            self.assertLess(abs(result['center'][1]-40),50)
            self.assertGreater(min(result['axes']),1300)
        self.assertIsNone(finder.find(np.zeros_like(image)))

    # ***
    # 函数名字：test_real_bright_color_cast
    # 函数作用：在较亮且偏蓝的实拍盘面上使用亮度分支，避免固定低饱和阈值漏检。
    # 参数意义与范围：self：当前测试实例；人工轮廓中心估计约(800,60)，容差用于图像回归而非物理精度。
    # ***
    def test_real_bright_color_cast(self):
        finder = self.finder()
        image = cv2.imread(str(ROOT/'tests/data/disc_bright_scene.jpg'))
        self.assertIsNotNone(image)
        result = finder.find(image)
        self.assertIsNotNone(result)
        self.assertEqual(result['segmentation'],'brightness')
        self.assertLess(abs(result['center'][0]-800),40)
        self.assertLess(abs(result['center'][1]-60),50)
        self.assertGreater(min(result['axes']),1250)

    # ***
    # 函数名字：test_colored_objects_cross_outer_rim
    # 函数作用：验证彩色物块跨越盘缘时保持外盘几何，拒绝拟合局部轮廓的小椭圆。
    # 参数意义与范围：self：当前测试实例；人工圆盘中心为(830,50)，物块遮挡左侧和右下盘缘。
    # ***
    def test_colored_objects_cross_outer_rim(self):
        finder = self.finder()
        image = np.full((1080,1920,3),(35,65,40),np.uint8)
        cv2.ellipse(image,(830,50),(730,705),8,0,360,(150,140,135),-1)
        cv2.ellipse(image,(120,180),(130,180),25,0,360,(180,35,20),-1)
        cv2.ellipse(image,(1250,550),(180,160),45,0,360,(35,140,35),-1)
        result = finder.find(image)
        self.assertIsNotNone(result)
        self.assertLess(np.linalg.norm(np.array(result['center'])-[830,50]),15)
        self.assertGreater(min(result['axes']),1350)


if __name__ == '__main__':
    unittest.main()
