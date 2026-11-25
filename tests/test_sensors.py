# Purpose: verify sensor protocols, coordinate transforms and full-frame detection behavior.
import math
import struct
import unittest

import numpy as np

from car26.camera.detector import as_hwc, decode_heads, prepare_nv12
from car26.camera.processing import filter_objects
from car26.mapping import inside_polygon
from car26.radar.processing import cluster_points, point_xy, transform_xy
from car26.radar.protocol import StandardParser, descriptor, request
from car26.types import Detection, ScanPoint


def packet(angle, distance, first=False, quality=15):
    q6, q2 = round(angle * 64), round(distance * 4000)
    return bytes(((quality << 2) | (1 if first else 2), ((q6 & 127) << 1) | 1,
                  q6 >> 7, q2 & 255, q2 >> 8))


class SensorTests(unittest.TestCase):
    # ***
    # 函数名字：test_nv12_uv_order_and_rgb_gain_before_resize
    # 函数作用：验证方形输入的颜色增益、缩放及NV12色度交错顺序。
    # 参数意义与范围：self：当前SensorTests测试实例。
    # ***
    def test_nv12_uv_order_and_rgb_gain_before_resize(self):
        import cv2
        image = np.arange(8 * 8 * 3, dtype=np.uint8).reshape(8, 8, 3)
        gains = [.953, 1, .860]
        lut = np.clip(np.arange(256, dtype=np.float32)[:, None] * np.array(gains, dtype=np.float32)[::-1], 0, 255).astype(np.uint8).reshape(1, 256, 3)
        balanced = cv2.resize(cv2.LUT(image, lut), (4, 4), interpolation=cv2.INTER_AREA)
        planar = cv2.cvtColor(balanced, cv2.COLOR_BGR2YUV_I420).reshape(-1)
        expected = np.concatenate((planar[:16], planar[16:20], planar[20:24]))
        expected[16:] = np.column_stack((planar[16:20], planar[20:24])).reshape(-1)
        np.testing.assert_array_equal(prepare_nv12(image, gains, 4), expected)

    def test_c1_descriptor_and_motor_checksum(self):
        self.assertEqual(descriptor(bytes.fromhex("a55a0500004081")), (5, 1, 0x81))
        motor = request(0xA8, struct.pack("<H", 600))
        checksum = 0
        for byte in motor:
            checksum ^= byte
        self.assertEqual(checksum, 0)
        self.assertEqual(motor[:5], bytes.fromhex("a5a8025802"))

    def test_c1_noise_fragmentation_zero_distance(self):
        parser = StandardParser()
        raw = packet(45, 1.25, True)
        self.assertEqual(parser.feed(b"\x00\x00" + raw[:3]), [])
        values = parser.feed(raw[3:] + packet(90, 0))
        self.assertEqual(len(values), 2)
        self.assertTrue(values[0][0])
        self.assertAlmostEqual(values[0][1].distance_m, 1.25)
        self.assertEqual(values[1][1].distance_m, 0)
        self.assertEqual(parser.dropped_bytes, 2)
        self.assertLess(len(parser.buffer), 5)

    def test_c1_coordinate_sign(self):
        x, y = point_xy(ScanPoint(90, 1, 15))
        self.assertAlmostEqual(x, 0)
        self.assertAlmostEqual(y, -1)

    def test_circular_cluster_and_outlier(self):
        points = (ScanPoint(0, 1, 15), ScanPoint(1, 1, 15), ScanPoint(180, 3, 15),
                  ScanPoint(359, 1, 15))
        obstacles = cluster_points(points, .05, 3)
        self.assertEqual(len(obstacles), 1)
        self.assertEqual(obstacles[0].point_count, 3)
        self.assertAlmostEqual(obstacles[0].x_m, 1, places=3)

    def test_extrinsic_then_calibrated_heading(self):
        x, y = transform_xy(1, 0, (1, 0, 90), 90)
        self.assertAlmostEqual(x, -1)
        self.assertAlmostEqual(y, 1)

    def test_roi_edge_inside(self):
        polygon = [[0, 0], [10, 0], [10, 10], [0, 10]]
        self.assertTrue(inside_polygon(0, 5, polygon))
        self.assertTrue(inside_polygon(5, 5, polygon))
        self.assertFalse(inside_polygon(11, 5, polygon))

    # ***
    # 函数名字：test_object_area_center_mark_priority
    # 函数作用：验证面积和重叠优先级不变且下半幅物块不再被中心高度过滤。
    # 参数意义与范围：self：当前SensorTests测试实例。
    # ***
    def test_object_area_center_mark_priority(self):
        settings = {"minimum_area_px": 2000,
                    "mark23_overlap": .8, "mark_block_overlap": .5}
        two = Detection("targetTwo", .8, 0, 0, 100, 100)
        three = Detection("targetThree", .7, 0, 0, 100, 100)
        small = Detection("redblock", .9, 0, 0, 10, 10)
        low = Detection("lightblueblock", .9, 0, 250, 100, 300)
        self.assertEqual(filter_objects((two, three, small, low), 480, settings), (three, low))
        block = Detection("redblock", .9, 0, 0, 100, 100)
        self.assertEqual(filter_objects((three, block), 480, settings), (block,))

    # ***
    # 函数名字：test_chw_and_hwc_dfl_equivalent_with_independent_geometry
    # 函数作用：验证两类输出布局一致且模型坐标正确撤销补边回到非方形原图。
    # 参数意义与范围：self：当前SensorTests测试实例。
    # ***
    def test_chw_and_hwc_dfl_equivalent_with_independent_geometry(self):
        outputs = []
        for side in (4, 2, 1):
            cls = np.full((1, side, side, 2), -30, dtype=np.float32)
            box = np.full((1, side, side, 64), -30, dtype=np.float32)
            box.reshape(1, side, side, 4, 16)[..., 1] = 30
            outputs.extend((cls, box))
        outputs[0][0, 1, 2, 1] = 30
        args = (["redblock", "lightblueblock"], 64, 32, .5, .5)
        hwc = decode_heads(outputs, *args, input_size=32)
        chw = decode_heads([x.transpose(0, 3, 1, 2) for x in outputs], *args, input_size=32)
        self.assertEqual(hwc, chw)
        self.assertEqual(len(hwc), 1)
        self.assertEqual(hwc[0].label, "lightblueblock")
        self.assertEqual((hwc[0].x1, hwc[0].y1, hwc[0].x2, hwc[0].y2), (24, 0, 56, 24))
        # Model cls metadata can say NCHW while its memory is NHWC.
        misleading = list(outputs)
        misleading[0] = misleading[0].reshape(1, 2, 4, 4)
        self.assertEqual(decode_heads(misleading, *args, input_size=32), hwc)

    def test_integer_output_is_not_silently_decoded(self):
        with self.assertRaisesRegex(ValueError, "QUANTIZED"):
            as_hwc(np.zeros((1, 64, 4, 4), dtype=np.int32), 64, 4)
