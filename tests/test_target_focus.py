# Purpose: test alignment geometry and block/target transitions with shared or different capture sizes.
import struct
import os
import unittest
from unittest.mock import Mock, patch
import numpy as np

from car26.camera.focus import alignment_target, map_seed, CircleFinder
from car26.camera.module import CameraModule
from car26.camera.modes import object_capture_settings
from car26.config import load_config
from car26.communication.protocol import alignment_frame
from car26.application import Application
from car26.telemetry import Telemetry
from car26.types import CameraFrame, Detection
from test_application import ROOT


# ***
# 函数名字：box
# 函数作用：创建具有指定标签和中心的人工检测框。
# 参数意义与范围：
#   label：类别字符串；范围为BLOCK_CODES或MARK_CODES中的合法类别。
#   x：水平像素或平面坐标；有限数值，单位和边界由所属图像或标定坐标系确定。
#   y：垂直像素或平面坐标；有限数值，单位和边界由所属图像或标定坐标系确定。
# ***
def box(label, x, y):
    return Detection(label, .9, x-25,y-25,x+25,y+25)


class TargetFocusTests(unittest.TestCase):
    # ***
    # 函数名字：test_scale_bounds_exclude_tiny_fragments_and_reset
    # 函数作用：验证大标靶排除细碎小圆且后续小标靶不沿用旧尺度。
    # 参数意义与范围：
    #   self：当前TargetFocusTests实例。
    # ***
    def test_scale_bounds_exclude_tiny_fragments_and_reset(self):
        finder=CircleFinder.__new__(CircleFinder)
        finder.base_settings=dict(load_config(ROOT/'config/board-object.json').data['target_focus'])
        finder.settings=dict(finder.base_settings)
        finder.set_target_scale(Detection('targetTwo',.9,228,70,368,244),(640,360),(1920,1080))
        self.assertEqual(finder.settings['minimum_radius_px'],84)
        self.assertGreater(finder.settings['maximum_radius_px'],250)
        self.assertGreater(finder.settings['search_radius_px'],350)
        finder.set_target_scale(box('targetTwo',320,100),(640,360),(1920,1080))
        self.assertEqual(finder.settings['minimum_radius_px'],30)
    # ***
    # 函数名字：marks
    # 函数作用：创建三类水平排列且标靶二最接近中心的人工检测框。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    # ***
    def marks(self):
        return (box("targetOne",100,100),box("targetTwo",320,100),box("targetThree",540,100))

    # ***
    # 函数名字：test_gate_rejects_bend_duplicate_missing_wrong_nearest
    # 函数作用：验证三标靶门控拒绝歪折、重复、缺失和最近类别错误。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    # ***
    def test_gate_rejects_bend_duplicate_missing_wrong_nearest(self):
        marks=self.marks()
        self.assertEqual(alignment_target(marks,(640,360),3).label,"targetTwo")
        self.assertIsNone(alignment_target(marks[:2],(640,360),3))
        self.assertIsNone(alignment_target(marks+(marks[0],),(640,360),3))
        self.assertIsNone(alignment_target((marks[0],box("targetTwo",320,120),box("targetThree",540,140)),(640,360),3))
        self.assertIsNone(alignment_target((box("targetTwo",100,100),box("targetOne",320,100),marks[2]),(640,360),3))
        self.assertIsNone(alignment_target((box("targetOne",220,100),box("targetTwo",420,100),marks[2]),(640,360),3))

    # ***
    # 函数名字：test_seed_and_signed_protocol
    # 函数作用：验证分辨率种子映射和正负像素差的串口编码。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    # ***
    def test_seed_and_signed_protocol(self):
        self.assertEqual(map_seed(self.marks()[1],(640,360),(1920,1080),(1,1,0,0)),(960,300))
        wire=alignment_frame(-960,540)
        self.assertEqual(wire[:2],b'[\x54')
        self.assertEqual(struct.unpack('<hh',wire[2:6]),(-960,540))
        self.assertEqual(wire[-1:],b']')
        for pair in ((32768,0),(True,0),(0,-32769)):
            with self.assertRaises(ValueError): alignment_frame(*pair)

    # ***
    # 函数名字：test_three_degree_boundary
    # 函数作用：验证三度门限附近的接受与拒绝结果。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    # ***
    def test_three_degree_boundary(self):
        self.assertIsNotNone(alignment_target((box('targetOne',100,100),box('targetTwo',320,111),box('targetThree',540,122)),(640,360),3))
        self.assertIsNone(alignment_target((box('targetOne',100,100),box('targetTwo',320,112),box('targetThree',540,124)),(640,360),3))

    # ***
    # 函数名字：test_invalid_focus_configuration
    # 函数作用：验证非法精定位配置在打开设备之前被拒绝。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    # ***
    def test_invalid_focus_configuration(self):
        for key,value in [('angle_threshold_deg',float('nan')),('fps',31),('minimum_radius_px',200),('settle_frames',0)]:
            config=load_config(ROOT/'config/board-object.json')
            config.data['target_focus'][key]=value
            self.assertTrue(config.validate())

    # ***
    # 函数名字：test_native_nearest_circle_and_blank_frame
    # 函数作用：验证原生圆检测能选择最近圆并拒绝空白及方框。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    # ***
    @unittest.skipUnless(os.environ.get('CAR26_FOCUS_LIBRARY'),'native library is built on the board')
    def test_native_nearest_circle_and_blank_frame(self):
        import cv2
        settings=load_config(ROOT/'config/board-object.json').data['target_focus']
        finder=CircleFinder(os.environ['CAR26_FOCUS_LIBRARY'],settings)
        image=np.zeros((1080,1920,3),np.uint8)
        cv2.circle(image,(970,530),35,(255,255,255),3)
        cv2.circle(image,(1080,540),30,(255,255,255),3)
        result=finder.find(image,(960,540))
        self.assertIsNotNone(result)
        self.assertLess(abs(result[0]-970),3)
        self.assertLess(abs(result[1]-530),3)
        self.assertIsNone(finder.find(np.zeros_like(image),(960,540)))
        square=np.zeros_like(image)
        cv2.rectangle(square,(925,505),(995,575),(255,255,255),3)
        self.assertIsNone(finder.find(square,(960,540)))
        partial=np.zeros_like(image)
        cv2.ellipse(partial,(970,530),(35,35),0,20,260,(255,255,255),3)
        arc=finder.find(partial,(960,540))
        self.assertIsNotNone(arc)
        self.assertLess(abs(arc[0]-970),3)
        self.assertLess(abs(arc[1]-530),3)

    # ***
    # 函数名字：test_dispatch_suppresses_missing_circle_and_pending_completion
    # 函数作用：验证无圆或结束请求后不再发送旧偏差。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    # ***
    def test_dispatch_suppresses_missing_circle_and_pending_completion(self):
        config=load_config(ROOT/'config/board-object.json')
        app=Application(config)
        app.communication=Mock()
        app._object_frame(CameraFrame('object',1,0,1920,1080,self.marks(),focus_active=True))
        app.communication.send_detections.assert_not_called()
        app.communication.send_alignment.assert_not_called()
        app._object_frame(CameraFrame('object',2,0,1920,1080,focus_active=True,focus_delta=(-4,5),target_alpha=.5))
        app.communication.send_alignment.assert_called_once_with(.5,(-4,5),1)
        app.focus_ending=True
        app._object_frame(CameraFrame('object',3,0,1920,1080,focus_active=True,focus_delta=(8,9),target_alpha=.5))
        self.assertEqual(app.communication.send_alignment.call_count,1)
        app._object_frame(CameraFrame('object',4,0,640,360,self.marks()))
        app.communication.send_detections.assert_called_once()

    # ***
    # 函数名字：capture_scenario
    # 函数作用：验证同分辨率复用采集、不同分辨率重开及精定位退出后的恢复。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   restore：测试是否模拟精定位结束后恢复；布尔值True或False。
    #   angle_outside：是否模拟OpenCV角度超出3度而回退；布尔值True或False。
    #   different_resolution：是否令物块采集1280×720、标靶采集1920×1080；布尔值。
    #   disc_enabled：是否模拟0x61圆盘对准功能并检查进入0x62后的停算及返回重置；布尔值。
    # ***
    def capture_scenario(self, restore, angle_outside=False, different_resolution=False, disc_enabled=False):
        config=load_config(ROOT/'config/board-object.json')
        # Explicit legacy auto-entry fixture; the selected board uses command entry.
        config.data['target_focus']['trigger']='auto'
        for profile in ('block','target'):
            config.data['cameras']['object']['recognition_modes'][profile].update(width=1280,height=720)
        config.data['disc_centering']['enabled']=disc_enabled
        config.data['hmi']['enabled']=False
        config.data['target_focus']['mode_transform']=[1,1,0,0] if different_resolution else [.8,.9,.1,.1]
        if different_resolution:
            config.data['cameras']['object']['recognition_modes']['block'].update(width=1280,height=720)
            config.data['cameras']['object']['recognition_modes']['target'].update(width=1920,height=1080)
        settings=config.data['cameras']['object']
        block_size=tuple(settings['recognition_modes']['block'][key] for key in ('width','height'))
        target_size=tuple(settings['recognition_modes']['target'][key] for key in ('width','height'))
        module=CameraModule(config,Telemetry())
        high=np.zeros((block_size[1],block_size[0],3),np.uint8)
        capture=Mock()
        count=0
        # ***
        # 函数名字：high_read
        # 函数作用：提供当前采集配置尺寸的人工帧，在稳定帧及算法切换完成后终止循环。
        # 参数意义与范围：
        #   无。
        # ***
        def high_read():
            nonlocal count
            count+=1
            limit=(6 if restore else 5)+(6 if restore else 3)*different_resolution
            if count==limit: module.stop_event.set()
            return True,high
        capture.read.side_effect=high_read
        finder=Mock()
        # ***
        # 函数名字：find
        # 函数作用：按测试条件模拟圆检测结果和结束指令。
        # 参数意义与范围：
        #   image：目标模式采集尺寸的BGR uint8人工图，非空H×W×3。
        #   seed：圆搜索种子(x, y)；有限像素坐标且位于当前图像边界内。
        # ***
        def find(image,seed):
            self.assertEqual(image.shape,(target_size[1],target_size[0],3))
            if restore and not angle_outside: module.finish_focus()
            baseline=target_size[1]/2-10
            return (seed[0]+10,baseline+(seed[0]-target_size[0]/2)*.1 if angle_outside else baseline,20)
        finder.find.side_effect=find
        detector=Mock()
        detector.detect.return_value=self.marks()
        disc_finder=Mock()
        disc_finder.find.return_value={'offset':[0,0]}
        modes=[]
        # ***
        # 函数名字：open_capture
        # 函数作用：记录采集模式打开请求，更新人工帧尺寸并返回假采集设备。
        # 参数意义与范围：
        #   settings：当前模块配置字典；键、类型及数值范围由Configuration.validate约束。
        # ***
        def open_capture(settings):
            nonlocal high
            modes.append((settings['width'],settings['height']))
            high=np.zeros((settings['height'],settings['width'],3),np.uint8)
            return capture
        with patch.object(module,'_open_capture',side_effect=open_capture), patch.object(module,'_stream_controls') as controls, \
             patch('car26.camera.module.BpuDetector',return_value=detector), \
             patch('car26.camera.module.DiscFinder',return_value=disc_finder), \
             patch('car26.camera.module.CircleFinder',return_value=finder):
            module._capture('object')
        self.assertIsNone(module.error)
        self.assertEqual(modes, ([block_size,target_size]+([block_size] if restore else []))
                         if different_resolution else [block_size])
        self.assertEqual(controls.call_count,len(modes))
        self.assertEqual(detector.detect.call_count,2 if restore else 1)
        self.assertEqual(module.latest.focus_active,not restore)
        self.assertEqual(module.latest.focus_delta,None if restore else (10,-10))
        self.assertEqual(capture.release.call_count,len(modes))
        for call in detector.detect.call_args_list:
            self.assertEqual(call.args[0].shape,(360,640,3))
        for call,mark in zip(finder.set_target_scale.call_args_list,self.marks()):
            self.assertEqual(call.args,(mark,(640,360),target_size))
        self.assertEqual(finder.find.call_args_list[0].args[1],(100*target_size[0]/640,100*target_size[1]/360))
        snapshot=module.telemetry.snapshot()['camera']
        self.assertEqual((snapshot['capture_width'],snapshot['capture_height']),modes[-1])
        self.assertIsNone(snapshot['recognition_mode'])
        states=(('block',True),('block',False),('target',True),('target',False))
        if restore: states += (('block',True),('block',False))
        self.assertEqual(module.drain_mode_events(),states)
        self.assertEqual((module.latest.width,module.latest.height),(640,360) if restore else target_size)
        if angle_outside:
            self.assertTrue(any(e.get('reason')=='angle_outside_threshold' for e in module.telemetry.snapshot()['events']))
        if disc_enabled:
            disc_finder.find.assert_not_called()
            self.assertEqual(snapshot['disc_processed_frames'],0)
            self.assertFalse(snapshot['disc_aligned'])

    # ***
    # 函数名字：test_disc_phase_reset_and_target_isolation
    # 函数作用：验证未收到确认时进入0x61、切至0x62及恢复0x61均不调用圆盘算法。
    # 参数意义与范围：self：当前TargetFocusTests实例。
    # ***
    def test_disc_phase_reset_and_target_isolation(self):
        self.capture_scenario(True,disc_enabled=True)

    # ***
    # 函数名字：test_camera_restores_when_angle_leaves_threshold
    # 函数作用：验证精定位采用当前圆心角度且超出阈值后恢复YOLO。
    # 参数意义与范围：
    #   self：当前TargetFocusTests实例。
    # ***
    def test_camera_restores_when_angle_leaves_threshold(self):
        self.capture_scenario(True,True)

    # ***
    # 函数名字：test_camera_pauses_yolo
    # 函数作用：验证默认720p精定位期间暂停YOLO并保持同一采集设备。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    # ***
    def test_camera_pauses_yolo(self):
        self.capture_scenario(False)

    # ***
    # 函数名字：test_camera_restores_after_done
    # 函数作用：验证结束精定位后恢复原模式且不立即重复触发。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    # ***
    def test_camera_restores_after_done(self):
        self.capture_scenario(True)

    # ***
    # 函数名字：test_capture_configuration_rejects_invalid_profiles
    # 函数作用：验证独立模式允许不同合法尺寸，拒绝非法尺寸、帧率和未知模式字段。
    # 参数意义与范围：self：当前TargetFocusTests实例。
    # ***
    def test_capture_configuration_rejects_invalid_profiles(self):
        config=load_config(ROOT/'config/board-object.json')
        profile=config.data['cameras']['object']['recognition_modes']['target']
        profile.update(width=1280,height=720)
        self.assertEqual(config.validate(),[])
        for key,value,error in [('width',0,'INVALID_OBJECT_CAPTURE_DIMENSIONS:target'),
                                ('height',True,'INVALID_OBJECT_CAPTURE_DIMENSIONS:target'),
                                ('fps',90,'INVALID_OBJECT_CAPTURE_FPS'),
                                ('typo',1,'INVALID_OBJECT_RECOGNITION_PROFILE:target')]:
            with self.subTest(key=key):
                original=profile.copy()
                profile[key]=value
                self.assertIn(error,config.validate())
                profile.clear()
                profile.update(original)

    # ***
    # 函数名字：test_camera_switches_resolution_and_restores
    # 函数作用：验证物块720p→标靶1080p→物块720p期间只有一个相机所有者且不发送稳定帧。
    # 参数意义与范围：
    #   self：当前TargetFocusTests实例。
    # ***
    def test_camera_switches_resolution_and_restores(self):
        self.capture_scenario(True,different_resolution=True)

    # ***
    # 函数名字：test_target_size_mapping_and_legacy_fallback
    # 函数作用：验证非1080p标靶模式的种子映射，并兼容无模式配置的旧文件。
    # 参数意义与范围：
    #   self：当前TargetFocusTests实例。
    # ***
    def test_target_size_mapping_and_legacy_fallback(self):
        settings=load_config(ROOT/'config/board-object.json').data['cameras']['object']
        settings['recognition_modes']['block'].update(width=1280,height=720)
        settings['recognition_modes']['target'].update(width=1280,height=720)
        resolved=object_capture_settings(settings,'target')
        size=(resolved['width'],resolved['height'])
        self.assertEqual(map_seed(self.marks()[1],(640,360),size,(1,1,0,0)),(640,200))
        self.assertEqual(object_capture_settings(settings,'block')['width'],1280)
        settings.pop('recognition_modes')
        settings.update(capture_width=1920,capture_height=1080)
        self.assertEqual(object_capture_settings(settings,'target')['width'],1920)
        with self.assertRaises(ValueError): object_capture_settings(settings,'unknown')

    # ***
    # 函数名字：test_resolution_switch_failure_clears_old_result
    # 函数作用：验证不同分辨率重开失败时先释放旧设备，且不保留可发送的旧结果。
    # 参数意义与范围：
    #   self：当前TargetFocusTests实例。
    # ***
    def test_resolution_switch_failure_clears_old_result(self):
        config=load_config(ROOT/'config/board-object.json')
        settings=config.data['cameras']['object']
        settings['recognition_modes']['target'].update(width=1920,height=1080)
        module=CameraModule(config,Telemetry())
        module.latest=CameraFrame('object',1,0,640,360,self.marks())
        old=Mock()
        with patch.object(module,'_open_capture',side_effect=RuntimeError('CAMERA_OPEN_FAILED')) as opening:
            with self.assertRaisesRegex(RuntimeError,'CAMERA_OPEN_FAILED'):
                module._switch_object_mode(old,object_capture_settings(settings,'block'),'target')
        old.release.assert_called_once()
        opening.assert_called_once()
        self.assertIsNone(module.latest)


if __name__=='__main__': unittest.main()
