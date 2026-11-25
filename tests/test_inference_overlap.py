# Purpose: verify bounded BPU/capture overlap, exact frame association, confirmation freshness and cleanup.
from concurrent.futures import ThreadPoolExecutor
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch
import numpy as np
from car26.camera.detector import BpuDetector
from car26.camera.module import CameraModule
from car26.config import load_config
from car26.telemetry import Telemetry
from car26.types import CameraFrame,Detection
from test_application import ROOT


class InferenceOverlapTests(unittest.TestCase):
    # ***
    # 函数名字：detector
    # 函数作用：构建无设备的原模型接口替身，保留真实预处理和检测框解码。
    # 参数意义与范围：self：当前测试实例；forward为可选单输入模型回调，None使用固定人工输出。
    # ***
    def detector(self,forward=None):
        outputs=[]
        for side in (80,40,20):
            classes=np.full((1,side,side,2),-30,np.float32)
            boxes=np.full((1,side,side,4,16),-30,np.float32)
            boxes[...,2]=30
            outputs.extend([classes,boxes.reshape(1,side,side,64)])
        outputs[0][0,40,40,1]=4
        tensors=[SimpleNamespace(buffer=array) for array in outputs]
        value=BpuDetector.__new__(BpuDetector)
        value.settings={'rgb_gains':[1,1,1],'score_threshold':.25,'nms_threshold':.45}
        value.labels=['redblock','blackblock'];value.candidate_selector=None
        value.supports_prefetch=True;value.closed=False;value.metrics={}
        value.executor=ThreadPoolExecutor(max_workers=1,thread_name_prefix='car26-bpu-test')
        value.model=SimpleNamespace(forward=forward or Mock(return_value=tensors))
        return value,tensors

    # ***
    # 函数名字：test_same_outputs_and_one_prefetch_per_call
    # 函数作用：验证重叠与串行路径输出完全相同，每次只执行一个预取回调。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_same_outputs_and_one_prefetch_per_call(self):
        value,_=self.detector();image=np.zeros((360,640,3),np.uint8)
        try:
            before=value.detect(image)
            callback=Mock()
            after=value.detect(image,prefetch=callback)
            self.assertEqual(before,after)
            self.assertEqual((after[0].label,after[0].x1,after[0].y1,after[0].x2,after[0].y2),
                             ('blackblock',308,168,340,200))
            callback.assert_called_once_with()
            self.assertEqual(value.model.forward.call_count,2)
            self.assertTrue(value.metrics['overlapped'])
        finally:value.close()
        value.close()
        with self.assertRaisesRegex(RuntimeError,'INFERENCE_CLOSED'):value.detect(image)

    # ***
    # 函数名字：test_capture_runs_while_single_model_job_is_pending
    # 函数作用：验证模型在唯一工作线程执行，当前相机线程可在模型未完成时预取。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_capture_runs_while_single_model_job_is_pending(self):
        entered,release,finished=threading.Event(),threading.Event(),threading.Event()
        value,tensors=self.detector();owner=threading.get_ident();threads=[]
        # ***
        # 函数名字：forward
        # 函数作用：模拟必须等预取回调释放后才完成的BPU任务。
        # 参数意义与范围：nv12为独立的614400字节uint8模型输入。
        # ***
        def forward(nv12):
            threads.append(threading.get_ident());entered.set()
            if not release.wait(2):raise RuntimeError('MODEL_NOT_OVERLAPPED')
            finished.set();return tensors
        value.model.forward=forward
        # ***
        # 函数名字：prefetch
        # 函数作用：核对相机所有者线程，并在模型仍执行时完成一次预取。
        # 参数意义与范围：无。
        # ***
        def prefetch():
            self.assertEqual(threading.get_ident(),owner)
            self.assertTrue(entered.wait(2));self.assertFalse(finished.is_set());release.set()
        try:value.detect(np.zeros((360,640,3),np.uint8),prefetch=prefetch)
        finally:release.set();value.close()
        self.assertEqual(len(threads),1);self.assertNotEqual(threads[0],owner)
        self.assertTrue(finished.is_set())

    # ***
    # 函数名字：test_prefetch_and_model_failures_drain_and_close
    # 函数作用：验证预取异常和模型异常均不返回旧检测，并能等待任务结束及关闭线程。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_prefetch_and_model_failures_drain_and_close(self):
        image=np.zeros((360,640,3),np.uint8)
        value,_=self.detector()
        try:
            with self.assertRaisesRegex(RuntimeError,'CAPTURE_FAILED'):
                value.detect(image,prefetch=Mock(side_effect=RuntimeError('CAPTURE_FAILED')))
            value.model.forward.assert_called_once()
        finally:value.close()
        value,_=self.detector(Mock(side_effect=RuntimeError('BPU_FAILED')))
        try:
            with self.assertRaisesRegex(RuntimeError,'BPU_FAILED'):value.detect(image,prefetch=Mock())
        finally:value.close()

    # ***
    # 函数名字：camera_scenario
    # 函数作用：用逐帧标识检查相机线程、时间戳、模式切换及确认门控，不打开设备。
    # 参数意义与范围：self：当前测试实例；scenario为normal、focus或disc。
    # ***
    def camera_scenario(self,scenario):
        config=load_config(ROOT/'config/board-object.json')
        config.data['target_focus']['trigger']='auto'
        for profile in ('block','disc','target'):
            config.data['cameras']['object']['recognition_modes'][profile].update(width=1280,height=720)
        config.data['hmi']['enabled']=False
        config.data['target_focus']['enabled']=scenario=='focus'
        config.data['disc_centering']['enabled']=scenario=='disc'
        config.data['cameras']['object']['uvc_controls']={}
        module=CameraModule(config,Telemetry());capture=Mock()
        detector=Mock(supports_prefetch=True,metrics={})
        owner=threading.get_ident();calls=[];stamps={};frames=[];tick=[0.0]
        marks=tuple(Detection(label,.9,x-25,75,x+25,125) for label,x in
                    [('targetOne',100),('targetTwo',320),('targetThree',540)])
        # ***
        # 函数名字：clock
        # 函数作用：提供单调递增的测试时钟，识别缓存帧是否错误改写采集时间。
        # 参数意义与范围：无。
        # ***
        def clock():
            tick[0]+=.001;return tick[0]
        # ***
        # 函数名字：read
        # 函数作用：在唯一相机线程产生带编号720p帧，并在预取中模拟MCU确认。
        # 参数意义与范围：无。
        # ***
        def read():
            self.assertEqual(threading.get_ident(),owner)
            number=len(stamps)+1;stamps[number]=tick[0]
            if scenario=='normal' and number==3:module.stop_event.set()
            if scenario=='disc' and number==2:self.assertTrue(module.request_disc())
            return True,np.full((720,1280,3),number,np.uint8)
        capture.read.side_effect=read
        # ***
        # 函数名字：detect
        # 函数作用：记录当前帧编号，仅在传入回调时读取下一帧。
        # 参数意义与范围：image为640×360 BGR图；prefetch为None或无参数预取回调。
        # ***
        def detect(image,prefetch=None):
            calls.append(int(image[0,0,0]))
            if prefetch is not None:prefetch()
            return marks if scenario=='focus' else (Detection('redblock',.9,100,100,200,200),)
        detector.detect.side_effect=detect
        circle=Mock();disc=Mock();circle_images=[];disc_images=[]
        # ***
        # 函数名字：find_circle
        # 函数作用：记录精定位图编号并结束测试，返回合法的当前种子圆。
        # 参数意义与范围：image为720p BGR图；seed为图内有限(x,y)坐标。
        # ***
        def find_circle(image,seed):
            circle_images.append(int(image[0,0,0]));module.stop_event.set();return (*seed,20)
        circle.find.side_effect=find_circle
        # ***
        # 函数名字：find_disc
        # 函数作用：记录确认后的原图编号并返回一次合法偏差。
        # 参数意义与范围：image为720p原始BGR图。
        # ***
        def find_disc(image):
            disc_images.append(int(image[0,0,0]));module.stop_event.set();return {'offset':[2,-3]}
        disc.find.side_effect=find_disc
        # ***
        # 函数名字：make_frame
        # 函数作用：保留真实不可变CameraFrame，并记录发布时的采集时间戳。
        # 参数意义与范围：args为CameraFrame位置参数；kwargs为对应可选关键字参数。
        # ***
        def make_frame(*args,**kwargs):
            value=CameraFrame(*args,**kwargs);frames.append(value);return value
        with patch('car26.camera.module.time.monotonic',side_effect=clock), \
             patch.object(module,'_open_capture',return_value=capture),patch.object(module,'_stream_controls'), \
             patch('car26.camera.module.BpuDetector',return_value=detector), \
             patch('car26.camera.module.CircleFinder',return_value=circle), \
             patch('car26.camera.module.DiscFinder',return_value=disc), \
             patch('car26.camera.module.CameraFrame',side_effect=make_frame):
            module._capture('object')
        self.assertIsNone(module.error)
        capture.release.assert_called_once();detector.close.assert_called_once()
        return calls,stamps,frames,circle_images,disc_images

    # ***
    # 函数名字：test_prefetched_frame_keeps_original_timestamp_and_owner
    # 函数作用：验证只预取一帧，第二帧时间戳来自其真实read而非下轮处理开始。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_prefetched_frame_keeps_original_timestamp_and_owner(self):
        calls,stamps,frames,_,_=self.camera_scenario('normal')
        self.assertEqual(calls,[1,2]);self.assertEqual(list(stamps),[1,2,3])
        self.assertEqual([value.timestamp for value in frames],[stamps[1],stamps[2]])

    # ***
    # 函数名字：test_focus_discards_prefetch_before_target_mode
    # 函数作用：验证进入精定位时丢弃旧功能预取帧，三个圆使用模式进入后的新帧。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_focus_discards_prefetch_before_target_mode(self):
        calls,stamps,frames,circles,_=self.camera_scenario('focus')
        self.assertEqual(calls,[1]);self.assertEqual(circles,[3,3,3])
        self.assertTrue(frames[-1].focus_active);self.assertEqual(frames[-1].timestamp,stamps[3])

    # ***
    # 函数名字：test_disc_confirmation_discards_older_prefetch
    # 函数作用：验证预取过程中到达的确认不使用旧缓存帧，搜索时暂停预取。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_disc_confirmation_discards_older_prefetch(self):
        calls,stamps,frames,_,discs=self.camera_scenario('disc')
        self.assertEqual(calls,[1,3]);self.assertEqual(discs,[3])
        self.assertEqual(frames[-1].timestamp,stamps[3]);self.assertEqual(frames[-1].disc_delta,(2,-3))

    # ***
    # 函数名字：test_overlap_option_requires_boolean
    # 函数作用：验证开关拒绝整数及字符串，避免配置意外启用工作线程。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_overlap_option_requires_boolean(self):
        for value in (0,1,'true'):
            config=load_config(ROOT/'config/board-object.json')
            config.data['models']['object']['overlap_capture']=value
            self.assertIn('INVALID_INFERENCE_OVERLAP:object',config.validate())


if __name__=='__main__':unittest.main()
