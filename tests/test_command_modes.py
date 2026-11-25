# Purpose: verify commanded 640x360-to-720p transitions, frozen YOLO seeds and single camera ownership.
import unittest
from unittest.mock import Mock,patch
import numpy as np
from car26.application import Application
from car26.camera.module import CameraModule
from car26.camera.modes import object_capture_settings
from car26.communication.module import CommunicationModule
from car26.communication.protocol import state_frame,TARGET_FOCUS_REQUEST
from car26.config import load_config
from car26.telemetry import Telemetry
from car26.types import Detection,CameraFrame
from test_application import ROOT


# ***
# 函数名字：marks
# 函数作用：创建三个唯一水平标靶，偏移用于区分请求快照与后续YOLO结果。
# 参数意义与范围：shift为有限水平像素偏移，测试使用0或30。
# ***
def marks(shift=0):
    return tuple(Detection(label,.9,x+shift-25,75,x+shift+25,125)
                 for label,x in [('targetOne',100),('targetTwo',320),('targetThree',540)])


class CommandModeTests(unittest.TestCase):
    # ***
    # 函数名字：test_selected_profiles_and_command_trigger
    # 函数作用：验证三个入口均实际640采集，圆盘与标靶档位均720p，且默认不自动精定位。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_selected_profiles_and_command_trigger(self):
        for name in ['config/board-object.json','projects/without_radar/config/board.json','projects/with_radar/config/board.json']:
            cfg=load_config(ROOT/name);settings=cfg.data['cameras']['object']
            self.assertEqual(tuple(object_capture_settings(settings,'block')[key] for key in ('width','height')),(640,360))
            for mode in ('disc','target'):
                self.assertEqual(tuple(object_capture_settings(settings,mode)[key] for key in ('width','height')),(1280,720))
            self.assertEqual(cfg.data['target_focus']['trigger'],'command')

    # ***
    # 函数名字：test_focus_request_freezes_fresh_yolo_and_rejects_busy_stale_or_missing
    # 函数作用：验证冻结最近结果、重复合并、陈旧/缺失标靶拒绝及取消清理。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_focus_request_freezes_fresh_yolo_and_rejects_busy_stale_or_missing(self):
        module=CameraModule(load_config(ROOT/'config/board-object.json'),Telemetry())
        module._set_recognition_mode('block')
        self.assertFalse(module.request_focus())
        module.latest_yolo=(1,9.8,640,360,marks())
        with patch('car26.camera.module.time.monotonic',return_value=10):
            self.assertTrue(module.request_focus())
            saved=module.focus_request
            module.latest_yolo=(2,9.9,640,360,marks(30))
            self.assertEqual(saved[2][4],marks())
            self.assertFalse(module.request_focus());self.assertFalse(module.request_disc())
        module.finish_focus();self.assertIsNone(module.focus_request)
        with patch('car26.camera.module.time.monotonic',return_value=12):self.assertFalse(module.request_focus())
        module.latest_yolo=(3,12,640,360,marks()[:2])
        with patch('car26.camera.module.time.monotonic',return_value=12):self.assertFalse(module.request_focus())
        module._set_recognition_mode('target');self.assertFalse(module.request_focus())

    # ***
    # 函数名字：test_only_raw_enter_frame_creates_target_identity_request
    # 函数作用：验证仅原始0x62进入产生内部标识，退出和同名ASCII不能冒充。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_only_raw_enter_frame_creates_target_identity_request(self):
        transport=Mock();transport.read.return_value=state_frame(0x62,1)+state_frame(0x62,0)+b'[focus_start]'
        comm=CommunicationModule({},Telemetry(),transport)
        actions=comm.poll()
        self.assertIs(actions[0],TARGET_FOCUS_REQUEST);self.assertEqual(actions[1],'focus_start')
        self.assertEqual(comm.peer_states,set());transport.write.assert_not_called()

    # ***
    # 函数名字：test_late_disc_request_cannot_measure_low_resolution_frame
    # 函数作用：验证指令在切档检查后到达时，仍不允许在640帧上运行圆盘检测。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_late_disc_request_cannot_measure_low_resolution_frame(self):
        module=CameraModule(load_config(ROOT/'config/board-object.json'),Telemetry())
        module._set_recognition_mode('block');module.capture_profile='block'
        with patch('car26.camera.module.time.monotonic',return_value=10):self.assertTrue(module.request_disc())
        module.disc_finder=Mock();module.disc_finder.find.return_value={'offset':[1,2]}
        self.assertEqual(module._measure_disc(np.zeros((360,640,3),np.uint8),11),(0,None,None,0.0))
        module.disc_finder.find.assert_not_called();self.assertIsNotNone(module.disc_request)
        module.capture_profile='disc'
        completed,delta,_,_=module._measure_disc(np.zeros((720,1280,3),np.uint8),12)
        self.assertEqual((completed,delta),(1,(1,2)));module.disc_finder.find.assert_called_once()

    # ***
    # 函数名字：test_application_routes_raw_request_without_switching_devices_on_main_thread
    # 函数作用：验证主线程仅登记精定位请求，不把标识交给Mission或直接操作设备。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_application_routes_raw_request_without_switching_devices_on_main_thread(self):
        cfg=load_config(ROOT/'config/board-object.json');cfg.data['hmi']['enabled']=False
        app=Application(cfg);transport=Mock();transport.write.side_effect=len
        transport.read.return_value=state_frame(0x62,1)
        app.communication=CommunicationModule(cfg.data['communication'],app.telemetry,transport)
        app.mission.command=Mock(return_value=[]);app.focus_ending=True
        # ***
        # 函数名字：open_mode
        # 函数作用：模拟物块进入并提供最近YOLO结果，不打开设备。
        # 参数意义与范围：role为object。
        # ***
        def open_mode(role):
            app.camera.role=role;app.camera._set_recognition_mode('block')
            import time
            app.camera.latest_yolo=(1,time.monotonic(),640,360,marks())
        # ***
        # 函数名字：stop
        # 函数作用：在主循环一轮后停止测试。
        # 参数意义与范围：timeout为非负等待秒数，测试不实际等待。
        # ***
        def stop(timeout):app.stop_event.set()
        app.stop_event.wait=Mock(side_effect=stop)
        with patch.object(app.camera,'switch',side_effect=open_mode),patch.object(app.camera,'close'), \
             patch.object(app.camera,'request_focus',wraps=app.camera.request_focus) as request, \
             patch.object(app.camera,'_open_capture') as opening,patch('car26.application.signal.signal'):
            self.assertEqual(app.run(),0)
        request.assert_called_once();opening.assert_not_called();app.mission.command.assert_not_called()
        self.assertFalse(app.focus_ending)
        self.assertTrue(any(event.get('kind')=='focus_request' and event.get('accepted') for event in app.telemetry.snapshot()['events']))

    # ***
    # 函数名字：run_flow
    # 函数作用：模拟真实分辨率重开、稳定帧、指令和定位结果，记录各帧来源及资源所有者。
    # 参数意义与范围：self为测试实例；kind为idle/target/disc；restore表示结束后回640；stale表示故意延迟请求；fail_open表示720重开失败。
    # ***
    def run_flow(self,kind,restore=False,stale=False,fail_open=False):
        cfg=load_config(ROOT/'config/board-object.json')
        cfg.data['hmi']['enabled']=False;cfg.data['cameras']['object']['uvc_controls']={}
        cfg.data['target_focus']['settle_frames']=1
        cfg.data['target_focus']['enabled']=kind!='disc'
        cfg.data['disc_centering']['enabled']=kind=='disc'
        module=CameraModule(cfg,Telemetry());detector=Mock(supports_prefetch=True,metrics={})
        clock=[0.0];state={'size':(640,360),'reads':0,'owner':None}
        opens=[];readings=[];seen=[];seeds=[];disc_images=[];published=[];completed=[]
        # ***
        # 函数名字：tick
        # 函数作用：提供确定的单调时间，独立验证快照过期与原始帧时间。
        # 参数意义与范围：无。
        # ***
        def tick():clock[0]+=.001;return clock[0]
        # ***
        # 函数名字：read
        # 函数作用：生成当前真实档位尺寸的编号帧，第三次读取时提交指令。
        # 参数意义与范围：无。
        # ***
        def read():
            state['reads']+=1;n=state['reads'];width,height=state['size']
            readings.append((n,(width,height)))
            if n==3 and kind in ('target','disc'):
                accepted=module.request_focus() if kind=='target' else module.request_disc()
                self.assertTrue(accepted)
                if stale:clock[0]+=2
            if (kind=='idle' or stale) and n==6:module.stop_event.set()
            if restore and len(opens)==3 and n>=(8 if kind=='disc' else 7):module.stop_event.set()
            if kind=='disc' and len(opens)==3 and module.pending_disc is not None:
                completed.append(module.result())
            return True,np.full((height,width,3),n,np.uint8)
        # ***
        # 函数名字：open_capture
        # 函数作用：保证旧设备先释放，再建立唯一新档位所有者，可模拟失败。
        # 参数意义与范围：settings为已解析采集参数字典。
        # ***
        def open_capture(settings):
            self.assertIsNone(state['owner'])
            size=(settings['width'],settings['height']);opens.append(size)
            if fail_open and len(opens)==2:raise RuntimeError('CAMERA_OPEN_FAILED:object')
            capture=Mock();capture.read.side_effect=read;identity=len(opens)
            state['owner']=identity;state['size']=size
            # ***
            # 函数名字：release
            # 函数作用：幂等释放本次设备所有者，不影响新档位所有者。
            # 参数意义与范围：无。
            # ***
            def release():
                if state['owner']==identity:state['owner']=None
            capture.release.side_effect=release;return capture
        # ***
        # 函数名字：detect
        # 函数作用：记录640处理图并在模型等待期预取，第二次结果故意改变位置。
        # 参数意义与范围：image为640×360 BGR图；prefetch为None或无参数回调。
        # ***
        def detect(image,prefetch=None):
            self.assertEqual(image.shape,(360,640,3));seen.append(int(image[0,0,0]))
            if prefetch is not None:prefetch()
            return marks(30 if len(seen)==2 else 0)
        detector.detect.side_effect=detect;circle=Mock();disc=Mock()
        # ***
        # 函数名字：find_circle
        # 函数作用：核对720p局部精定位种子，按测试条件请求结束或停止测试。
        # 参数意义与范围：image为720p原图；seed为YOLO快照映射后的(x,y)。
        # ***
        def find_circle(image,seed):
            self.assertEqual(image.shape,(720,1280,3));seeds.append(seed)
            if restore:module.finish_focus()
            else:module.stop_event.set()
            return (*seed,20)
        circle.find.side_effect=find_circle
        # ***
        # 函数名字：find_disc
        # 函数作用：在720p先模拟未检出再检出，验证持续搜索及恢复低分辨率。
        # 参数意义与范围：image为720p原图。
        # ***
        def find_disc(image):
            self.assertEqual(image.shape,(720,1280,3));disc_images.append(int(image[0,0,0]))
            return None if len(disc_images)==1 else {'offset':[5,-4]}
        disc.find.side_effect=find_disc
        # ***
        # 函数名字：make_frame
        # 函数作用：记录真实不可变结果，检查原图来源与已完成偏差。
        # 参数意义与范围：args/kwargs为CameraFrame构造参数。
        # ***
        def make_frame(*args,**kwargs):
            value=CameraFrame(*args,**kwargs);published.append(value);return value
        with patch('car26.camera.module.time.monotonic',side_effect=tick), \
             patch.object(module,'_open_capture',side_effect=open_capture),patch.object(module,'_stream_controls'), \
             patch('car26.camera.module.BpuDetector',return_value=detector), \
             patch('car26.camera.module.CircleFinder',return_value=circle),patch('car26.camera.module.DiscFinder',return_value=disc), \
             patch('car26.camera.module.CameraFrame',side_effect=make_frame):
            module._capture('object')
        self.assertIsNone(state['owner']);detector.close.assert_called_once()
        if fail_open:self.assertIn('CAMERA_OPEN_FAILED',module.error)
        else:self.assertIsNone(module.error)
        return {'module':module,'opens':opens,'reads':readings,'seen':seen,'seeds':seeds,
                'disc_images':disc_images,'frames':published,'completed':completed}

    # ***
    # 函数名字：test_aligned_yolo_does_not_auto_enter_target
    # 函数作用：验证即使三个标靶满足旧门控，没有指令也始终实际640采集。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_aligned_yolo_does_not_auto_enter_target(self):
        result=self.run_flow('idle')
        self.assertEqual(result['opens'],[(640,360)])
        self.assertFalse(any(frame.focus_active for frame in result['frames']))

    # ***
    # 函数名字：test_target_switch_uses_frozen_latest_yolo_and_restores
    # 函数作用：验证640→720→640且种子取指令到达时的最近结果，不取后续改变的结果。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_target_switch_uses_frozen_latest_yolo_and_restores(self):
        result=self.run_flow('target',restore=True)
        self.assertEqual(result['opens'],[(640,360),(1280,720),(640,360)])
        self.assertEqual(result['seeds'][:3],[(200,200),(640,200),(1080,200)])
        self.assertEqual(result['frames'][-1].width,640);self.assertFalse(result['frames'][-1].focus_active)

    # ***
    # 函数名字：test_disc_switch_and_one_completed_event
    # 函数作用：验证圆盘720p持续检测至检出，完成事件保留并恢复640，业务仍为0x61。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_disc_switch_and_one_completed_event(self):
        result=self.run_flow('disc',restore=True)
        self.assertEqual(result['opens'],[(640,360),(1280,720),(640,360)])
        self.assertEqual(result['disc_images'],[5,6])
        self.assertEqual(len(result['completed']),1);self.assertEqual(result['completed'][0].disc_delta,(5,-4))
        self.assertEqual(result['module'].drain_mode_events(),(('block',True),('block',False)))

    # ***
    # 函数名字：test_expired_request_does_not_switch
    # 函数作用：验证排队期间过期的YOLO快照不会被新结果替换后偷偷启动精定位。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_expired_request_does_not_switch(self):
        result=self.run_flow('target',stale=True)
        self.assertEqual(result['opens'],[(640,360)]);self.assertEqual(result['seeds'],[])

    # ***
    # 函数名字：test_failed_high_resolution_open_releases_old_owner
    # 函数作用：验证720p重开失败时旧设备已释放，不发布精定位结果。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_failed_high_resolution_open_releases_old_owner(self):
        result=self.run_flow('target',fail_open=True)
        self.assertEqual(result['opens'],[(640,360),(1280,720)]);self.assertEqual(result['seeds'],[])


if __name__=='__main__':unittest.main()
