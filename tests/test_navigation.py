# Purpose: validate both navigation variants with synthetic maps, poses and radar frames only.
import json
import tempfile
import unittest
from unittest.mock import Mock, patch

from car26.application import Application
from car26.config import Configuration, load_config
from car26.navigation import NavigationModule
from car26.planning import LanePlanner, validate_map
from car26.telemetry import Telemetry
from car26.types import RadarFrame, ScanPoint
from test_application import ROOT


# ***
# 函数名字：sample_map
# 函数作用：返回具有直线及绕行支路的人工地图；不写入实机配置。
# 参数意义与范围：无。
# ***
def sample_map():
    return {'confirmed': True, 'frame': 'test_field', 'vehicle_radius_m': .1, 'margin_m': .05,
            'nodes': {'A': [0, 0], 'B': [1, 0], 'C': [2, 0], 'D': [0, 1], 'E': [2, 1]},
            'edges': [{'nodes': pair, 'clearance_m': .5} for pair in
                      [('A', 'B'), ('B', 'C'), ('A', 'D'), ('D', 'E'), ('E', 'C')]],
            'start_node': 'A', 'goals': ['C'], 'fixed_obstacles': []}


class NavigationTests(unittest.TestCase):
    # ***
    # 函数名字：fixture
    # 函数作用：创建临时规划配置与定位快照，不访问真实设备。
    # 参数意义与范围：
    #   self：当前NavigationTests实例。
    #   mode：fixed或dynamic，选择对应分项目的模板。
    #   data：人工地图字典；None使用sample_map。
    # ***
    def fixture(self, mode='dynamic', data=None):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        config = load_config(ROOT / 'projects/with_radar/config/board.json')
        config = Configuration(config.data, directory.name)
        config.data['navigation'].update(mode=mode, enabled=True)
        config.data['radar'].update(enabled=mode == 'dynamic', device='fake-radar', extrinsic=[0, 0, 0], extrinsic_calibrated=True)
        config.data['navigation']['pose_file'] = 'pose.json'
        config.path('route.json').write_text(json.dumps(sample_map() if data is None else data), encoding='utf-8')
        self.pose(config)
        telemetry = Telemetry()
        return config, NavigationModule(config, telemetry), telemetry

    # ***
    # 函数名字：pose
    # 函数作用：写入人工定位快照并允许覆盖字段以构造异常场景。
    # 参数意义与范围：
    #   self：当前NavigationTests实例。
    #   config：临时Configuration对象。
    #   changes：待覆盖的定位字段，允许无效值用于拒绝测试。
    # ***
    def pose(self, config, **changes):
        value = dict(valid=True, frame='test_field', x_m=0, y_m=0, yaw_deg=0, timestamp_unix=100)
        value.update(changes)
        config.path('pose.json').write_text(json.dumps(value), encoding='utf-8')

    # ***
    # 函数名字：scan
    # 函数作用：创建人工雷达帧，默认远处点不阻断测试道路。
    # 参数意义与范围：
    #   self：当前NavigationTests实例。
    #   sequence：正整数帧序号，默认1。
    #   timestamp：单调时间秒数，默认10。
    #   points：ScanPoint元组；None使用道路外的远处人工点。
    # ***
    def scan(self, sequence=1, timestamp=10, points=None):
        return RadarFrame(sequence, timestamp, (ScanPoint(0, 10, 10),) if points is None else points, ())

    # ***
    # 函数名字：test_fixed_route_and_return_visit_order
    # 函数作用：验证固定路线沿最短道路依次访问任务点并返回原起点。
    # 参数意义与范围：self：当前NavigationTests实例。
    # ***
    def test_fixed_route_and_return_visit_order(self):
        data = sample_map()
        data['goals'] = ['C', 'A']
        result = LanePlanner(data).plan()
        self.assertEqual(result['node_ids'], ['A', 'B', 'C', 'B', 'A'])
        self.assertEqual(result['length_m'], 4)

    # ***
    # 函数名字：test_obstacle_inflation_forces_detour
    # 函数作用：验证固定障碍与车体包络阻断直路并触发绕行。
    # 参数意义与范围：self：当前NavigationTests实例。
    # ***
    def test_obstacle_inflation_forces_detour(self):
        data = sample_map()
        data['fixed_obstacles'] = [[1, 0, .1]]
        result = LanePlanner(data).plan()
        self.assertEqual(result['node_ids'], ['A', 'D', 'E', 'C'])
        self.assertEqual(result['length_m'], 4)

    # ***
    # 函数名字：test_no_path_never_returns_partial_route
    # 函数作用：验证后续任务点不可达时不输出局部路线冒充完整规划。
    # 参数意义与范围：self：当前NavigationTests实例。
    # ***
    def test_no_path_never_returns_partial_route(self):
        data = sample_map()
        data['goals'] = ['B', 'C']
        result = LanePlanner(data).plan([(2, 0, .2)])
        self.assertEqual(result['status'], 'no_path')
        self.assertEqual(result['path'], [])

    # ***
    # 函数名字：test_clearance_and_tangent_contact_are_blocked
    # 函数作用：验证道路净空不足和障碍切触均不能视为可通行。
    # 参数意义与范围：self：当前NavigationTests实例。
    # ***
    def test_clearance_and_tangent_contact_are_blocked(self):
        data = sample_map()
        for edge in data['edges']:
            edge['clearance_m'] = .1
        self.assertEqual(LanePlanner(data).plan()['status'], 'no_path')
        planner = LanePlanner(sample_map())
        self.assertFalse(planner.clear((0, 0), (1, 0), [(0.5, planner.radius, 0)]))

    # ***
    # 函数名字：test_mid_edge_attachment_and_off_route_rejection
    # 函数作用：验证移动中的位姿可接入道路中段，远离道路时拒绝规划。
    # 参数意义与范围：self：当前NavigationTests实例。
    # ***
    def test_mid_edge_attachment_and_off_route_rejection(self):
        planner = LanePlanner(sample_map())
        result = planner.plan(position=(.5, .01), snap_m=.05)
        self.assertEqual(result['status'], 'ready')
        self.assertEqual(result['path'][0], (.5, .01))
        self.assertAlmostEqual(result['length_m'], 1.51)
        self.assertEqual(planner.plan(position=(10, 10), snap_m=.05)['status'], 'off_route')

    # ***
    # 函数名字：test_invalid_map_is_rejected
    # 函数作用：验证未确认地图、非有限坐标及缺失固定障碍列表不能用于规划。
    # 参数意义与范围：self：当前NavigationTests实例。
    # ***
    def test_invalid_map_is_rejected(self):
        for field, value in [('confirmed', False), ('fixed_obstacles', None), ('vehicle_radius_m', None)]:
            data = sample_map()
            data[field] = value
            self.assertTrue(validate_map(data))
            with self.assertRaises(ValueError):
                LanePlanner(data)
        data = sample_map()
        data['nodes']['A'] = [float('nan'), 0]
        self.assertIn('NAVIGATION_NODES_INVALID', validate_map(data))

    # ***
    # 函数名字：test_fixed_plans_once_without_pose_or_radar
    # 函数作用：验证无雷达版仅计算一次且不读取定位或雷达数据。
    # 参数意义与范围：self：当前NavigationTests实例。
    # ***
    def test_fixed_plans_once_without_pose_or_radar(self):
        _, module, telemetry = self.fixture('fixed')
        with patch.object(module, '_pose', side_effect=AssertionError('must not read pose')), \
                patch.object(module.planner, 'plan', wraps=module.planner.plan) as plan:
            module.tick(None, 10, 100)
            first = telemetry.snapshot()['navigation']
            module.tick(None, 20, 110)
        plan.assert_called_once()
        self.assertEqual(first, telemetry.snapshot()['navigation'])
        self.assertEqual(first['node_ids'], ['A', 'B', 'C'])

    # ***
    # 函数名字：test_unconfigured_template_reports_missing_data
    # 函数作用：验证未知场地留空时网页状态明确且不制造路线。
    # 参数意义与范围：self：当前NavigationTests实例。
    # ***
    def test_unconfigured_template_reports_missing_data(self):
        data = json.loads((ROOT / 'projects/with_radar/config/route.json').read_text())
        _, module, telemetry = self.fixture('fixed', data)
        module.tick(None, 10, 100)
        state = telemetry.snapshot()['navigation']
        self.assertEqual(state['status'], 'configuration_required')
        self.assertTrue(state['readiness_errors'])
        self.assertEqual(state['path'], [])

    # ***
    # 函数名字：test_dynamic_observation_replans_and_is_not_silently_forgotten
    # 函数作用：验证新障碍触发绕行且下一圈缺少回波不会清除已知阻断。
    # 参数意义与范围：self：当前NavigationTests实例。
    # ***
    def test_dynamic_observation_replans_and_is_not_silently_forgotten(self):
        config, module, telemetry = self.fixture()
        module.tick(self.scan(), 10, 100)
        self.assertIn('B', telemetry.snapshot()['navigation']['node_ids'])
        self.pose(config, timestamp_unix=101)
        module.tick(self.scan(2, 11, (ScanPoint(0, 1, 10),)), 11, 101)
        self.assertEqual(telemetry.snapshot()['navigation']['node_ids'][-3:], ['D', 'E', 'C'])
        self.pose(config, timestamp_unix=102)
        module.tick(self.scan(3, 12), 12, 102)
        self.assertNotIn('B', telemetry.snapshot()['navigation']['node_ids'])

    # ***
    # 函数名字：test_stale_and_invalid_inputs_retract_existing_route
    # 函数作用：验证失效定位、错误坐标系、未来时间及雷达过期立即撤下旧路线。
    # 参数意义与范围：self：当前NavigationTests实例。
    # ***
    def test_stale_and_invalid_inputs_retract_existing_route(self):
        cases = [({'valid': False}, 'waiting_pose'), ({'frame': 'wrong'}, 'waiting_pose'),
                 ({'timestamp_unix': 200}, 'waiting_pose'), ({'timestamp_unix': 90}, 'waiting_pose'),
                 ({'x_m': float('nan')}, 'waiting_pose')]
        for changes, expected in cases:
            with self.subTest(changes=changes):
                config, module, telemetry = self.fixture()
                module.tick(self.scan(), 10, 100)
                self.assertEqual(telemetry.snapshot()['navigation']['status'], 'ready')
                self.pose(config, **changes)
                module.tick(self.scan(2, 11), 11, 101)
                state = telemetry.snapshot()['navigation']
                self.assertEqual(state['status'], expected)
                self.assertEqual(state['path'], [])
        _, module, telemetry = self.fixture()
        module.tick(self.scan(), 10, 100)
        module.tick(self.scan(), 20, 110)
        self.assertEqual(telemetry.snapshot()['navigation']['status'], 'waiting_radar')
        self.assertEqual(telemetry.snapshot()['navigation']['path'], [])

    # ***
    # 函数名字：test_pose_scan_time_mismatch_is_rejected
    # 函数作用：验证单独新鲜但与扫描不同步的定位不能用于坐标转换。
    # 参数意义与范围：self：当前NavigationTests实例。
    # ***
    def test_pose_scan_time_mismatch_is_rejected(self):
        config, module, telemetry = self.fixture()
        self.pose(config, timestamp_unix=99.7)
        module.tick(self.scan(), 10, 100)
        self.assertEqual(telemetry.snapshot()['navigation']['status'], 'waiting_synchronized_pose')

    # ***
    # 函数名字：test_extrinsic_and_pose_transform_preserves_c1_polarity
    # 函数作用：验证C1顺时针角、雷达安装平移和地图航向共同转换观测。
    # 参数意义与范围：self：当前NavigationTests实例。
    # ***
    def test_extrinsic_and_pose_transform_preserves_c1_polarity(self):
        config, module, _ = self.fixture()
        config.data['radar']['extrinsic'] = [1, 0, 0]
        self.pose(config, x_m=2, y_m=3, yaw_deg=90)
        module.tick(self.scan(points=(ScanPoint(90, 1, 10),)), 10, 100)
        obstacle = next(iter(module.observed.values()))
        self.assertAlmostEqual(obstacle[0], 3, delta=.05)
        self.assertAlmostEqual(obstacle[1], 4, delta=.05)

    # ***
    # 函数名字：test_observation_capacity_failure_withdraws_route
    # 函数作用：验证观测达到容量上限后保持故障而不丢弃旧障碍开放路线。
    # 参数意义与范围：self：当前NavigationTests实例。
    # ***
    def test_observation_capacity_failure_withdraws_route(self):
        config, module, telemetry = self.fixture()
        config.data['navigation']['maximum_observations'] = 1
        module.tick(self.scan(points=(ScanPoint(0, 10, 10), ScanPoint(90, 10, 10))), 10, 100)
        self.assertEqual(telemetry.snapshot()['navigation']['status'], 'observation_limit')
        self.pose(config, timestamp_unix=101)
        module.tick(self.scan(2, 11), 11, 101)
        self.assertEqual(telemetry.snapshot()['navigation']['path'], [])

    # ***
    # 函数名字：test_variant_configuration_paths_and_radar_boundary
    # 函数作用：验证两版共享资源解析正确且禁止配置模式与雷达开关冲突。
    # 参数意义与范围：self：当前NavigationTests实例。
    # ***
    def test_variant_configuration_paths_and_radar_boundary(self):
        for variant, enabled in [('without_radar', False), ('with_radar', True)]:
            config = load_config(ROOT / 'projects' / variant / 'config/board.json')
            self.assertEqual(config.data['radar']['enabled'], enabled)
            self.assertTrue(config.path(config.data['models']['object']['file']).is_file())
            self.assertEqual(config.path(config.data['target_focus']['native_library']), ROOT / 'native/lib/libcar26_focus.so')
            errors = config.validate()
            self.assertEqual(errors, ['RADAR_DEVICE_REQUIRED'] if enabled else [])
            if enabled:
                config.data['radar']['enabled'] = False
                self.assertIn('NAVIGATION_RADAR_MODE_CONFLICT', config.validate())
            else:
                self.assertFalse(config.data['navigation']['enabled'])
                self.assertEqual(config.data['mission']['entry'], 'qr')

    # ***
    # 函数名字：test_application_without_radar_never_starts_or_reads_radar
    # 函数作用：验证无雷达分项目在模拟主循环中不启用或读取雷达。
    # 参数意义与范围：self：当前NavigationTests实例。
    # ***
    def test_application_without_radar_never_starts_or_reads_radar(self):
        config = load_config(ROOT / 'projects/without_radar/config/board.json')
        with patch('car26.application.NavigationModule', side_effect=AssertionError('no planning')):
            app = Application(config)
        self.assertIsNone(app.navigation)
        self.assertEqual(app.mission.state, 'QR')
        app.communication, app.camera, app.radar, app.hmi = Mock(), Mock(), Mock(), Mock()
        app.communication.poll.return_value = []
        app.camera.result.return_value = (None, ())
        app.camera.drain_mode_events.return_value = ()
        app.camera.role = None
        # ***
        # 函数名字：stop_after_iteration
        # 函数作用：在模拟主循环完成一轮后请求正常退出。
        # 参数意义与范围：timeout：主循环等待秒数；非负数，本测试忽略实际等待。
        # ***
        def stop_after_iteration(timeout):
            app.stop_event.set()

        app.stop_event.wait = Mock(side_effect=stop_after_iteration)
        with patch('car26.application.signal.signal'):
            self.assertEqual(app.run(), 0)
        app.radar.start.assert_not_called()
        app.radar.result.assert_not_called()
        app.communication.send.assert_not_called()
