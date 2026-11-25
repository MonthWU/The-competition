# Purpose: publish fixed or radar-updated theoretical routes with explicit configuration and pose gates.
import json
import math

from .planning import LanePlanner, finite, validate_map
from .radar.processing import transform_xy


# ***
# 函数名字：validate_navigation
# 函数作用：检查可选规划配置及模式边界，不打开传感器。
# 参数意义与范围：configuration：当前Configuration对象，路径以其配置目录为基准。
# ***
def validate_navigation(configuration):
    settings = configuration.data.get('navigation', {'enabled': False})
    errors = []
    try:
        if type(settings['enabled']) is not bool:
            return ['INVALID_NAVIGATION_ENABLED']
        if not settings['enabled']:
            return []
        mode = settings['mode']
        if mode not in ('fixed', 'dynamic'):
            errors.append('INVALID_NAVIGATION_MODE')
        if (mode == 'dynamic') != configuration.data['radar']['enabled']:
            errors.append('NAVIGATION_RADAR_MODE_CONFLICT')
        if configuration.data['mission']['entry'] == 'full':
            errors.append('NAVIGATION_LEGACY_PRESCAN_CONFLICT')
        for key, low, high in (('interval_s', .1, 30), ('pose_timeout_s', .1, 10),
                               ('sync_tolerance_s', .01, 2), ('snap_m', 0, 1),
                               ('goal_tolerance_m', .001, 1), ('point_radius_m', .001, .5),
                               ('memory_cell_m', .001, .5)):
            if not finite(settings[key]) or not low <= settings[key] <= high:
                errors.append('INVALID_NAVIGATION_PARAMETER:' + key)
        if type(settings['maximum_observations']) is not int or not 1 <= settings['maximum_observations'] <= 16384:
            errors.append('INVALID_NAVIGATION_MEMORY_LIMIT')
        if settings['pose_file'] is not None and (not isinstance(settings['pose_file'], str) or not settings['pose_file']):
            errors.append('INVALID_NAVIGATION_POSE_FILE')
        data = json.loads(configuration.path(settings['map_file']).read_text(encoding='utf-8'))
        if not isinstance(data, dict):
            errors.append('NAVIGATION_MAP_INVALID')
        elif data.get('confirmed') is not False:
            errors.extend(validate_map(data))
    except (OSError, KeyError, TypeError, ValueError, AttributeError) as error:
        errors.append('INVALID_NAVIGATION_CONFIG:' + str(error))
    return errors


class NavigationModule:
    # ***
    # 函数名字：__init__
    # 函数作用：加载当前分项目的规划配置；未确认地图只显示缺失项。
    # 参数意义与范围：
    #   self：当前NavigationModule实例。
    #   configuration：经过配置检查的Configuration对象。
    #   telemetry：当前应用的线程安全Telemetry对象。
    # ***
    def __init__(self, configuration, telemetry):
        self.configuration, self.telemetry = configuration, telemetry
        self.settings = configuration.data['navigation']
        self.data = json.loads(configuration.path(self.settings['map_file']).read_text(encoding='utf-8'))
        errors = validate_map(self.data)
        self.planner = None if errors else LanePlanner(self.data)
        self.last_update = None
        self.last_poll = None
        self.last_sequence = None
        self.observed = {}
        self.capacity_exceeded = False
        self.goal_index = 0
        self.revision = 0
        self.telemetry.update('navigation', mode=self.settings['mode'], frame=self.data.get('frame'),
                              graph=self.data['nodes'] if self.planner else {},
                              edges=self.data['edges'] if self.planner else [],
                              fixed_obstacles=self.data['fixed_obstacles'] if self.planner else [],
                              readiness_errors=errors, route_sent_to_mcu=False)
        self._publish('configuration_required' if errors else 'waiting', path=[], node_ids=[], length_m=0)

    # ***
    # 函数名字：_publish
    # 函数作用：发布当前路线状态并递增版本，空路线立即撤下旧规划。
    # 参数意义与范围：
    #   self：当前NavigationModule实例。
    #   status：当前状态字符串，如ready、no_path或waiting_pose。
    #   values：可序列化的规划结果或诊断字段；不包含运动命令。
    # ***
    def _publish(self, status, **values):
        self.revision += 1
        self.telemetry.update('navigation', status=status, revision=self.revision, **values)

    # ***
    # 函数名字：_pose
    # 函数作用：读取有界定位快照，核对有效标志、坐标系及新鲜度。
    # 参数意义与范围：
    #   self：当前NavigationModule实例。
    #   now_unix：当前Unix时间，有限秒数。
    # ***
    def _pose(self, now_unix):
        filename = self.settings['pose_file']
        if filename is None:
            return None
        try:
            with self.configuration.path(filename).open('rb') as source:
                raw = source.read(65537)
            if len(raw) > 65536:
                return None
            pose = json.loads(raw)
            if (pose['valid'] is not True or pose['frame'] != self.data['frame'] or
                not all(finite(pose[k]) for k in ('x_m', 'y_m', 'yaw_deg', 'timestamp_unix')) or
                not -3600 <= pose['yaw_deg'] <= 3600 or
                not 0 <= now_unix - pose['timestamp_unix'] <= self.settings['pose_timeout_s']):
                return None
            return pose
        except (OSError, ValueError, KeyError, TypeError):
            return None

    # ***
    # 函数名字：tick
    # 函数作用：固定模式只规划一次；动态模式在同步定位与新雷达帧到达后重算。
    # 参数意义与范围：
    #   self：当前NavigationModule实例。
    #   radar：RadarFrame或None；点为传感器局部坐标，时间戳来自主机单调时钟。
    #   now：当前主机单调时间，非负有限秒数。
    #   now_unix：与now同次取得的Unix时间，用于定位快照跨进程时序核对。
    # ***
    def tick(self, radar, now, now_unix):
        if self.planner is None:
            return
        if self.settings['mode'] == 'fixed':
            if self.last_update is None:
                self.last_update = now
                result = self.planner.plan()
                self._publish(**result, goal_index=0)
            return
        if self.last_poll is not None and now-self.last_poll < min(.1, self.settings['interval_s']):
            return
        self.last_poll = now
        error = None
        pose = self._pose(now_unix)
        settings = self.configuration.data['radar']
        if settings['extrinsic_calibrated'] is not True:
            error = 'waiting_calibration'
        elif radar is None or not 0 <= now - radar.timestamp <= settings['stream_timeout_s']:
            error = 'waiting_radar'
        elif pose is None:
            error = 'waiting_pose'
        elif abs(pose['timestamp_unix'] - (now_unix - (now-radar.timestamp))) > self.settings['sync_tolerance_s']:
            error = 'waiting_synchronized_pose'
        elif not radar.points:
            error = 'waiting_scan_points'
        if error:
            self._publish(error, path=[], node_ids=[], length_m=0)
            return
        if self.capacity_exceeded:
            self._publish('observation_limit', path=[], node_ids=[], length_m=0)
            return
        if radar.sequence == self.last_sequence or (self.last_update is not None and now-self.last_update < self.settings['interval_s']):
            return
        self.last_sequence, self.last_update = radar.sequence, now
        for point in radar.points:
            if not all(finite(v) for v in (point.angle_deg, point.distance_m)) or point.distance_m < 0:
                self._publish('invalid_scan', path=[], node_ids=[], length_m=0)
                return
            angle = math.radians(point.angle_deg)
            x, y = transform_xy(point.distance_m*math.cos(angle), -point.distance_m*math.sin(angle),
                                settings['extrinsic'], pose['yaw_deg'])
            x, y = x+pose['x_m'], y+pose['y_m']
            cell = self.settings['memory_cell_m']
            key = (math.floor(x/cell), math.floor(y/cell))
            # 保留已观测阻断，缺失回波不表示盲区已清空；用格心加半对角线覆盖整格。
            self.observed[key] = ((key[0]+.5)*cell, (key[1]+.5)*cell,
                                  self.settings['point_radius_m'] + cell/math.sqrt(2))
            if len(self.observed) > self.settings['maximum_observations']:
                self.capacity_exceeded = True
                self._publish('observation_limit', path=[], node_ids=[], length_m=0)
                return
        goals = self.data['goals']
        position = (pose['x_m'], pose['y_m'])
        # 每次观测最多推进一个任务点，重复节点代表不同任务阶段。
        if (self.goal_index < len(goals) and self.planner.clear(position, position, tuple(self.data['fixed_obstacles']) + tuple(self.observed.values()))
                and math.dist(position, self.data['nodes'][goals[self.goal_index]]) <= self.settings['goal_tolerance_m']):
            self.goal_index += 1
        result = self.planner.plan(tuple(self.observed.values()), position, goals[self.goal_index:], self.settings['snap_m'])
        self._publish(**result, goal_index=self.goal_index, observed_obstacles=list(self.observed.values()))
