# Purpose: plan routes on an explicitly surveyed lane graph without opening devices.
import heapq
import math


# ***
# 函数名字：finite
# 函数作用：判断一个值是否为非布尔的有限实数。
# 参数意义与范围：value：任意对象；仅有限int或float返回True。
# ***
def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


# ***
# 函数名字：segment_distance
# 函数作用：计算平面点到闭线段的最短距离。
# 参数意义与范围：
#   point：有限二维坐标，单位米。
#   a：线段起点，有限二维米坐标。
#   b：线段终点，有限二维米坐标，可等于a。
# ***
def segment_distance(point, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    t = max(0, min(1, ((point[0]-a[0])*dx + (point[1]-a[1])*dy) / length2)) if length2 else 0
    return math.dist(point, (a[0]+t*dx, a[1]+t*dy))


# ***
# 函数名字：validate_map
# 函数作用：检查路线图、车体包络与固定障碍，拒绝未确认的实机数据。
# 参数意义与范围：data：JSON字典；节点与圆形障碍均使用同一地图坐标系的米单位。
# ***
def validate_map(data):
    errors = []
    try:
        if data['confirmed'] is not True:
            errors.append('NAVIGATION_MAP_NOT_CONFIRMED')
        if not isinstance(data['frame'], str) or not data['frame']:
            errors.append('NAVIGATION_FRAME_REQUIRED')
        if not all(finite(data[k]) and data[k] >= 0 for k in ('vehicle_radius_m', 'margin_m')) or data['vehicle_radius_m'] <= 0:
            errors.append('NAVIGATION_ENVELOPE_REQUIRED')
        nodes = data['nodes']
        if not isinstance(nodes, dict) or not 2 <= len(nodes) <= 256:
            errors.append('NAVIGATION_NODES_REQUIRED')
        elif any(not isinstance(k, str) or not k or k.startswith('@') or len(p) != 2 or
                 not all(finite(v) for v in p) for k, p in nodes.items()):
            errors.append('NAVIGATION_NODES_INVALID')
        edges = data['edges']
        if not isinstance(edges, list) or not 1 <= len(edges) <= 512:
            errors.append('NAVIGATION_EDGES_REQUIRED')
        else:
            for edge in edges:
                a, b = edge['nodes']
                if a not in nodes or b not in nodes or a == b or math.dist(nodes[a], nodes[b]) == 0:
                    errors.append('NAVIGATION_EDGE_INVALID')
                if not finite(edge['clearance_m']) or edge['clearance_m'] <= 0:
                    errors.append('NAVIGATION_CLEARANCE_REQUIRED')
        if data['start_node'] not in nodes or not isinstance(data['goals'], list) or not data['goals'] or any(g not in nodes for g in data['goals']):
            errors.append('NAVIGATION_MISSION_REQUIRED')
        obstacles = data['fixed_obstacles']
        if not isinstance(obstacles, list) or len(obstacles) > 4096:
            errors.append('NAVIGATION_FIXED_OBSTACLES_REQUIRED')
        elif any(len(o) != 3 or not all(finite(v) for v in o) or o[2] < 0 for o in obstacles):
            errors.append('NAVIGATION_OBSTACLE_INVALID')
    except (KeyError, TypeError, ValueError, AttributeError) as error:
        errors.append('NAVIGATION_MAP_INVALID:' + str(error))
    return errors


class LanePlanner:
    # ***
    # 函数名字：__init__
    # 函数作用：创建规划器并保存已确认的道路图与安全膨胀距离。
    # 参数意义与范围：
    #   self：当前LanePlanner实例。
    #   data：通过validate_map的地图字典，最多256节点与512边。
    # ***
    def __init__(self, data):
        errors = validate_map(data)
        if errors:
            raise ValueError(';'.join(errors))
        self.data = data
        self.radius = data['vehicle_radius_m'] + data['margin_m']

    # ***
    # 函数名字：clear
    # 函数作用：检验线段是否避开所有经过车体包络膨胀的圆形障碍。
    # 参数意义与范围：
    #   self：当前LanePlanner实例。
    #   a：起点二维米坐标。
    #   b：终点二维米坐标。
    #   obstacles：有限圆形障碍(x米,y米,半径米)序列，半径非负。
    # ***
    def clear(self, a, b, obstacles):
        return all(segment_distance(o[:2], a, b) > self.radius + o[2] for o in obstacles)

    # ***
    # 函数名字：plan
    # 函数作用：按任务点顺序求最短安全图路线，可将当前位姿投影接入道路。
    # 参数意义与范围：
    #   self：当前LanePlanner实例。
    #   obstacles：新增圆形障碍序列；默认空，与固定障碍叠加。
    #   position：当前地图二维米坐标；None使用配置起点。
    #   goals：剩余目标节点列表；None使用配置任务点，空列表表示已完成。
    #   snap_m：允许位姿偏离道路中心线的最大米数，非负有限数。
    # ***
    def plan(self, obstacles=(), position=None, goals=None, snap_m=0):
        obstacles = tuple(self.data['fixed_obstacles']) + tuple(obstacles)
        nodes = dict(self.data['nodes'])
        graph = {key: [] for key in nodes}
        goals = self.data['goals'] if goals is None else goals
        if not goals:
            return {'status': 'complete', 'path': [], 'node_ids': [], 'length_m': 0}
        for edge in self.data['edges']:
            a, b = edge['nodes']
            if edge['clearance_m'] >= self.radius and self.clear(nodes[a], nodes[b], obstacles):
                distance = math.dist(nodes[a], nodes[b])
                graph[a].append((b, distance))
                graph[b].append((a, distance))
        start = self.data['start_node']
        if position is not None:
            candidates = []
            for index, edge in enumerate(self.data['edges']):
                a, b = (nodes[k] for k in edge['nodes'])
                distance = segment_distance(position, a, b)
                if distance <= snap_m and edge['clearance_m'] >= self.radius + distance:
                    candidates.append((distance, index))
            if not candidates:
                return {'status': 'off_route', 'path': [], 'node_ids': [], 'length_m': 0}
            _, index = min(candidates)
            edge = self.data['edges'][index]
            a, b = (nodes[k] for k in edge['nodes'])
            dx, dy = b[0]-a[0], b[1]-a[1]
            t = max(0, min(1, ((position[0]-a[0])*dx+(position[1]-a[1])*dy)/(dx*dx+dy*dy)))
            projection = (a[0]+t*dx, a[1]+t*dy)
            start = '@pose'
            nodes[start], nodes['@lane'] = position, projection
            graph[start], graph['@lane'] = [], []
            if self.clear(position, projection, obstacles):
                graph[start].append(('@lane', math.dist(position, projection)))
                for endpoint in edge['nodes']:
                    if self.clear(projection, nodes[endpoint], obstacles):
                        graph['@lane'].append((endpoint, math.dist(projection, nodes[endpoint])))
        route, length = [start], 0.0
        for goal in goals:
            queue, best, previous = [(0.0, start)], {start: 0.0}, {}
            while queue:
                cost, node = heapq.heappop(queue)
                if cost != best[node]:
                    continue
                if node == goal:
                    break
                for neighbor, distance in graph[node]:
                    proposed = cost + distance
                    if proposed < best.get(neighbor, math.inf):
                        best[neighbor], previous[neighbor] = proposed, node
                        heapq.heappush(queue, (proposed, neighbor))
            if goal not in best or not self.clear(nodes[goal], nodes[goal], obstacles):
                return {'status': 'no_path', 'path': [], 'node_ids': [], 'length_m': 0}
            segment, node = [], goal
            while node != start:
                segment.append(node)
                node = previous[node]
            route.extend(reversed(segment))
            length += best[goal]
            start = goal
        return {'status': 'ready', 'path': [nodes[n] for n in route], 'node_ids': route, 'length_m': length}
