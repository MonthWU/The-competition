import math

from ..types import Obstacle


# ***
# 函数名字：point_xy
# 函数作用：将雷达极坐标采样转换为车体平面坐标。
# 参数意义与范围：
#   point：ScanPoint对象；角度[0, 360)度、距离为非负米数、质量[0, 63]。
# ***
def point_xy(point):
    angle = math.radians(point.angle_deg)
    # C1 positive angles are clockwise; body y is positive to the left.
    return point.distance_m * math.cos(angle), -point.distance_m * math.sin(angle)


# ***
# 函数名字：cluster_points
# 函数作用：按相邻距离聚合扫描点并丢弃过小点簇。
# 参数意义与范围：
#   points：按扫描顺序排列的ScanPoint序列；距离已由配置过滤，可为空。
#   gap：相邻点最大聚类距离；正有限米数，配置范围[0.001, 5]。
#   minimum_points：最小点簇采样数；大于或等于1的整数。
# ***
def cluster_points(points, gap, minimum_points):
    xy = [point_xy(p) for p in points]
    groups = []
    for position in xy:
        if not groups or math.dist(position, groups[-1][-1]) > gap:
            groups.append([])
        groups[-1].append(position)
    if len(groups) > 1 and math.dist(groups[0][0], groups[-1][-1]) <= gap:
        groups[0] = groups.pop() + groups[0]
    obstacles = []
    for group in groups:
        if len(group) < minimum_points:
            continue
        x = sum(p[0] for p in group) / len(group)
        y = sum(p[1] for p in group) / len(group)
        width = math.hypot(max(p[0] for p in group) - min(p[0] for p in group),
                           max(p[1] for p in group) - min(p[1] for p in group))
        obstacles.append(Obstacle(x, y, width, len(group)))
    return tuple(obstacles)


# ***
# 函数名字：transform_xy
# 函数作用：应用安装外参和扫描朝向转换平面坐标。
# 参数意义与范围：
#   x：水平像素或平面坐标；有限数值，单位和边界由所属图像或标定坐标系确定。
#   y：垂直像素或平面坐标；有限数值，单位和边界由所属图像或标定坐标系确定。
#   extrinsic：安装外参(x米, y米, yaw度)；三个有限数值，各在[-10000, 10000]内。
#   shot_angle_deg：当前扫描朝向；有限度数，实际使用已标定的朝向配置。
# ***
def transform_xy(x, y, extrinsic, shot_angle_deg):
    tx, ty, yaw = extrinsic
    radians = math.radians(yaw)
    bx, by = tx + x * math.cos(radians) - y * math.sin(radians), ty + x * math.sin(radians) + y * math.cos(radians)
    angle = math.radians(shot_angle_deg)
    return bx * math.cos(angle) - by * math.sin(angle), bx * math.sin(angle) + by * math.cos(angle)
