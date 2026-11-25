import json
import math

from .communication.protocol import CANDIDATE_IDS
from .radar.processing import transform_xy


# ***
# 函数名字：inside_polygon
# 函数作用：判断给定点是否位于多边形内部或边界。
# 参数意义与范围：
#   x：水平像素或平面坐标；有限数值，单位和边界由所属图像或标定坐标系确定。
#   y：垂直像素或平面坐标；有限数值，单位和边界由所属图像或标定坐标系确定。
#   polygon：多边形顶点列表；至少三个有限(x, y)顶点，坐标系与待测点一致。
# ***
def inside_polygon(x, y, polygon):
    inside = False
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        cross = (x - a[0]) * (b[1] - a[1]) - (y - a[1]) * (b[0] - a[0])
        if abs(cross) < 1e-8 and min(a[0], b[0]) <= x <= max(a[0], b[0]) and min(a[1], b[1]) <= y <= max(a[1], b[1]):
            return True
        if (a[1] > y) != (b[1] > y) and x < (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0]:
            inside = not inside
    return inside


class CameraMapper:
    # ***
    # 函数名字：__init__
    # 函数作用：初始化当前类的依赖、配置和内部状态。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   configuration：项目Configuration对象；范围为已解析且满足对应模式约束的配置。
    # ***
    def __init__(self, configuration):
        self.settings = configuration.data["camera_mapping"]
        self.regions = json.loads(configuration.path(self.settings["roi_file"]).read_text(encoding="utf-8"))

    # ***
    # 函数名字：supports
    # 函数作用：判断指定起点是否具有本映射器所需的标定。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   start：起点编号；范围为4或24，并须有对应标定。
    # ***
    def supports(self, start):
        return start in self.settings["start_ids"]

    # ***
    # 函数名字：observations
    # 函数作用：依据已标定的场地关系生成本帧障碍候选及得分。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   frame：最新CameraFrame或RadarFrame；序号唯一且时间戳未超过配置的新鲜度门限。
    #   start：起点编号；范围为4或24，并须有对应标定。
    #   angle：预扫描角度；范围为0、45或90度。
    # ***
    def observations(self, frame, start, angle):
        if not self.supports(start):
            raise ValueError("CAMERA_MAP_START_NOT_CALIBRATED:" + str(start))
        if [frame.width, frame.height] != self.settings["calibration_size"]:
            raise ValueError("CAMERA_ROI_DIMENSIONS_MISMATCH")
        results = {}
        for detection in frame.detections:
            if detection.confidence < self.settings["score_threshold"]:
                continue
            x = detection.x1 + (detection.x2 - detection.x1) // 2
            y = detection.y1 + round((detection.y2 - detection.y1) * .7)
            hits = [region for region in self.regions[str(angle) + "deg"] if inside_polygon(x, y, region["pts"])]
            if len(hits) > 1:
                raise ValueError("ROI_OVERLAP_AT_DETECTION")
            for region in hits:
                index = int(region["label"]) - 1
                if not 0 <= index < len(CANDIDATE_IDS):
                    raise ValueError("INVALID_ROI_LABEL")
                target = CANDIDATE_IDS[index]
                results[target] = max(results.get(target, 0), detection.confidence)
        return results


class RadarMapper:
    # ***
    # 函数名字：__init__
    # 函数作用：初始化当前类的依赖、配置和内部状态。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   configuration：项目Configuration对象；范围为已解析且满足对应模式约束的配置。
    # ***
    def __init__(self, configuration):
        self.settings = configuration.data["radar_mapping"]
        self.extrinsic = configuration.data["radar"]["extrinsic"]

    # ***
    # 函数名字：supports
    # 函数作用：判断指定起点是否具有本映射器所需的标定。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   start：起点编号；范围为4或24，并须有对应标定。
    # ***
    def supports(self, start):
        return str(start) in self.settings["targets_by_start"] and str(start) in self.settings["heading_deg_by_start"]

    # ***
    # 函数名字：observations
    # 函数作用：依据已标定的场地关系生成本帧障碍候选及得分。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   frame：最新CameraFrame或RadarFrame；序号唯一且时间戳未超过配置的新鲜度门限。
    #   start：起点编号；范围为4或24，并须有对应标定。
    #   angle：预扫描角度；范围为0、45或90度。
    # ***
    def observations(self, frame, start, angle):
        if not self.supports(start):
            raise ValueError("RADAR_MAP_START_NOT_CALIBRATED:" + str(start))
        results = {}
        for obstacle in frame.obstacles:
            if not self.settings["minimum_width_m"] <= obstacle.width_m <= self.settings["maximum_width_m"]:
                continue
            heading = self.settings["heading_deg_by_start"][str(start)][(0, 45, 90).index(angle)]
            x, y = transform_xy(obstacle.x_m, obstacle.y_m, self.extrinsic, heading)
            candidates = [(math.dist((x, y), (t["x_m"], t["y_m"])), t["id"])
                          for t in self.settings["targets_by_start"][str(start)]]
            candidates.sort()
            if candidates and candidates[0][0] <= self.settings["match_radius_m"]:
                # Reject overlapping calibration gates rather than arbitrarily choosing an ID.
                if len(candidates) > 1 and candidates[1][0] <= self.settings["match_radius_m"]:
                    continue
                distance, target = candidates[0]
                results[target] = max(results.get(target, 0), 1 - distance / self.settings["match_radius_m"])
        return results
