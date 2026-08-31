"""road_judge —— 障碍道路判定（骨架）。

根据识别结果（地图坐标 + 误差带）判定命中哪些候选点（17 → 简化 13）。
误差圈半径随距离增大（斜视远端误差大），保守策略：圈内候选点全部标记为障碍。
"""

CANDIDATE_POINTS_17 = [
    # TODO: 现场确认 17 个候选点坐标（场地 2400×2400 坐标系，单位 mm）
]

VALID_POINTS_13 = [
    # TODO: 按运行逻辑简化为 13 个判定点（合并/剔除规则现场确认）
]


class RoadJudge:
    def __init__(self, error_near=50, error_far=200):
        raise NotImplementedError("TODO: 误差参数按标定实测校准")

    def error_radius(self, distance) -> float:
        """按距离返回误差圈半径。"""
        raise NotImplementedError("TODO: 近小远大（线性/分段）")

    def judge(self, obstacles) -> set:
        """输入障碍地图坐标列表，输出被标记的障碍节点集合。"""
        raise NotImplementedError("TODO: 障碍 → 误差圈 → 命中候选点 → 输出集合")
