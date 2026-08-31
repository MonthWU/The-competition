"""road_judge —— 障碍道路判定（骨架）。

输入：识别到的障碍位置（5×5 网格坐标或可换算为网格坐标）。
输出：命中的障碍候选点（x 位置）集合 → 交给 MapModel.place_obstacle() 标记通行矩阵。

判定策略（用户约定）：
  - 障碍只可能放置在 13 个候选点（x）上
  - 识别误差宽松（按场地 2400×2400 与 5×5 网格粒度折算）
  - 障碍坐标 → 就近候选点判定（距离 < 阈值即命中；保守策略：圈内候选点全部标记）
"""

import math

from map_model import OBSTACLE_CANDIDATES_13, MapModel


def nearest_candidate(r: float, c: float) -> tuple:
    """返回距 (r,c) 最近的候选点 ((cr, cc), dist)。"""
    best = None
    for (cr, cc) in OBSTACLE_CANDIDATES_13:
        d = math.hypot(cr - r, cc - c)
        if best is None or d < best[1]:
            best = ((cr, cc), d)
    return best


class RoadJudge:
    def __init__(self, hit_threshold: float = 1.5):
        """hit_threshold：障碍网格坐标与候选点的最大判定距离（5×5 网格单位）。
        误差宽松：1.5 格以内视为命中该候选点（可按现场实测调整）。
        """
        raise NotImplementedError("TODO: 误差参数按标定实测校准")

    def judge(self, obstacles: list) -> set:
        """输入障碍网格坐标列表 [(r,c), ...]，返回命中的障碍候选点集合 {(r,c)}。

        实现流程（TODO）：
          1. 对每个障碍坐标调 nearest_candidate()，dist <= hit_threshold 即命中
          2. 命中点并入结果集合（保守：误差圈内全部标记）
          3. 结果交给 MapModel.set_obstacles()
        """
        raise NotImplementedError("TODO: 障碍 → 就近候选点判定（13 候选点见 map_model）")


def demo():
    """骨架演示：手写障碍坐标 → 判定 → 更新通行矩阵。"""
    m = MapModel()
    # TODO: 正式实现后改为 RoadJudge().judge()
    obstacles = [(2.1, 2.0), (4.0, 1.3)]
    for r, c in obstacles:
        (cr, cc), d = nearest_candidate(r, c)
        print(f"障碍 ({r},{c}) → 最近候选 ({cr},{cc}) 距离 {d:.2f}")
        m.place_obstacle(cr, cc)
    print()
    print("标记后地图：")
    print(m)
    print("障碍候选点已占用:", m.obstacle_cells())


if __name__ == "__main__":
    demo()
