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

        实现流程：
          1. 对每个障碍坐标调 nearest_candidate()，dist <= hit_threshold 即命中
          2. 命中点并入结果集合（保守：误差圈内全部标记）
          3. 结果交给 MapModel.set_obstacles()
        注：实际生产链路（2026-09-23 起）改由用户在标注器标注后喂入
        `judge_from_hits(hits)` —— 本方法是参考/降级路径。
        """
        hits: set = set()
        for r, c in obstacles:
            (cr, cc), d = nearest_candidate(r, c)
            if d <= self.hit_threshold:
                hits.add((cr, cc))
        return hits

    def judge_from_hits(self, hits_from_model: dict) -> set:
        """接收模型/标注器给出的候选点命中表，返回最终障碍候选点集合。

        参数：
          hits_from_model: {grid_id (int 0~24): confidence (float)}
            —— 由 prescan_dnn_node 或用户在 map-quad-annotator 标注器中给出
            —— 注意：grid_id 必须在 OBSTACLE_CANDIDATES_13 对应的 13 个 id 中

        过滤规则（2026-09-23 用户定义）：
          1. 候选点有效性校验：grid_id 必须落在 OBSTACLE_CANDIDATES_13
          2. 置信度 >= hit_threshold（默认 0.5）
          3. 返回 {(r, c), ...} 供 MapModel.set_obstacles()
        """
        result: set = set()
        for grid_id, conf in hits_from_model.items():
            r, c = divmod(grid_id, 5)
            if (r, c) not in OBSTACLE_CANDIDATES_13:
                continue
            if conf < self.hit_threshold:
                continue
            result.add((r, c))
        return result


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


def demo_from_hits():
    """演示：通过标注器/模型标注的命中表输入。"""
    rj = RoadJudge(hit_threshold=0.5)
    hits = {1: 0.9, 3: 0.7, 13: 0.2, 6: 0.8}  # 13 置信度太低应被滤掉
    result = rj.judge_from_hits(hits)
    print(f"标注命中 {hits} → 最终障碍候选点: {sorted(result)}")

    m = MapModel()
    m.set_obstacles(list(result))
    print(m)


if __name__ == "__main__":
    demo()
