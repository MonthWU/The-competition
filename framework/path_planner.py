"""path_planner —— 场地避障路径规划（骨架）。

场地建模为节点图（启停区/二维码板/原料区/粗加工/暂存 + 行驶节点），
障碍节点不可入，BFS/A* 求最短避障路径，输出节点序列。
"""

GRAPH = {
    # TODO: 场地节点图（节点编号 ↔ 坐标），等待候选点/任务点确认后填入
}


class PathPlanner:
    def __init__(self, graph=None):
        raise NotImplementedError("TODO: 初始化图")

    def plan(self, start, goal, blocked=set()) -> list:
        """BFS 最短路径，返回节点序列；不可达返回 []。"""
        raise NotImplementedError("TODO: BFS/A* 实现")

    def to_commands(self, path) -> list:
        """节点序列 → 下位机指令序列（协议待确认）。"""
        raise NotImplementedError("TODO: 按下位机协议包装")
