"""map_model —— 田字形地图数字化模型（5×5 通行矩阵）。

地图来源：dotspng.png（13 节点图）。黑色线 = 小车可走路径。
数字化约定：
  - 5×5 矩阵，行列索引 0-4；行 0/2/4 = 三条横线，列 0/2/4 = 三条竖线
  - 1 = 可通行（节点 或 线上位置）；0 = 不可通行（空白区域 或 障碍）
  - 13 个节点对应 5×5 中部分格子（见 NODES_13）
  - 小车默认位置：左下角节点 (4, 0)
"""

MAP_SIZE = 5

# 初始通行矩阵（无障碍时；空白区域标 0 不可通行）
INITIAL_GRID = [
    [1, 1, 1, 1, 1],
    [1, 0, 1, 0, 1],
    [1, 1, 1, 1, 1],
    [1, 0, 1, 0, 1],
    [1, 1, 1, 1, 1],
]

# 13 节点 → 5×5 坐标（P 编号与 road_judge.py 一致）
NODES_13 = {
    "P01": (0, 0),  # 左上角
    "P10": (0, 1),  # 顶左中点
    "P02": (0, 2),  # 顶中
    "P11": (0, 3),  # 顶右中点
    "P03": (0, 4),  # 右上角
    "P04": (2, 0),  # 左中
    "P05": (2, 2),  # 中心
    "P06": (2, 4),  # 右中
    "P07": (4, 0),  # 左下角（小车默认位置）
    "P12": (4, 1),  # 底左中点
    "P08": (4, 2),  # 底中
    "P13": (4, 3),  # 底右中点
    "P09": (4, 4),  # 右下角
}

START_NODE = (4, 0)  # 小车默认在左下角节点


class MapModel:
    def __init__(self):
        self.grid = [row[:] for row in INITIAL_GRID]

    def is_traversable(self, r: int, c: int) -> bool:
        """判断 (r,c) 是否可通行（1=可通行）。"""
        return 0 <= r < MAP_SIZE and 0 <= c < MAP_SIZE and self.grid[r][c] == 1

    def block(self, r: int, c: int):
        """将 (r,c) 标记为障碍（0）。"""
        if 0 <= r < MAP_SIZE and 0 <= c < MAP_SIZE:
            self.grid[r][c] = 0

    def clear(self, r: int, c: int):
        """清除 (r,c) 障碍（恢复 1，仅对路径/节点位置有效）。"""
        if 0 <= r < MAP_SIZE and 0 <= c < MAP_SIZE:
            self.grid[r][c] = 1

    def set_obstacle_nodes(self, node_ids: list):
        """按节点编号（P01-P13）批量标记障碍。"""
        for nid in node_ids:
            if nid in NODES_13:
                r, c = NODES_13[nid]
                self.block(r, c)

    def set_obstacle_cells(self, cells: list):
        """按 (r,c) 列表批量标记障碍（含线上位置）。"""
        for r, c in cells:
            self.block(r, c)

    def obstacle_nodes(self) -> list:
        """返回当前被标记为障碍的节点编号列表。"""
        return [nid for nid, (r, c) in NODES_13.items() if self.grid[r][c] == 0]

    def reset(self):
        """恢复初始通行矩阵。"""
        self.grid = [row[:] for row in INITIAL_GRID]

    def to_list(self) -> list:
        """返回当前 5×5 通行矩阵（0/1 列表）。"""
        return [row[:] for row in self.grid]

    def __str__(self):
        lines = []
        for r in range(MAP_SIZE):
            row = ""
            for c in range(MAP_SIZE):
                mark = "1" if self.grid[r][c] == 1 else "0"
                row += " " + mark
            lines.append(row.strip())
        return "\n".join(lines)


def main():
    m = MapModel()
    print("初始地图（1=可通行 0=不可通行，小车默认在左下角 (4,0)）：")
    print(m)
    print()
    print("13 节点 → 5×5 坐标：")
    for nid, (r, c) in sorted(NODES_13.items(), key=lambda kv: (kv[1][0], kv[1][1])):
        print(f"  {nid}: ({r},{c})")


if __name__ == "__main__":
    main()
