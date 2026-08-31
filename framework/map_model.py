"""map_model —— 田字形地图数字化模型（5×5 通行矩阵）。

用户定义（dotspng 地图数字化）：
    [1 x 1 x 1]
    [x 0 x 0 x]
    [1 x x x 1]
    [x 0 x 0 x]
    [1 x 1 x 1]
其中：
  - 1 = 固定节点（角点/交叉点，8 个），可通行
  - x = 障碍物会放置的点（13 个候选位置），平时可通行，放置障碍后标 0
  - 0 = 空白区域（4 个），不可通行
小车默认位置：左下角节点 (4, 0)。

通行矩阵语义：1=可通行（节点或未占用候选点），0=不可通行（空白或已放置障碍）。
"""

MAP_SIZE = 5

# 可通行矩阵（初始无障碍）：x 候选点平时=1，空白=0
INITIAL_GRID = [
    [1, 1, 1, 1, 1],
    [1, 0, 1, 0, 1],
    [1, 1, 1, 1, 1],
    [1, 0, 1, 0, 1],
    [1, 1, 1, 1, 1],
]

# 1 位置：固定节点（8 个，不可放障碍）
NODE_CELLS = [
    (0, 0), (0, 2), (0, 4),
    (2, 0), (2, 4),
    (4, 0), (4, 2), (4, 4),
]

# x 位置：障碍物会放置的点（13 个候选）
OBSTACLE_CANDIDATES_13 = [
    (0, 1), (0, 3),           # 顶线左右段中点
    (1, 0), (1, 2), (1, 4),   # 左/中/右竖线上段中点
    (2, 1), (2, 2), (2, 3),   # 中线左段/中心/右段
    (3, 0), (3, 2), (3, 4),   # 左/中/右竖线下段中点
    (4, 1), (4, 3),           # 底线左右段中点
]

# 空白（0）位置：不可通行
BLANK_CELLS = [
    (1, 1), (1, 3), (3, 1), (3, 3),
]

START_NODE = (4, 0)  # 小车默认在左下角节点


class MapModel:
    def __init__(self):
        self.grid = [row[:] for row in INITIAL_GRID]
        self._obstacles = set()  # 已放置障碍的候选点

    # ---- 查询 ----
    def is_traversable(self, r: int, c: int) -> bool:
        """(r,c) 是否可通行（1=可通行）。"""
        return 0 <= r < MAP_SIZE and 0 <= c < MAP_SIZE and self.grid[r][c] == 1

    def is_candidate(self, r: int, c: int) -> bool:
        """(r,c) 是否为障碍候选点（x）。"""
        return (r, c) in OBSTACLE_CANDIDATES_13

    # ---- 障碍管理 ----
    def place_obstacle(self, r: int, c: int) -> bool:
        """在候选点 (r,c) 放置障碍（仅 x 位置有效），返回是否成功。"""
        if self.is_candidate(r, c):
            self.grid[r][c] = 0
            self._obstacles.add((r, c))
            return True
        return False

    def clear_obstacle(self, r: int, c: int):
        """清除 (r,c) 障碍（恢复可通行）。"""
        if (r, c) in self._obstacles:
            self.grid[r][c] = 1
            self._obstacles.discard((r, c))

    def set_obstacles(self, cells: list):
        """批量放置障碍（cells 为 (r,c) 列表，自动忽略非候选点）。"""
        for r, c in cells:
            self.place_obstacle(r, c)

    def obstacle_cells(self) -> list:
        """返回已放置障碍的候选点列表。"""
        return sorted(self._obstacles)

    def reset(self):
        """恢复初始可通行矩阵。"""
        self.grid = [row[:] for row in INITIAL_GRID]
        self._obstacles = set()

    # ---- 输出 ----
    def to_list(self) -> list:
        """返回当前 5×5 通行矩阵（0/1）。"""
        return [row[:] for row in self.grid]

    def to_map_view(self, show_candidates=True) -> list:
        """返回用户视角矩阵：1=节点/0=空白，x=候选点（未占用）或已标记障碍。
        show_candidates=True 时，候选点显示 'x'，放置障碍后显示 '0'。
        """
        view = []
        for r in range(MAP_SIZE):
            row = []
            for c in range(MAP_SIZE):
                if self.grid[r][c] == 0:
                    row.append("0")
                elif show_candidates and self.is_candidate(r, c):
                    row.append("x")
                else:
                    row.append("1")
            view.append(row)
        return view

    def __str__(self):
        return "\n".join(" ".join(str(v) for v in row) for row in self.to_map_view())


def main():
    m = MapModel()
    print("用户视角地图（1=节点 x=障碍候选 0=空白/障碍）：")
    print(m)
    print()
    print(f"障碍候选点 x: {len(OBSTACLE_CANDIDATES_13)} 个")
    print(f"固定节点 1: {len(NODE_CELLS)} 个")
    print(f"小车默认位置: {START_NODE}")
    print()
    print("演示：在 (2,2) 中心 和 (4,1) 底线左中点放置障碍")
    m.place_obstacle(2, 2)
    m.place_obstacle(4, 1)
    print(m)
    print("已占用障碍:", m.obstacle_cells())


if __name__ == "__main__":
    main()
