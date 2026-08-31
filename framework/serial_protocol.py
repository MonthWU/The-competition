"""serial_protocol —— 预扫描阶段串口协议定义（骨架，均暂定待确认）。

方向（相对原任务方向扩展）：
  - 读：下位机 → 上位机（触发预扫描指令）
  - 写：每角度成功回传 + 5×5 地图矩阵

帧格式统一沿用项目约定：0xFF(头) + TYPE(1B) + [数据] + 0xFE(尾)。
"""

# ---- 常量（暂定，与下位机确认后修改）----
SERIAL_DEV = "/dev/ttyS1"
SERIAL_BAUD = 115200

# 触发指令：下位机 → 上位机，请求开始预扫描（暂定 0x53 'S'）
TRIGGER_SCAN = b"\x53"  # 暂定：0x53 = 'S'（SCAN）

# 回传/地图类型字节（暂定）
TYPE_ACK = 0x50       # 成功回传（每角度）
TYPE_MAP = 0x40       # 地图矩阵传输

# 扫描角度
SCAN_ANGLES = (0, 45, 90)

# 地图尺寸
MAP_ROWS = 5
MAP_COLS = 5


def build_ack(angle: float) -> bytes:
    """每角度扫描成功的回传帧：FF 50 <angle> FE（angle 以 0/45/90 表示，暂定）。"""
    raise NotImplementedError("TODO: 按与下位机约定格式构造（角度编码方式待确认）")


def build_map_frame(grid: list) -> bytes:
    """把 5×5 纯 1/0 矩阵编码成串口帧：FF 40 <row0..row4 每行5bit> FE。

    编码草案（暂定）：每行 1 字节，低 5 位 = 该行 5 格（1=可通行 0=障碍/空白），
    共 5 字节 + 头尾 = 7 字节帧。下位机按行重建 5×5 地图。
    """
    raise NotImplementedError("TODO: 按协议编码（行->低5bit，0x01<<c）")


def parse_map_frame(frame: bytes) -> list:
    """解码下位机回传的地图帧（调试/联调用）。"""
    raise NotImplementedError("TODO: 解码 5 字节为 5×5 矩阵")


if __name__ == "__main__":
    # 演示帧格式
    demo_grid = [
        [1, 1, 1, 1, 1],
        [1, 0, 1, 0, 1],
        [1, 1, 1, 1, 1],
        [1, 0, 1, 0, 1],
        [1, 1, 1, 1, 1],
    ]
    print("地图帧编码草案（每行低5位）：")
    for r, row in enumerate(demo_grid):
        bits = "".join(str(v) for v in row)
        val = sum(v << c for c, v in enumerate(row))
        print(f"  row{r}: {bits} -> 0x{val:02X} ({val})")
    print("完整帧: FF 40 " + " ".join(f"{sum(v<<c for c,v in enumerate(row)):02X}" for row in demo_grid) + " FE")
