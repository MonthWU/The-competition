"""serial_protocol —— 预扫描避障通信协议 v2（RDK 视觉 ↔ 单片机）。

用户定义（2026-09-08 v2 最终）：
1. 传输参数：ttyS1，115200，8N1，无硬件流控；ASCII 纯文本，帧头 '[' + 载荷 + ']'，无换行
2. 交互时序（最开始时序 + 三次拍摄，每次一个角度）：
     单片机: [num]（启停位置坐标，5×5 网格 row-major ID 0~24，如 [4]=右上角）
     RDK 记录启停位置 → 回 [ack]
     单片机: [shot]（云台转到该起点对应的 0° 基线后触发）→ RDK 拍 0° → 回 [ack]
     单片机: [shot]（云台 45°）→ RDK 拍 45° → 回 [ack]
     单片机: [shot]（云台 90°）→ RDK 拍 90° → 回 [ack]
     三次全部处理完 → 发地图帧（仅一次）
   注：0° 基线取决于启停位置（如从 4 出发，0° 对应朝向 4→9→14→19→24 列方向），
   角度→地图方向映射待人工确认后固化（见 mission_dispatcher.START_DIR_MAP）。
3. 地图帧内容（[] 内空格分隔的 ASCII 数字）：
     [<障碍数量 0~3> <障碍ID 十进制 0~24>... <校验>]
     校验 = count XOR 所有障碍ID，以两位十六进制 ASCII（大写）表示；
     障碍 ID = 5×5 网格 row-major 编号：id = row*5 + col（0~24）。
示例：2 个障碍位于 (0,1) 与 (0,3) → ids=[1,3] count=2 → 校验 2^1^3=0x00
      → 地图帧 "[2 1 3 00]"；无障碍 → "[0 00]"。
"""

SERIAL_DEV = "/dev/ttyS1"
SERIAL_BAUD = 115200

# 帧头尾 / 帧内分隔
FRAME_HEAD = "["
FRAME_TAIL = "]"
FIELD_SEP = " "        # 帧内字段分隔（空格；如改分隔只需动这里）

# 命令文本（[] 内载荷）
TRIGGER_TEXT = b"shot"  # 单片机 → 视觉：触发一次拍摄（一个角度）
ACK_TEXT = b"ack"       # 视觉 → 单片机：该次拍摄完成回传

# 拍摄角度序列：单片机分三次发 [shot]，视觉按序拍 0/45/90
SCAN_ANGLES = (0, 45, 90)

# 地图尺寸与障碍约束
MAP_ROWS = 5
MAP_COLS = 5
MAP_BITS = MAP_ROWS * MAP_COLS   # 25（ID 0~24）
MAX_OBSTACLES = 3                # 比赛限 3 个障碍

# 启停位置（用户 2026-09-08 确认：只有两个角；右上 (0,4)=4、右下 (4,4)=24）
START_SLOTS = (4, 24)

# 帧尾后是否追加行尾（默认不加；若下位机 readline 解析可设 "\n"）
TX_NEWLINE = b""


class ProtocolError(ValueError):
    """协议非法。"""


# ================= 基础帧 =================

def _wrap(payload: str) -> bytes:
    return FRAME_HEAD.encode() + payload.encode() + FRAME_TAIL.encode() + TX_NEWLINE


def build_trigger() -> bytes:
    """触发帧：单片机应发送的 [shot]（RDK 侧校验/匹配参考）。"""
    return _wrap(TRIGGER_TEXT.decode())


def build_ack() -> bytes:
    """拍摄完成回传帧：[ack]。"""
    return _wrap(ACK_TEXT.decode())


# ================= 起始握手（启停位置）=================

def build_start(num: int) -> bytes:
    """启停位置帧：[num]（num=5×5 row-major 格号 0~24；单片机发送参考）。"""
    if not isinstance(num, int) or not 0 <= num < MAP_BITS:
        raise ProtocolError(f"启停位置须为 0~24 整数: {num!r}")
    return _wrap(str(num))


def parse_start(frame) -> int:
    """解析 [num] 帧 → 启停位置 0~24；不是 num 帧抛 ProtocolError。"""
    s = frame.decode("ascii", errors="replace").strip(" \r\n\t[]")
    if not s.isdigit():
        raise ProtocolError(f"启停位置帧非数字: {frame!r}")
    v = int(s)
    if not 0 <= v < MAP_BITS:
        raise ProtocolError(f"启停位置超出 0~24: {v}")
    return v


# ================= 启停位置帧（时序最开始）=================

def build_start_frame(slot_id: int) -> bytes:
    """启停位置帧：[<ID 0~24>]（单片机 → 视觉）。"""
    if not 0 <= int(slot_id) < MAP_BITS:
        raise ProtocolError(f"启停位置 ID 超出 0~24: {slot_id}")
    return _wrap(str(int(slot_id)))


def parse_start_frame(frame) -> int:
    """解析启停位置帧 → ID（0~24）。载荷为纯十进制数。"""
    s = frame.decode("ascii", errors="replace").strip(" \r\n\t[]")
    if not s.isdigit():
        raise ProtocolError(f"启停位置帧非十进制: {frame!r}")
    v = int(s)
    if not 0 <= v < MAP_BITS:
        raise ProtocolError(f"启停位置 ID 超出 0~24: {v}")
    return v


def is_start_frame(frame) -> bool:
    """判断一段接收帧是否为启停位置帧（载荷为 0~24 纯数字）。"""
    try:
        parse_start_frame(frame)
        return True
    except ProtocolError:
        return False


# ================= 地图帧（障碍清单 + 校验）=================

def grid_id(r: int, c: int) -> int:
    """5×5 网格 (r,c) → 障碍 ID（row-major 0~24）。"""
    return r * MAP_COLS + c


def grid_cell(i: int) -> tuple:
    """障碍 ID → (r, c)。"""
    if not 0 <= i < MAP_BITS:
        raise ProtocolError(f"障碍 ID 超出 0~24: {i}")
    return divmod(i, MAP_COLS)


def checksum(count: int, ids) -> int:
    """校验 = count XOR 所有障碍 ID。"""
    x = int(count)
    for i in ids:
        x ^= int(i)
    return x


def build_obstacle_frame(ids) -> bytes:
    """把障碍 ID 列表编码成地图帧 [count id1 id2.. CHK]。

    ids：障碍 ID 十进制 0~24 的可迭代；长度须 0~3。
    """
    lst = [int(i) for i in ids]
    n = len(lst)
    if n > MAX_OBSTACLES:
        raise ProtocolError(f"障碍数量超限（> {MAX_OBSTACLES}）: {n}")
    for i in lst:
        if not 0 <= i < MAP_BITS:
            raise ProtocolError(f"障碍 ID 超出 0~24: {i}")
    parts = [str(n)] + [str(i) for i in lst]
    chk = checksum(n, lst)
    parts.append(f"{chk:02X}")  # 两位大写十六进制
    return _wrap(FIELD_SEP.join(parts))


def build_map_frame(ids) -> bytes:
    """兼容名：同 build_obstacle_frame（v2 协议地图帧=障碍清单）。"""
    return build_obstacle_frame(ids)


def parse_obstacle_frame(frame) -> dict:
    """解析地图帧 → {count, ids, checksum(hex), valid}。容忍前后空白。"""
    s = frame.decode("ascii", errors="replace").strip(" \r\n\t[]")
    fields = [f for f in s.replace(FIELD_SEP, " ").split(" ") if f != ""]
    if not fields:
        raise ProtocolError("地图帧为空")
    try:
        count = int(fields[0])
    except ValueError:
        raise ProtocolError(f"障碍数量非法: {fields[0]!r}")
    if not 0 <= count <= MAX_OBSTACLES:
        raise ProtocolError(f"障碍数量超出 0~{MAX_OBSTACLES}: {count}")
    if len(fields) != 2 + count:  # count + ids + chk
        raise ProtocolError(
            f"字段数不符: 期望 {2 + count}（count+{count} 个ID+校验），实际 {len(fields)}: {fields!r}")
    ids = []
    for t in fields[1:1 + count]:
        v = int(t)
        if not 0 <= v < MAP_BITS:
            raise ProtocolError(f"障碍 ID 超出 0~24: {v}")
        ids.append(v)
    chk_str = fields[-1]
    try:
        chk = int(chk_str, 16)
    except ValueError:
        raise ProtocolError(f"校验非十六进制: {chk_str!r}")
    expect = checksum(count, ids)
    return {
        "count": count,
        "ids": ids,
        "checksum": f"{chk:02X}",
        "valid": chk == expect,
    }


def parse_map_frame(frame) -> dict:
    """兼容名：同 parse_obstacle_frame。"""
    return parse_obstacle_frame(frame)


def parse_trigger(frame: bytes) -> bool:
    """判断一段接收字节是否触发命令 shot（容忍 [ ] 与空白）。"""
    s = frame.decode("ascii", errors="replace").strip(" \r\n\t[]").lower()
    return s == TRIGGER_TEXT.decode()


def parse_ack(frame: bytes):
    """解码回传帧，返回载荷 'ack'。"""
    s = frame.decode("ascii", errors="replace").strip(" \r\n\t[]").lower()
    if s != ACK_TEXT.decode():
        raise ProtocolError(f"ACK 帧非法: {frame!r}")
    return s


if __name__ == "__main__":
    print("触发帧:", build_trigger())
    print("ACK 帧 :", build_ack())
    print("启停帧(4=右上角):", build_start_frame(4), "解析:", parse_start_frame(b"[4]"))
    print("is_start_frame([24]):", is_start_frame(b"[24]"),
          " is_start_frame([shot]):", is_start_frame(b"[shot]"))
    # 有 2 个障碍 (0,1)(0,3) → ids 1,3
    f1 = build_obstacle_frame([1, 3])
    print("地图帧(2障):", f1)
    r1 = parse_obstacle_frame(f1)
    print("  解析:", r1, "校验正确:", r1["valid"])
    # 无障
    f0 = build_obstacle_frame([])
    print("地图帧(0障):", f0, "解析:", parse_obstacle_frame(f0))
    # 最大 3 障
    f3 = build_obstacle_frame([0, 12, 24])
    print("地图帧(3障):", f3, "解析 valid:", parse_obstacle_frame(f3)["valid"])
    # 防御
    try:
        build_obstacle_frame([1, 2, 3, 4])
    except ProtocolError as e:
        print("已拦截(>3):", e)
    try:
        build_obstacle_frame([25])
    except ProtocolError as e:
        print("已拦截(ID>24):", e)