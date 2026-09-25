"""mission_dispatcher —— 预扫描流程编排节点（协议 v3：启停坐标 + 三次 shot + 障碍清单帧）。

协议（2026-09-08 用户定义，含启停时序）：
  时序最前：单片机发 [num]（启停位置，5×5 row-major ID 0~24）→ RDK 记录 → 回 [ack]
  之后    ：单片机分三次发 [shot]（0°/45°/90° 各一次，0° 基线由启停位置决定）→
            视觉每收一次拍一个角度并回 [ack]
  三次完成后：视觉发一次地图帧 [count 障碍ID... 校验]（仅一次）

架构约束（用户确认）：RDK 只负责视觉与通信；云台转角、行走等控制由下位机完成。
0° 基线：由启停位置决定（如从 4=右上角出发，0° 沿 4→9→14→19→24 列方向）；
角度→地图方向映射 = 待人工确认后固化（START_DIR_MAP 草案）。
"""

import serial

from serial_protocol import (
    SERIAL_DEV,
    SERIAL_BAUD,
    SCAN_ANGLES,
    TRIGGER_TEXT,
    build_ack,
    build_obstacle_frame,
    build_start_frame,
    is_start_frame,
    parse_start_frame,
    grid_id,
    START_SLOTS,
)
from map_model import MapModel
import subprocess
import time

import rclpy
from prescan_dnn_node import PrescanDnnNode
from obstacle_detector import ObstacleDetector
from road_judge import RoadJudge

PRESCAN_LAUNCH = "/root/dev_ws/appli/framework/launch/prescan.launch.py"
SCAN_WAIT_SEC = 8.0  # 等一帧 block 检测的最大等待时间


class MissionDispatcher:
    # 0° 基线方向（用户 2026-09-08 确认；启停位仅右上 4 与右下 24）
    # 语义：起点 ID → 0° 摄像头应朝向的 5 格 ID 序列（网格方向）。
    #  - 从 4（右上角启停）出发：0° 朝 4→9→14→19→24（右列向下）
    #  - 从 24（右下角启停）出发：0° 朝左 24→23→22→21→20（底行向左）
    #    0~90° 扫描为顺时针旋转（用户说明）。
    START_DIR_MAP = {
        4:  [4, 9, 14, 19, 24],
        24: [24, 23, 22, 21, 20],
    }

    def __init__(self, map_model=None):
        self.ser = None
        self.map_model = map_model or MapModel()
        self._buf = bytearray()
        self.start_id = None       # 最近一次收到的启停位置 ID（0~24）
        # 视觉组件与预扫描子进程状态（2026-09-25 补齐）：
        # 此前 scan_angle() 引用了 self.obstacle_detector / self.road_judge /
        # self._launch_proc / self._rclpy_init，但 __init__ 从未定义它们 ——
        # 一旦进入 scan_angle 就抛 AttributeError。
        self.obstacle_detector = None
        self.road_judge = None
        self._launch_proc = None
        self._rclpy_init = False

    def _ensure_vision(self):
        """延迟构造视觉组件（缺文件时整个 dispatcher 仍可构造，只在真正扫描时报错）。"""
        if self.obstacle_detector is None:
            self.obstacle_detector = ObstacleDetector()
        if self.road_judge is None:
            self.road_judge = RoadJudge()

    # ================= 串口（预扫描阶段独占）=================

    def open_serial(self, dev: str = None, baud: int = None) -> bool:
        dev = dev or SERIAL_DEV
        baud = baud or SERIAL_BAUD
        try:
            self.ser = serial.Serial(dev, baud, timeout=0.1)
            self._buf.clear()
            return True
        except (serial.SerialException, OSError) as e:
            print(f"[mission] open_serial 失败 {dev}: {e}")
            self.ser = None
            return False

    def close_serial(self):
        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def _read_more(self) -> bool:
        """把串口已有字节读入缓冲，返回是否有新数据。"""
        try:
            n = self.ser.in_waiting
        except Exception:
            return False
        if n:
            self._buf.extend(self.ser.read(n))
            return True
        return False

    def wait_trigger(self, timeout_s: float = 10.0) -> bool:
        """等待一次触发命令 [shot]（缓冲内匹配载荷 "shot"）。超时返回 False。"""
        import time

        if self.ser is None:
            return False
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self._buf.find(TRIGGER_TEXT) >= 0:
                idx = self._buf.find(TRIGGER_TEXT)
                del self._buf[: idx + len(TRIGGER_TEXT)]
                return True
            self._read_more()
            time.sleep(0.02)
        return False

    def wait_start(self, timeout_s: float = 10.0) -> bool:
        """时序最前：等待启停位置帧 [num]（0~24）。成功则记录 self.start_id。"""
        import time

        if self.ser is None:
            return False
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            # 尝试从缓冲中提取完整 "[...]" 帧并判读
            s = bytes(self._buf).decode("ascii", errors="replace")
            while "[" in s and "]" in s:
                lo = s.find("[")
                hi = s.find("]")
                if hi < lo:  # 丢弃孤立的 ']'
                    del self._buf[: hi + 1]
                    s = bytes(self._buf).decode("ascii", errors="replace")
                    continue
                cand = self._buf[lo:hi + 1]
                try:
                    v = parse_start_frame(bytes(cand))
                except Exception:
                    # 非启停帧：只消费到这个 ']'，继续（可能是噪声/残留）
                    del self._buf[: hi + 1]
                    s = bytes(self._buf).decode("ascii", errors="replace")
                    continue
                # 命中：消费到 ']'，记录并返回
                del self._buf[: hi + 1]
                if v not in START_SLOTS:
                    print(f"[mission] 错误：启停位置 [{v}] 不在启停区 {START_SLOTS}，中止预扫描")
                    self.start_id = None
                    return False
                self.start_id = v
                print(f"[mission] 收到启停位置: [{v}] (start_id={v})")
                return True
            self._read_more()
            time.sleep(0.02)
        return False

    def send_ack(self) -> bool:
        """拍摄完成回传：[ack]。"""
        if self.ser is None:
            return False
        try:
            self.ser.write(build_ack())
            self.ser.flush()
            print(f"[mission] ACK: {build_ack()!r}")
            return True
        except serial.SerialException as e:
            print(f"[mission] send_ack 失败: {e}")
            return False

    def send_map(self) -> bool:
        """三次拍摄全部完成后发送地图帧：[count 障碍ID... 校验]（v2/v3）。"""
        if self.ser is None:
            return False
        ids = [grid_id(r, c) for (r, c) in self.map_model.obstacle_cells()]
        ids.sort()
        frame = build_obstacle_frame(ids)
        try:
            self.ser.write(frame)
            self.ser.flush()
            print(f"[mission] 地图帧已下发: {frame!r}")
            return True
        except serial.SerialException as e:
            print(f"[mission] send_map 失败: {e}")
            return False

    # ================= 视觉环节钩子（待障碍识别节点实现）=================

    def scan_angle(self, angle: float) -> bool:
        """实接（2026-09-23）

        流程：
          1. subprocess 拉起 framework/launch/prescan.launch.py（起 LRCP AR0234 + dnn_node_example）
          2. 在主进程 rclpy.init + PrescanDnnNode 订阅 /hobot_dnn_detection
          3. spin 等一帧 block 检测（最长 SCAN_WAIT_SEC 秒）
          4. obstacle_detector.pixel_to_grid → road_judge.judge_from_hits
          5. MapModel.set_obstacles

        注：单应性矩阵未标定 → pixel_to_grid 返回占位 (2,2)；
        实测闭环需先现场标定 calibrate(image_points, map_points)。
        """
        print(f"[mission] scan_angle({angle}) start_id={self.start_id} "
              f"正在拉起 prescan.launch ...")
        self._ensure_vision()
        try:
            self._launch_proc = subprocess.Popen(
                ["ros2", "launch", PRESCAN_LAUNCH],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            time.sleep(3.0)  # 等相机 + codec + dnn 起来
            if not rclpy.ok():
                rclpy.init()
                self._rclpy_init = True
            sub_node = PrescanDnnNode(name="prescan_sub_" + str(int(angle)))
            deadline = time.time() + SCAN_WAIT_SEC
            detected = []
            while time.time() < deadline:
                rclpy.spin_once(sub_node, timeout_sec=0.5)
                if sub_node.done_event.is_set():
                    detected = sub_node.last_blocks
                    break
            sub_node.destroy_node()
            if self._rclpy_init:
                rclpy.shutdown()
                self._rclpy_init = False
            if not detected:
                print(f"[mission] scan_angle({angle})：未检测到 block（可能无障碍或模型未命中）")
                return True  # 无障碍按协议仍 ack
            hits = {}
            for (cx_px, cy_px, conf) in detected:
                r, c = self.obstacle_detector.pixel_to_grid(cx_px, cy_px)
                gid = grid_id(r, c)
                if gid is not None:
                    hits[gid] = max(hits.get(gid, 0.0), conf)
            obstacles = self.road_judge.judge_from_hits(hits)
            if obstacles:
                self.map_model.set_obstacles(list(obstacles))
                print(f"[mission] scan_angle({angle})：命中候选 {sorted(obstacles)}")
            else:
                print(f"[mission] scan_angle({angle})：0 置信度命中")
            return True
        except Exception as e:
            print(f"[mission] scan_angle({angle}) 异常: {e}")
            return False
        finally:
            if self._launch_proc and self._launch_proc.poll() is None:
                self._launch_proc.terminate()
                try:
                    self._launch_proc.wait(timeout=2.0)
                except Exception:
                    self._launch_proc.kill()
                self._launch_proc = None

    # ================= 主流程 =================

    def run_prescan(self, dev: str = None, trigger_timeout: float = 10.0) -> bool:
        """执行预扫描 + 帧通信闭环（启停坐标 → 三次 shot → 地图帧）。

        时序：等 [num]（启停位置）→ 回 [ack] → 分三次等 [shot]，每次拍对应角度
        (0/45/90) 并回 [ack] → 三次完成后发地图帧 [count 障碍ID... 校验]。
        """
        if not self.open_serial(dev):
            return False
        try:
            # [0] 启停位置
            if not self.wait_start(trigger_timeout):
                print("[mission] 等待启停位置 [num] 超时")
                return False
            if not self.send_ack():
                return False
            # [1-3] 三角度
            for angle in SCAN_ANGLES:
                if not self.wait_trigger(trigger_timeout):
                    print(f"[mission] 第 {angle}° 等待 [shot] 超时")
                    return False
                if not self.scan_angle(angle):
                    print(f"[mission] 第 {angle}° 拍摄/识别失败（视觉钩子未实现/未命中）")
                    return False
                if not self.send_ack():
                    return False
            return self.send_map()
        finally:
            self.close_serial()


def main():
    disp = MissionDispatcher()
    ok = disp.run_prescan(trigger_timeout=5.0)
    print(f"\n预扫描通信结果: {'成功' if ok else '失败（请看上方日志）'}")
    print("注：无下位机环境请用 run_prescan(dev=<虚拟串口>) 联调。")


if __name__ == "__main__":
    main()