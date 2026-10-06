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
import os
import signal
from collections import Counter, defaultdict

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
from obstacle_locator import ObstacleLocator

PRESCAN_LAUNCH = "/root/dev_ws/appli/framework/launch/prescan.launch.py"
SCAN_WAIT_SEC = 8.0  # 每个角度等待相机与 DNN 输出的最长时间
# 2026-09-27：每次收到下位机 [shot] 时，把当时相机画面留档到此目录（供后期检查）
SCAN_IMG_DIR = "/root/dev_ws/appli/_tmp_scan_imgs"
SCHOOL_PROFILE = os.environ.get(
    "APPLI_SCHOOL_PROFILE", "/root/dev_ws/appli/framework/school_profile.json"
)


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
        self._launch_proc = None
        self._rclpy_init = False
        self.locator = None
        self._votes = Counter()
        self._confidence = defaultdict(float)

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
            frame = b"[" + TRIGGER_TEXT + b"]"
            if self._buf.find(frame) >= 0:
                idx = self._buf.find(frame)
                del self._buf[: idx + len(frame)]
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
        if self.locator is None or len(ids) != self.locator.expected_count:
            print(f"[mission] OBSTACLE_COUNT_INVALID: detected={len(ids)} expected=1")
            return False
        frame = build_obstacle_frame(ids)
        try:
            self.ser.write(frame)
            self.ser.flush()
            print(f"[mission] 地图帧已下发: {frame!r}")
            return True
        except serial.SerialException as e:
            print(f"[mission] send_map 失败: {e}")
            return False

    # ================= 障碍视觉扫描 =================

    def scan_angle(self, angle: float) -> bool:
        """Collect several inference frames and map detections to road cells."""
        print(f"[mission] SCAN_ANGLE_{angle}: start_id={self.start_id}")
        os.makedirs(SCAN_IMG_DIR, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        log_path = os.path.join(SCAN_IMG_DIR, "prescan_%03d_%s.log" % (angle, stamp))
        image_path = os.path.join(SCAN_IMG_DIR, "scan_%03d_%s.jpg" % (angle, stamp))
        sub_node = None
        log_file = None
        try:
            log_file = open(log_path, "ab")
            self._launch_proc = subprocess.Popen(
                ["ros2", "launch", PRESCAN_LAUNCH],
                stdout=log_file,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            time.sleep(3.0)
            if self._launch_proc.poll() is not None:
                print(f"[mission] PRESCAN_LAUNCH_FAILED: {log_path}")
                return False
            if not rclpy.ok():
                rclpy.init()
                self._rclpy_init = True
            sub_node = PrescanDnnNode(name="prescan_sub_" + str(int(angle)))
            deadline = time.monotonic() + SCAN_WAIT_SEC
            processed_frames = 0
            positive_frames = 0
            while time.monotonic() < deadline:
                rclpy.spin_once(sub_node, timeout_sec=0.3)
                if sub_node.frame_seen == processed_frames:
                    continue
                processed_frames = sub_node.frame_seen
                best_in_frame = {}
                for x, y, confidence in sub_node.last_blocks:
                    if confidence < 0.5:
                        continue
                    cell = self.locator.locate(angle, x, y)
                    if cell is not None:
                        best_in_frame[cell] = max(best_in_frame.get(cell, 0.0), confidence)
                for cell, confidence in best_in_frame.items():
                    self._votes[cell] += 1
                    self._confidence[cell] += confidence
                if best_in_frame:
                    positive_frames += 1
                if (positive_frames >= self.locator.minimum_frames
                        and sub_node.img_seen > 0):
                    break

            saved = sub_node.save_last_image(image_path)
            print(f"[mission] SCAN_RESULT angle={angle} frames={processed_frames} "
                  f"positive={positive_frames} image_saved={saved} log={log_path}")
            if processed_frames == 0 or sub_node.img_seen == 0:
                print("[mission] SCAN_NO_CAMERA_OR_INFERENCE_FRAME")
                return False
            return True
        except Exception as e:
            print(f"[mission] SCAN_ANGLE_FAILED angle={angle}: {e}; log={log_path}")
            return False
        finally:
            if sub_node is not None:
                sub_node.destroy_node()
            if self._rclpy_init:
                rclpy.shutdown()
                self._rclpy_init = False
            if self._launch_proc is not None:
                try:
                    os.killpg(self._launch_proc.pid, signal.SIGTERM)
                    self._launch_proc.wait(timeout=3.0)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    try:
                        os.killpg(self._launch_proc.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                except Exception as e:
                    print(f"[mission] PRESCAN_CLEANUP_WARNING: {e}")
                self._launch_proc = None
            if log_file is not None:
                log_file.close()

    def select_school_obstacle(self) -> bool:
        """Accept exactly one well-supported cell before sending a map frame."""
        if self.locator.fixed_cell is not None:
            self.map_model.reset()
            self.map_model.set_obstacles([self.locator.fixed_cell])
            print(f"[mission] SCHOOL_OBSTACLE_SELECTED: id={grid_id(*self.locator.fixed_cell)} "
                  "source=fixed_obstacle_id")
            return True
        if not self._votes:
            print("[mission] SCHOOL_OBSTACLE_NOT_DETECTED")
            return False
        ranked = sorted(self._votes, key=lambda cell: (
            self._votes[cell], self._confidence[cell]), reverse=True)
        winner = ranked[0]
        votes = self._votes[winner]
        share = votes / sum(self._votes.values())
        if votes < self.locator.minimum_frames or share < self.locator.minimum_share:
            print(f"[mission] SCHOOL_OBSTACLE_AMBIGUOUS: votes={dict(self._votes)} "
                  f"winner_share={share:.3f}")
            return False
        self.map_model.reset()
        self.map_model.set_obstacles([winner])
        print(f"[mission] SCHOOL_OBSTACLE_SELECTED: id={grid_id(*winner)} "
              f"votes={votes} share={share:.3f} source={self.locator.source}")
        return True

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
            try:
                self.locator = ObstacleLocator(SCHOOL_PROFILE, self.start_id)
            except (OSError, ValueError, KeyError) as e:
                print(f"[mission] CALIBRATION_REQUIRED: {e}")
                return False
            self._votes.clear()
            self._confidence.clear()
            if not self.send_ack():
                return False
            # [1-3] 三角度
            for angle in SCAN_ANGLES:
                if not self.wait_trigger(trigger_timeout):
                    print(f"[mission] 第 {angle}° 等待 [shot] 超时")
                    return False
                if not self.scan_angle(angle):
                    print(f"[mission] 第 {angle}° 拍摄/识别失败")
                    return False
                if not self.send_ack():
                    return False
            if not self.select_school_obstacle():
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
