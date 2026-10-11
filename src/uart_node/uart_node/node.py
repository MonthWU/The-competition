"""UART Sender Node — RDK_25_E_004

移植自 RDK_25_E_001 的 control/UartSender.py + device/Uart.py + utils/Packet.py，
以 ROS2 + 独立高精度发送线程运行。

架构：
  - 独立后台线程实现 ~200Hz 精确发送（与 E001 UartSender._loop 一致）
  - 使用 SCHED_FIFO priority 30 确保实时性
  - 订阅 /imu/data 获取 angle_z / speed_z
  - 24 字节标准帧：AA 14 + 20B payload + checksum + FF

话题：
  - sub /imu/data (std_msgs/String): 来自 imu_node 的 angle_z,speed_z
  - pub /uart/tx (std_msgs/String): 每帧调试信息（可选）

协议（与 E001 完全一致）：
  payload = struct.pack("<iiiiI", ax, ay, angle_z, speed_z, timestamp_ms)  # 20B
  frame = AA + 0x14 + payload + sum(payload)&0xFF + FF  # 24B
"""
import os
import threading
import time
import struct
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ──────────────────────────────────────────────
# 24B 帧协议 — 移植自 E001 utils/Packet.py
# ──────────────────────────────────────────────

INT32_MIN = -2147483648
INT32_MAX = 2147483647
UINT32_MAX = 0xFFFFFFFF
FRAME_HEAD = 0xAA
FRAME_TAIL = 0xFF
PAYLOAD_LEN = 20


def clamp_i32(v):
    return max(INT32_MIN, min(INT32_MAX, int(v)))


def hexdump(data):
    return " ".join(f"{int(b) & 0xFF:02X}" for b in data)


def build_payload(ax, ay, angle_z, speed_z, timestamp_ms):
    """20 字节 payload：4×int32 + 1×uint32，小端。"""
    return struct.pack(
        "<iiiiI",
        clamp_i32(ax),
        clamp_i32(ay),
        clamp_i32(angle_z),
        clamp_i32(speed_z),
        int(timestamp_ms) & UINT32_MAX,
    )


def make_packet(payload: bytes) -> bytes:
    """AA + 0x14 + 20B payload + checksum + FF → 24 字节。"""
    if len(payload) != PAYLOAD_LEN:
        raise ValueError(f"payload must be {PAYLOAD_LEN}B, got {len(payload)}B")
    checksum = sum(payload) & 0xFF
    return bytes([FRAME_HEAD, PAYLOAD_LEN]) + payload + bytes([checksum, FRAME_TAIL])


def build_e_packet(ax, ay, angle_z, speed_z, timestamp_ms) -> bytes:
    """一键构建完整 24B 帧 — 兼容 E001 build_e_packet_i32x5。"""
    return make_packet(build_payload(ax, ay, angle_z, speed_z, timestamp_ms))


# ──────────────────────────────────────────────
# UART 封装 — 移植自 E001 device/Uart.py（精简版）
# ──────────────────────────────────────────────
class Uart:
    def __init__(self, port, baudrate=115200, timeout=0.01,
                 write_timeout=0.05, flush_after_write=False,
                 debug=False):
        import serial
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.write_timeout = write_timeout
        self.flush_after_write = flush_after_write
        self.debug = debug

        self.ser = None
        self.write_count = 0
        self.write_fail_count = 0
        self.short_write_count = 0
        self.last_write_ms = 0.0

        self._open()

    def _open(self):
        import serial
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=8,
                parity="N",
                stopbits=1,
                timeout=self.timeout,
                write_timeout=self.write_timeout,
            )
            print(f"[UART] opened {self.port} @ {self.baudrate}")
        except Exception as e:
            self.ser = None
            print(f"[UART] open failed: {self.port}, {e}")

    def is_open(self):
        return self.ser is not None and self.ser.is_open

    def write(self, data: bytes) -> bool:
        if not self.is_open():
            self.write_fail_count += 1
            return False

        t0 = time.perf_counter()
        try:
            n = self.ser.write(data)
            elapsed = (time.perf_counter() - t0) * 1000.0
            self.last_write_ms = elapsed
            self.write_count += 1

            if n != len(data):
                self.short_write_count += 1
                self.write_fail_count += 1
                return False

            if self.flush_after_write:
                self.ser.flush()

            return True
        except Exception:
            self.last_write_ms = (time.perf_counter() - t0) * 1000.0
            self.write_fail_count += 1
            return False

    def close(self):
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass


# ──────────────────────────────────────────────
# UART 精确发送线程 — 移植自 E001 UartSender._loop
# ──────────────────────────────────────────────
class UartSenderWorker:
    """精确 200Hz 发送线程，与 E001 UartSender._loop 一致。"""

    def __init__(self, uart: Uart, send_hz=200, debug_every=200,
                 rt_priority=30, pos_scale=100, angle_scale=100, speed_scale=100):
        self.uart = uart
        self.send_hz = send_hz
        self.send_period = 1.0 / max(1.0, send_hz)
        self.debug_every = debug_every
        self.rt_priority = rt_priority
        self.pos_scale = pos_scale
        self.angle_scale = angle_scale
        self.speed_scale = speed_scale

        # 共享状态（由 ROS2 回调更新）
        self.lock = threading.Lock()
        self.ax = 0.0
        self.ay = 0.0
        self.angle_z = 0.0
        self.speed_z = 0.0
        self.imu_valid = False

        # 统计
        self.send_count = 0
        self.write_fail_count = 0
        self.deadline_late_count = 0
        self.last_tx_hz = 0.0

        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def update(self, ax, ay, angle_z, speed_z):
        """由 ROS2 回调线程调用，更新发送数据。"""
        with self.lock:
            self.ax = float(ax)
            self.ay = float(ay)
            self.angle_z = float(angle_z)
            self.speed_z = float(speed_z)

    def stop(self):
        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)

    def _to_i32(self, value, scale):
        return clamp_i32(round(float(value) * float(scale)))

    def _timestamp_ms(self):
        return int(time.monotonic() * 1000) & UINT32_MAX

    def _loop(self):
        # 设置实时调度优先级（与 E001 一致）
        if self.rt_priority > 0:
            try:
                os.sched_setscheduler(0, os.SCHED_FIFO,
                                      os.sched_param(self.rt_priority))
                print(f"[UART Worker] SCHED_FIFO priority={self.rt_priority}")
            except Exception as e:
                print(f"[UART Worker] SCHED_FIFO not available: {e}")

        next_send_time = time.perf_counter()
        stats_t0 = next_send_time
        stats_count0 = 0

        while not self.stop_event.is_set():
            now = time.perf_counter()

            if now >= next_send_time:
                late_sec = now - next_send_time
                if late_sec > self.send_period * 1.5:
                    self.deadline_late_count += 1

                # 读取共享状态
                with self.lock:
                    ax = self.ax
                    ay = self.ay
                    angle_z = self.angle_z
                    speed_z = self.speed_z
                    imu_valid = self.imu_valid

                # 构建 24B 帧
                ax_i = self._to_i32(ax, self.pos_scale)
                ay_i = self._to_i32(ay, self.pos_scale)
                angle_i = self._to_i32(angle_z, self.angle_scale)
                speed_i = self._to_i32(speed_z, self.speed_scale)
                ts = self._timestamp_ms()

                payload = build_payload(ax_i, ay_i, angle_i, speed_i, ts)
                pkt = make_packet(payload)

                ok = self.uart.write(pkt)
                self.send_count += 1
                if not ok:
                    self.write_fail_count += 1

                # 调试日志（每隔 debug_every 次）
                force_print = (self.debug_every > 0
                               and self.send_count % self.debug_every == 0)
                if force_print:
                    print(
                        f"[UART TX] #{self.send_count} "
                        f"ax={ax_i}, ay={ay_i}, "
                        f"angle_z={angle_i}, speed_z={speed_i}, "
                        f"ts={ts} | "
                        f"real=({ax:.1f}, {ay:.1f}, {angle_z:.2f}, {speed_z:.2f}) | "
                        f"write_ms={self.uart.last_write_ms:.3f} | "
                        f"ok={ok} | imu_valid={imu_valid} | "
                        f"pkt=[{hexdump(pkt)}]"
                    )

                # 单调时钟步进（与 E001 一致）
                next_send_time += self.send_period
                if next_send_time < time.perf_counter() - self.send_period:
                    next_send_time = time.perf_counter() + self.send_period

                # 每秒统计
                dt = time.perf_counter() - stats_t0
                if dt >= 1.0:
                    delta = self.send_count - stats_count0
                    self.last_tx_hz = delta / max(dt, 1e-6)
                    print(
                        f"[UART STATS] target={self.send_hz:.1f}Hz, "
                        f"real={self.last_tx_hz:.1f}Hz, "
                        f"sent={self.send_count}, "
                        f"fail={self.write_fail_count}, "
                        f"late_delta={self.deadline_late_count}, "
                        f"last_write_ms={self.uart.last_write_ms:.3f}"
                    )
                    stats_t0 = time.perf_counter()
                    stats_count0 = self.send_count

            else:
                remain = next_send_time - now
                if remain > 0.0008:
                    time.sleep(remain - 0.0003)
                elif remain > 0.0002:
                    time.sleep(0.0001)
                else:
                    time.sleep(0.00005)


# ──────────────────────────────────────────────
# ROS2 Node
# ──────────────────────────────────────────────
class UartSenderNode(Node):
    def __init__(self):
        super().__init__("uart_sender")

        # 尝试 SCHED_FIFO priority 30（与 E001 UART 线程一致）
        try:
            param = os.sched_param(os.SCHED_FIFO, 30)
            os.sched_setscheduler(0, os.SCHED_FIFO, param)
            self.get_logger().info("SCHED_FIFO priority=30 set")
        except Exception:
            self.get_logger().warn("SCHED_FIFO priority=30 not available")

        # 参数
        port = self.declare_parameter("port", "/dev/ttyS1").value
        baudrate = self.declare_parameter("baudrate", 115200).value
        send_hz = self.declare_parameter("send_hz", 200).value
        debug_every = self.declare_parameter("debug_every", 200).value
        pos_scale = self.declare_parameter("pos_scale", 100).value
        angle_scale = self.declare_parameter("angle_scale", 100).value
        speed_scale = self.declare_parameter("speed_scale", 100).value

        # UART 设备
        self.uart = Uart(
            port=port,
            baudrate=baudrate,
            timeout=0.01,
            write_timeout=0.05,
            flush_after_write=False,
        )

        # 200Hz 精确发送线程
        self.worker = UartSenderWorker(
            uart=self.uart,
            send_hz=send_hz,
            debug_every=debug_every,
            rt_priority=30,
            pos_scale=pos_scale,
            angle_scale=angle_scale,
            speed_scale=speed_scale,
        )

        # 订阅 IMU / Vision
        self.sub = self.create_subscription(
            String, "/imu/data", self.imu_cb, 10
        )
        self.vision_sub = self.create_subscription(
            String, "/vision/error", self.vision_cb, 10
        )

        self.last_ax = 0.0
        self.last_ay = 0.0
        self.last_angle_z = 0.0
        self.last_speed_z = 0.0

        # 可选的调试发布
        self.pub = self.create_publisher(String, "/uart/tx", 10)

        self.get_logger().info(
            f"UART Sender started: port={port}, baud={baudrate}, "
            f"send_hz={send_hz}Hz, debug_every={debug_every}, "
            f"proto=24B (AA+14+20B+cs+FF)"
        )

    def imu_cb(self, msg):
        """接收 IMU 数据并与最新视觉误差合并。"""
        try:
            parts = msg.data.split(",")
            if len(parts) >= 2:
                self.last_angle_z = float(parts[0])
                self.last_speed_z = float(parts[1])
                self.worker.update(ax=self.last_ax, ay=self.last_ay,
                                   angle_z=self.last_angle_z, speed_z=self.last_speed_z)
                dbg = String()
                dbg.data = f"ax={self.last_ax:.4f},ay={self.last_ay:.4f},angle_z={self.last_angle_z:.2f},speed_z={self.last_speed_z:.2f}"
                self.pub.publish(dbg)
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"[UART] parse imu msg failed: {msg.data}, {e}")

    def vision_cb(self, msg):
        """接收 vision error，并与最新 IMU 一起更新发送线程。"""
        try:
            parts = msg.data.split(",")
            if len(parts) >= 2:
                self.last_ax = float(parts[0])
                self.last_ay = float(parts[1])
                self.worker.update(ax=self.last_ax, ay=self.last_ay,
                                   angle_z=self.last_angle_z, speed_z=self.last_speed_z)
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"[UART] parse vision msg failed: {msg.data}, {e}")


def main(args=None):
    rclpy.init(args=args)
    node = UartSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("UART Node shutting down")
    finally:
        node.worker.stop()
        node.uart.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
