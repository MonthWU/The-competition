"""IMU Node — RDK_25_E_004

移植自 RDK_25_E_001 的 device/IMU.py + device/ImuCache.py，
以 ROS2 MultiThreadedExecutor 风格运行。

架构：
  - 后台线程直接读取 UART3（与 E001 IMU._read_loop 一致），解析 0x55 0x52/0x53 帧
  - ImuCache 做毛刺过滤和断线保值（与 E001 ImuCache._loop 一致）
  - ROS2 定时器 ~50Hz 发布 /imu/data（String：angle_z,speed_z）
  - 支持 SCHED_FIFO priority 15

话题：
  - pub /imu/data (std_msgs/String): "angle_z,speed_z"
"""
import os
import threading
import time
import struct

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ──────────────────────────────────────────────
# IMU 后台读取线程 — 移植自 E001 的 device/IMU.py
# ──────────────────────────────────────────────
class ImuReader:
    """WIT/JY901 类 IMU 后台读取线程。

    与 E001 device/IMU.py 行为保持一致：
    - 自动发送配置命令（解锁、Z 轴归零、200Hz、保存、重启）
    - 后台线程持续从 UART3 读取，解析 0x55 0x53（角度）和 0x55 0x52（角速度）帧
    - 使用线程安全锁对外提供 get_angle() / get_gyro()
    """

    # 配置命令 — 与 E001 完全一致
    CMD_UNLOCK = bytes([0xFF, 0xAA, 0x69, 0x88, 0xB5])
    CMD_RESET_Z = bytes([0xFF, 0xAA, 0x76, 0x00, 0x00])
    CMD_SET_200HZ = bytes([0xFF, 0xAA, 0x03, 0x0B, 0x00])
    CMD_SET_115200 = bytes([0xFF, 0xAA, 0x04, 0x06, 0x00])
    CMD_SAVE = bytes([0xFF, 0xAA, 0x00, 0x00, 0x00])
    CMD_RESTART = bytes([0xFF, 0xAA, 0x00, 0xFF, 0x00])

    def __init__(self, port="/dev/ttyS3", baudrate=115200, auto_config=True):
        import serial

        self.port = port
        self.baudrate = baudrate

        # 线程安全数据
        self.lock = threading.Lock()
        self.angle_z = 0.0
        self.angular_velocity_y = 0.0
        self.angular_velocity_z = 0.0
        self.new_data_received = False
        self.bad_checksum_count = 0
        self.error_count = 0

        self.running = True
        self.rx_buffer = bytearray()
        self.ser = None
        self.available = False

        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=8,
                parity="N",
                stopbits=1,
                timeout=0.02,
            )
            self.available = True
            print(f"[IMU Reader] opened {port} @ {baudrate}")

            if auto_config:
                self._send_config()

        except Exception as e:
            print(f"[IMU Reader] open failed: {port}, {e}")
            self.available = False
            return

        # 启动后台读取线程
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def _send_cmd(self, cmd: bytes, delay=0.2):
        try:
            self.ser.write(cmd)
            time.sleep(delay)
        except Exception:
            pass

    def _send_config(self):
        """配置流程与 E001 device/IMU.py config_imu() 一致"""
        print("[IMU Reader] auto config start")
        self._send_cmd(self.CMD_UNLOCK)
        self._send_cmd(self.CMD_RESET_Z)
        self._send_cmd(self.CMD_SET_200HZ)
        self._send_cmd(self.CMD_SET_115200)
        self._send_cmd(self.CMD_SAVE)
        self._send_cmd(self.CMD_RESTART)
        time.sleep(0.5)
        print("[IMU Reader] auto config done")

    def _read_loop(self):
        """持续读取 UART，解析 0x55 0x52/0x53 帧 — 与 E001 IMU._read_loop 一致"""
        while self.running:
            try:
                if self.ser is None or not self.ser.is_open:
                    time.sleep(0.01)
                    continue

                n = self.ser.in_waiting
                if n <= 0:
                    time.sleep(0.001)
                    continue

                n = min(n, 128)
                data = self.ser.read(n)
                if not data:
                    time.sleep(0.001)
                    continue

                self.rx_buffer.extend(data)

                while len(self.rx_buffer) >= 11:
                    if self.rx_buffer[0] != 0x55:
                        self.rx_buffer.pop(0)
                        continue
                    if self.rx_buffer[1] not in (0x52, 0x53):
                        self.rx_buffer.pop(0)
                        continue

                    frame = bytes(self.rx_buffer[:11])
                    del self.rx_buffer[:11]
                    self._parse_frame(frame)

                if len(self.rx_buffer) > 256:
                    self.rx_buffer = self.rx_buffer[-64:]

            except Exception as e:
                self.error_count += 1
                if self.error_count % 50 == 0:
                    print(f"[IMU Reader ERROR] {e}")
                time.sleep(0.005)

    @staticmethod
    def _checksum(frame: bytes) -> int:
        return sum(frame[:10]) & 0xFF

    @staticmethod
    def _to_int16(low: int, high: int) -> int:
        v = (high << 8) | low
        return v - 65536 if v >= 32768 else v

    def _parse_frame(self, frame: bytes):
        """解析 0x55 帧 — 与 E001 IMU._parse_frame 一致"""
        if len(frame) != 11:
            return
        if frame[0] != 0x55:
            return
        if self._checksum(frame) != frame[10]:
            self.bad_checksum_count += 1
            return

        ft = frame[1]
        with self.lock:
            if ft == 0x53:  # 角度帧
                yaw_raw = self._to_int16(frame[6], frame[7])
                self.angle_z = yaw_raw / 32768.0 * 180.0
                self.new_data_received = True
            elif ft == 0x52:  # 角速度帧
                wy_raw = self._to_int16(frame[4], frame[5])
                wz_raw = self._to_int16(frame[6], frame[7])
                self.angular_velocity_y = wy_raw / 32768.0 * 2000.0
                self.angular_velocity_z = wz_raw / 32768.0 * 2000.0
                self.new_data_received = True

    def get_angle(self) -> float:
        with self.lock:
            return self.angle_z

    def get_gyro(self):
        with self.lock:
            return self.angular_velocity_y, self.angular_velocity_z

    def get_data(self):
        with self.lock:
            return {
                "angle_z": self.angle_z,
                "angular_velocity_y": self.angular_velocity_y,
                "angular_velocity_z": self.angular_velocity_z,
                "new_data_received": self.new_data_received,
                "bad_checksum_count": self.bad_checksum_count,
            }

    def close(self):
        self.running = False
        if hasattr(self, "thread") and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass


# ──────────────────────────────────────────────
# ImuCache — 毛刺过滤 + 丢包保值
# 移植自 E001 device/ImuCache.py
# ──────────────────────────────────────────────
class ImuCache:
    """对 ImuReader 的 angle_z / speed_z 做毛刺过滤。

    与 E001 ImuCache 一致：
    - 内部定期（~50Hz）从 ImuReader 拉取最新值
    - 超出 angle_abs_max / speed_abs_max 时使用上一轮有效值
    - 对外提供 get() -> (angle_z, speed_z, valid)
    """

    def __init__(self, reader: ImuReader,
                 read_hz=50,
                 angle_abs_max=180.0,
                 speed_abs_max=200.0):
        self.reader = reader
        self.read_period = 1.0 / max(1.0, read_hz)
        self.angle_limit = angle_abs_max
        self.speed_limit = speed_abs_max

        self.lock = threading.Lock()
        self.angle_z = 0.0
        self.speed_z = 0.0
        self.last_valid_angle_z = 0.0
        self.last_valid_speed_z = 0.0
        self.valid = False
        self.update_count = 0
        self.spike_count = 0
        self.error_count = 0

        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)

    def get(self):
        with self.lock:
            return self.angle_z, self.speed_z, self.valid

    def _is_spike(self, angle_z, speed_z):
        if abs(angle_z) > self.angle_limit:
            return True, f"angle_z={angle_z:.2f} > {self.angle_limit:.2f}"
        if abs(speed_z) > self.speed_limit:
            return True, f"speed_z={speed_z:.2f} > {self.speed_limit:.2f}"
        return False, ""

    def _use_last_valid(self):
        with self.lock:
            self.angle_z = self.last_valid_angle_z
            self.speed_z = self.last_valid_speed_z
            self.valid = False

    def _update_valid(self, angle_z, speed_z):
        with self.lock:
            self.angle_z = float(angle_z)
            self.speed_z = float(speed_z)
            self.last_valid_angle_z = float(angle_z)
            self.last_valid_speed_z = float(speed_z)
            self.valid = True

    def _loop(self):
        next_time = time.monotonic()
        while not self.stop_event.is_set():
            now = time.monotonic()
            if now >= next_time:
                try:
                    angle_z = float(self.reader.get_angle())
                    _, speed_z = self.reader.get_gyro()
                    speed_z = float(speed_z)

                    is_spike, reason = self._is_spike(angle_z, speed_z)
                    if is_spike:
                        self.spike_count += 1
                        self._use_last_valid()
                    else:
                        self._update_valid(angle_z, speed_z)
                        self.update_count += 1
                except Exception as e:
                    self.error_count += 1
                    self._use_last_valid()

                next_time += self.read_period
                if next_time < time.monotonic() - self.read_period:
                    next_time = time.monotonic() + self.read_period
            else:
                time.sleep(0.001)


# ──────────────────────────────────────────────
# ROS2 Node
# ──────────────────────────────────────────────
class ImuNode(Node):
    def __init__(self):
        super().__init__("imu_node")

        # 尝试设置 RT 优先级 15
        try:
            param = os.sched_param(os.SCHED_FIFO, 15)
            os.sched_setscheduler(0, os.SCHED_FIFO, param)
            self.get_logger().info("SCHED_FIFO priority=15 set")
        except Exception:
            self.get_logger().warn("SCHED_FIFO priority=15 not available")

        # 从参数或环境变量读取串口配置
        port = self.declare_parameter("port", "/dev/ttyS3").value
        baudrate = self.declare_parameter("baudrate", 115200).value
        auto_config = self.declare_parameter("auto_config", True).value
        pub_hz = self.declare_parameter("pub_hz", 50).value

        # IMU 读取线程（独立后台线程，与 E001 一致）
        self.imu_reader = ImuReader(port=port, baudrate=baudrate, auto_config=auto_config)
        # ImuCache 毛刺过滤（独立后台线程，与 E001 一致）
        self.imu_cache = ImuCache(
            reader=self.imu_reader,
            read_hz=pub_hz,
            angle_abs_max=180.0,
            speed_abs_max=200.0,
        )

        # ROS2 发布
        self.pub = self.create_publisher(String, "/imu/data", 10)
        # 定时器频率由 pub_hz 参数控制，默认 50Hz
        self.timer = self.create_timer(1.0 / max(1.0, pub_hz), self._publish_cb)
        self._publish_count = 0

        self.get_logger().info(
            f"IMU Node started: port={port}, baud={baudrate}, "
            f"auto_config={auto_config}, pub_hz={pub_hz}, "
            f"available={self.imu_reader.available}"
        )

    def _publish_cb(self):
        angle_z, speed_z, valid = self.imu_cache.get()
        self._publish_count += 1

        msg = String()
        msg.data = f"{angle_z:.4f},{speed_z:.4f}"
        self.pub.publish(msg)

        # 每 50 次发布打一次日志（约每秒一次）
        if self._publish_count % 50 == 1:
            self.get_logger().info(
                f"[IMU] pub angle_z={angle_z:.2f}, speed_z={speed_z:.2f}, "
                f"valid={valid}, cache_updates={self.imu_cache.update_count}, "
                f"spikes={self.imu_cache.spike_count}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("IMU Node shutting down")
    finally:
        node.imu_cache.stop()
        node.imu_reader.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
