"""UART Sender Node — V01 (de-IMU)

视觉数据协议（新版 14B）：
[0]      1B   帧头        FRAME_HEAD       = 0x91
[1..4]   4B   distance     int32_le        // round(distance * 100)
[5..8]   4B   vision_x     int32_le        // round(vision_x * 100)
[9..12]  4B   timestamp_ms uint32_le       // 毫秒时间戳
[13]     1B   帧尾        FRAME_TAIL       = 0x93

固定发送频率目标：50Hz。
"""
import os
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


INT32_MIN = -2147483648
INT32_MAX = 2147483647
UINT32_MAX = 0xFFFFFFFF
FRAME_HEAD = 0x91
FRAME_TAIL = 0x93
PACKET_LEN = 14
SEND_HZ = 50.0


def clamp_i32(v):
    return max(INT32_MIN, min(INT32_MAX, int(v)))


def hexdump(data):
    return " ".join(f"{int(b) & 0xFF:02X}" for b in data)


def make_packet(distance, vision_x, timestamp_ms):
    return (
        bytes([FRAME_HEAD])
        + struct.pack(
            "<iiI",
            clamp_i32(distance),
            clamp_i32(vision_x),
            int(timestamp_ms) & UINT32_MAX,
        )
        + bytes([FRAME_TAIL])
    )


class Uart:
    def __init__(self, port, baudrate=115200, timeout=0.01,
                 write_timeout=0.05, flush_after_write=False):
        import serial
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.write_timeout = write_timeout
        self.flush_after_write = flush_after_write
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout,
            write_timeout=self.write_timeout,
        )

    def write(self, data: bytes):
        t0 = time.perf_counter()
        n = self.ser.write(data)
        if self.flush_after_write:
            self.ser.flush()
        dt_ms = (time.perf_counter() - t0) * 1000.0
        return n == len(data), dt_ms

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


class SenderWorker:
    def __init__(self, node, uart, pos_scale=100, rt_priority=20, debug_every=200):
        self.node = node
        self.uart = uart
        self.pos_scale = pos_scale
        self.rt_priority = rt_priority
        self.debug_every = debug_every
        self.lock = threading.Lock()
        self.distance = 0.0
        self.vision_x = 0.0
        self.last_names = ("distance", "vision_x")
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.sent = 0
        self.fail = 0
        self.last_write_ms = 0.0
        self.stats_last_t = time.time()
        self.stats_last_sent = 0
        self.thread.start()

    def _set_sched_fifo(self):
        try:
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(self.rt_priority))
            self.node.get_logger().info(f"SCHED_FIFO priority={self.rt_priority} enabled")
        except Exception:
            self.node.get_logger().warn(f"SCHED_FIFO priority={self.rt_priority} not available")

    def update(self, distance, vision_x, name0="distance", name1="vision_x"):
        with self.lock:
            self.distance = float(distance)
            self.vision_x = float(vision_x)
            self.last_names = (name0, name1)

    def stop(self):
        self.running = False
        self.thread.join(timeout=1.0)

    def _to_i32(self, value, scale):
        return clamp_i32(round(float(value) * float(scale)))

    def _loop(self):
        self._set_sched_fifo()
        period = 1.0 / SEND_HZ
        next_t = time.perf_counter()

        while self.running:
            now = time.perf_counter()
            sleep_t = next_t - now
            if sleep_t > 0:
                time.sleep(sleep_t)
            next_t += period

            with self.lock:
                distance = self.distance
                vision_x = self.vision_x

            distance_i = self._to_i32(distance, self.pos_scale)
            vision_x_i = self._to_i32(vision_x, self.pos_scale)
            ts = int(time.monotonic() * 1000) & UINT32_MAX
            pkt = make_packet(distance_i, vision_x_i, ts)
            ok, dt_ms = self.uart.write(pkt)
            self.last_write_ms = dt_ms
            self.sent += 1
            if not ok:
                self.fail += 1

            if self.debug_every > 0 and (self.sent % self.debug_every == 0):
                self.node.get_logger().info(
                    f"[UART TX] #{self.sent} distance={distance_i}, vision_x={vision_x_i}, ts={ts} | "
                    f"real=({distance:.4f}, {vision_x:.4f}) | write_ms={dt_ms:.3f} | ok={ok} | pkt=[{hexdump(pkt)}]"
                )

            t = time.time()
            if t - self.stats_last_t >= 1.0:
                hz = (self.sent - self.stats_last_sent) / (t - self.stats_last_t)
                self.node.get_logger().info(
                    f"[UART STATS] target={SEND_HZ:.1f}Hz, real={hz:.1f}Hz, sent={self.sent}, fail={self.fail}, last_write_ms={self.last_write_ms:.3f}"
                )
                self.stats_last_t = t
                self.stats_last_sent = self.sent


class UartSenderNode(Node):
    def __init__(self):
        super().__init__('uart_sender')

        self.declare_parameter("port", "/dev/ttyS1")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("timeout", 0.01)
        self.declare_parameter("write_timeout", 0.05)
        self.declare_parameter("flush_after_write", False)
        self.declare_parameter("target_topic", "/vision/target")
        self.declare_parameter("error_topic", "/vision/error")
        self.declare_parameter("pos_scale", 100)
        self.declare_parameter("rt_priority", 20)
        self.declare_parameter("debug_every", 200)
        self.declare_parameter("packet_mode", "distance_vision_x")

        port = self.get_parameter("port").value
        baudrate = int(self.get_parameter("baudrate").value)
        timeout = float(self.get_parameter("timeout").value)
        write_timeout = float(self.get_parameter("write_timeout").value)
        flush_after_write = bool(self.get_parameter("flush_after_write").value)
        target_topic = str(self.get_parameter("target_topic").value)
        error_topic = str(self.get_parameter("error_topic").value)
        pos_scale = int(self.get_parameter("pos_scale").value)
        rt_priority = int(self.get_parameter("rt_priority").value)
        debug_every = int(self.get_parameter("debug_every").value)
        self.packet_mode = str(self.get_parameter("packet_mode").value)

        self.uart = Uart(port, baudrate, timeout, write_timeout, flush_after_write)
        self.worker = SenderWorker(self, self.uart, pos_scale=pos_scale, rt_priority=rt_priority, debug_every=debug_every)

        self.pub = self.create_publisher(String, "/uart/tx", 10)
        self.sub_target = self.create_subscription(String, target_topic, self.target_cb, 10)
        self.sub_error = self.create_subscription(String, error_topic, self.error_cb, 10)

        self.get_logger().info(
            f"UART Sender started: port={port}, baud={baudrate}, send_hz={SEND_HZ:.0f}Hz, "
            f"target_topic={target_topic}, error_topic={error_topic}, packet_mode={self.packet_mode}, proto=14B (91 + 12B + 93), IMU removed"
        )

    def _publish_dbg(self, name0, v0, name1, v1):
        dbg = String()
        dbg.data = f"{name0}={v0:.4f},{name1}={v1:.4f}"
        self.pub.publish(dbg)

    def target_cb(self, msg):
        try:
            parts = msg.data.split(",")
            if len(parts) >= 2:
                distance = float(parts[0])
                vision_x = float(parts[1])
                self.worker.update(distance, vision_x, "distance", "vision_x")
                self._publish_dbg("distance", distance, "vision_x", vision_x)
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"[UART] parse target msg failed: {msg.data}, {e}")

    def error_cb(self, msg):
        try:
            parts = msg.data.split(",")
            if len(parts) >= 2:
                ax = float(parts[0])
                ay = float(parts[1])
                if self.packet_mode == 'vision_error':
                    self.worker.update(ax, ay, "ax", "ay")
                    self._publish_dbg("ax", ax, "ay", ay)
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"[UART] parse vision error msg failed: {msg.data}, {e}")


def main(args=None):
    rclpy.init(args=args)
    node = UartSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("UART Node shutting down")
    finally:
        try:
            node.worker.stop()
        except Exception:
            pass
        try:
            node.uart.close()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
