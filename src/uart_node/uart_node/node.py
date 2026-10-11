"""UART Sender Node — V01 (de-IMU)

第二轮源码减法目标：
- 彻底去掉 IMU 依赖；
- 保留独立 200Hz 精确发送线程；
- 只接收视觉侧结果；
- 当前优先支持 /vision/target: "d,a_x"，并兼容旧 /vision/error: "ax,ay"。

当前 24B 包结构仍保持 5 个整数位，便于与旧下位机链路兼容：
    <i i i i I>
    [v0, v1, 0, 0, timestamp_ms]
其中：
- 若 topic=/vision/target，则 v0=d, v1=a_x
- 若 topic=/vision/error，则 v0=ax, v1=ay
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
FRAME_HEAD = 0xAA
FRAME_TAIL = 0xFF
PAYLOAD_LEN = 20


def clamp_i32(v):
    return max(INT32_MIN, min(INT32_MAX, int(v)))


def hexdump(data):
    return " ".join(f"{int(b) & 0xFF:02X}" for b in data)


def build_payload(v0, v1, reserve0, reserve1, timestamp_ms):
    return struct.pack(
        "<iiiiI",
        clamp_i32(v0),
        clamp_i32(v1),
        clamp_i32(reserve0),
        clamp_i32(reserve1),
        int(timestamp_ms) & UINT32_MAX,
    )


def make_packet(payload: bytes) -> bytes:
    if len(payload) != PAYLOAD_LEN:
        raise ValueError(f"payload must be {PAYLOAD_LEN}B, got {len(payload)}B")
    checksum = sum(payload) & 0xFF
    return bytes([FRAME_HEAD, PAYLOAD_LEN]) + payload + bytes([checksum, FRAME_TAIL])


class Uart:
    def __init__(self, port, baudrate=115200, timeout=0.01,
                 write_timeout=0.05, flush_after_write=False):
        import serial
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.write_timeout = write_timeout
        self.flush_after_write = flush_after_write

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


class UartSenderWorker:
    """精确 200Hz 发送线程；已去掉 IMU 字段。"""

    def __init__(self, uart: Uart, send_hz=200, debug_every=200,
                 rt_priority=30, value0_scale=100, value1_scale=100):
        self.uart = uart
        self.send_hz = send_hz
        self.send_period = 1.0 / max(1.0, send_hz)
        self.debug_every = debug_every
        self.rt_priority = rt_priority
        self.value0_scale = value0_scale
        self.value1_scale = value1_scale

        self.lock = threading.Lock()
        self.value0 = 0.0
        self.value1 = 0.0
        self.value0_name = 'v0'
        self.value1_name = 'v1'

        self.send_count = 0
        self.write_fail_count = 0
        self.deadline_late_count = 0
        self.last_tx_hz = 0.0

        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def update(self, value0, value1, value0_name='v0', value1_name='v1'):
        with self.lock:
            self.value0 = float(value0)
            self.value1 = float(value1)
            self.value0_name = str(value0_name)
            self.value1_name = str(value1_name)

    def stop(self):
        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)

    def _to_i32(self, value, scale):
        return clamp_i32(round(float(value) * float(scale)))

    def _timestamp_ms(self):
        return int(time.monotonic() * 1000) & UINT32_MAX

    def _loop(self):
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

                with self.lock:
                    value0 = self.value0
                    value1 = self.value1
                    value0_name = self.value0_name
                    value1_name = self.value1_name

                value0_i = self._to_i32(value0, self.value0_scale)
                value1_i = self._to_i32(value1, self.value1_scale)
                ts = self._timestamp_ms()

                payload = build_payload(value0_i, value1_i, 0, 0, ts)
                pkt = make_packet(payload)

                ok = self.uart.write(pkt)
                self.send_count += 1
                if not ok:
                    self.write_fail_count += 1

                force_print = (self.debug_every > 0 and self.send_count % self.debug_every == 0)
                if force_print:
                    print(
                        f"[UART TX] #{self.send_count} "
                        f"{value0_name}={value0_i}, {value1_name}={value1_i}, ts={ts} | "
                        f"real=({value0:.4f}, {value1:.4f}) | "
                        f"write_ms={self.uart.last_write_ms:.3f} | ok={ok} | "
                        f"pkt=[{hexdump(pkt)}]"
                    )

                next_send_time += self.send_period
                if next_send_time < time.perf_counter() - self.send_period:
                    next_send_time = time.perf_counter() + self.send_period

                dt = time.perf_counter() - stats_t0
                if dt >= 1.0:
                    delta = self.send_count - stats_count0
                    self.last_tx_hz = delta / max(dt, 1e-6)
                    print(
                        f"[UART STATS] target={self.send_hz:.1f}Hz, real={self.last_tx_hz:.1f}Hz, "
                        f"sent={self.send_count}, fail={self.write_fail_count}, "
                        f"late_delta={self.deadline_late_count}, last_write_ms={self.uart.last_write_ms:.3f}"
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


class UartSenderNode(Node):
    def __init__(self):
        super().__init__("uart_sender")

        try:
            param = os.sched_param(os.SCHED_FIFO, 30)
            os.sched_setscheduler(0, os.SCHED_FIFO, param)
            self.get_logger().info("SCHED_FIFO priority=30 set")
        except Exception:
            self.get_logger().warn("SCHED_FIFO priority=30 not available")

        port = self.declare_parameter("port", "/dev/ttyS1").value
        baudrate = self.declare_parameter("baudrate", 115200).value
        send_hz = self.declare_parameter("send_hz", 200).value
        debug_every = self.declare_parameter("debug_every", 200).value
        target_topic = self.declare_parameter("target_topic", "/vision/target").value
        error_topic = self.declare_parameter("error_topic", "/vision/error").value
        packet_mode = self.declare_parameter("packet_mode", "d_ax").value
        value0_scale = self.declare_parameter("value0_scale", 100).value
        value1_scale = self.declare_parameter("value1_scale", 100).value

        self.uart = Uart(
            port=port,
            baudrate=baudrate,
            timeout=0.01,
            write_timeout=0.05,
            flush_after_write=False,
        )

        self.worker = UartSenderWorker(
            uart=self.uart,
            send_hz=send_hz,
            debug_every=debug_every,
            rt_priority=30,
            value0_scale=value0_scale,
            value1_scale=value1_scale,
        )

        self.packet_mode = str(packet_mode)
        self.target_sub = self.create_subscription(String, target_topic, self.target_cb, 10)
        self.error_sub = self.create_subscription(String, error_topic, self.error_cb, 10)
        self.pub = self.create_publisher(String, "/uart/tx", 10)

        self.get_logger().info(
            f"UART Sender started: port={port}, baud={baudrate}, send_hz={send_hz}Hz, "
            f"target_topic={target_topic}, error_topic={error_topic}, packet_mode={self.packet_mode}, "
            f"proto=24B (AA+14+20B+cs+FF), IMU removed"
        )

    def _publish_dbg(self, name0, v0, name1, v1):
        dbg = String()
        dbg.data = f"{name0}={v0:.4f},{name1}={v1:.4f}"
        self.pub.publish(dbg)

    def target_cb(self, msg):
        try:
            parts = msg.data.split(",")
            if len(parts) >= 2:
                d = float(parts[0])
                a_x = float(parts[1])
                self.worker.update(d, a_x, "d", "a_x")
                self._publish_dbg("d", d, "a_x", a_x)
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"[UART] parse target msg failed: {msg.data}, {e}")

    def error_cb(self, msg):
        # 兼容旧视觉链路；如果已切到 /vision/target，此回调不会影响新主链。
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
        node.worker.stop()
        node.uart.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
