import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class QrcCamKiller(Node):
    def __init__(self, name="1"):
        super().__init__(f'qrc_cam_killer_{name}')
        self._done = False
        self.qrc_res_sub = self.create_subscription(
            String,
            "qrc_result",
            self.kill_qrc,
            10,
        )
        self.qrc_kill_pub = self.create_publisher(String, "kill_qrc", 10)

    def kill_qrc(self, msg):
        """收到非空解码文本后停止扫码节点并退出。

        注意：不在回调内 destroy_node()（回调内销毁节点会让 rclpy 卡住、进程残留），
        改为置 _done 标志，由 main 的循环统一收尾。
        """
        if not msg.data:
            return
        self.get_logger().info(f"Received: {msg.data}, killing qrc* nodes")
        self.qrc_kill_pub.publish(String(data="kill"))
        time.sleep(0.3)  # 留出 DDS 发送窗口，确保 kill 消息送达订阅端
        self.get_logger().info("Killed all qrc* nodes, killer 自身退出")
        self._done = True


def main(args=None):
    rclpy.init(args=args)
    qrc_cam_killer = QrcCamKiller()
    try:
        while rclpy.ok() and not qrc_cam_killer._done:
            rclpy.spin_once(qrc_cam_killer, timeout_sec=0.2)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        qrc_cam_killer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
