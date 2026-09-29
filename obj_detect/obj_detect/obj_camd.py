import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import subprocess
import os
import time


class ObjCamd(Node):
    """Object detection camera daemon
    Run usb camera after qrc code camera released
    """

    def __init__(self, name="1"):
        super().__init__(f"obj_camd_{name}")
        self.qrc_res_sub = self.create_subscription(
            String,
            "kill_qrc",
            self.on_qrcam_killed,
            10,
        )

        self.declare_parameters(
            namespace="",
            parameters=[
                ("usb_video_device", "/dev/video0"),
                ("usb_framerate", 90),
                ("usb_image_width", 640),
                ("usb_image_height", 480),
                (
                    "launch_file_path",
                    "/root/dev_ws/appli/obj_detect/launch/obj_cam.launch.py",
                ),
            ],
        )

    def on_qrcam_killed(self, msg):
        """To start object detection camera"""
        self.get_logger().info(
            f"Received: {msg.data}, starting object detection camera"
        )
        # Launch object detection camera
        self.get_logger().info(
            f"Launching object detection camera {self.get_parameter('usb_video_device').get_parameter_value().string_value}"
        )
        self.obj_cam_launch()

    def obj_cam_launch(self):
        """Launch object detection camera"""
        device = self.get_parameter("usb_video_device").value
        if not os.path.exists(device):
            self.get_logger().error(f"CAMERA_NOT_FOUND: {device}")
            return
        launch_file_path = (
            self.get_parameter("launch_file_path").get_parameter_value().string_value
        )
        self.get_logger().info(f"launch_file_path: {launch_file_path}")
        # Launch object detection camera
        subprocess.Popen(
            [
                "ros2",
                "launch",
                launch_file_path,
                f"cap_objdet:={device}",
                f"usb_framerate:={self.get_parameter('usb_framerate').value}",
            ]
        )


def main(args=None):
    rclpy.init(args=args)
    obj_camd = ObjCamd()
    rclpy.spin(obj_camd)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
