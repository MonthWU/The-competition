import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import subprocess
import os
import time
import signal


class ObjCamd(Node):
    """Object detection camera daemon
    Run usb camera after qrc code camera released
    """

    def __init__(self, name="1"):
        super().__init__(f"obj_camd_{name}")
        self._requested = False
        self._release_deadline = 0.0
        self._camera_launch = None
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
                ("qrc_video_device", "/dev/v4l/by-id/usb-KINGSEN_KS1A293-video-index0"),
                ("usb_framerate", 90),
                ("usb_image_width", 640),
                ("usb_image_height", 480),
                (
                    "launch_file_path",
                    "/root/dev_ws/appli/obj_detect/launch/obj_cam.launch.py",
                ),
            ],
        )

        self.watch_timer = self.create_timer(0.1, self.check_camera)

    def on_qrcam_killed(self, msg):
        """To start object detection camera"""
        if msg.data != "kill" or self._requested or self._camera_launch is not None:
            return
        self._requested = True
        self._release_deadline = time.monotonic() + 10.0
        self.get_logger().info("QR forwarded; waiting for the QR camera to release before object scan")

    def check_camera(self):
        if self._camera_launch is not None:
            returncode = self._camera_launch.poll()
            if returncode is not None:
                raise RuntimeError(f"OBJECT_CAMERA_EXITED: rc={returncode}")
            return
        if not self._requested:
            return
        qrc_device = self.get_parameter("qrc_video_device").value
        result = subprocess.run(["fuser", "-s", qrc_device], capture_output=True, timeout=1.0)
        if result.returncode == 0:
            if time.monotonic() >= self._release_deadline:
                raise RuntimeError(f"QR_CAMERA_RELEASE_TIMEOUT: {qrc_device}")
            return
        if result.returncode != 1:
            raise RuntimeError(f"QR_CAMERA_RELEASE_CHECK_FAILED: {qrc_device}")
        self.obj_cam_launch()

    def obj_cam_launch(self):
        """Launch object detection camera"""
        device = self.get_parameter("usb_video_device").value
        if not os.path.exists(device):
            raise RuntimeError(f"CAMERA_NOT_FOUND: {device}")
        launch_file_path = (
            self.get_parameter("launch_file_path").get_parameter_value().string_value
        )
        self.get_logger().info(f"launch_file_path: {launch_file_path}")
        # Launch object detection camera
        self._camera_launch = subprocess.Popen(
            [
                "ros2",
                "launch",
                launch_file_path,
                f"cap_objdet:={device}",
                f"usb_framerate:={self.get_parameter('usb_framerate').value}",
                f"usb_image_width:={self.get_parameter('usb_image_width').value}",
                f"usb_image_height:={self.get_parameter('usb_image_height').value}",
            ],
            start_new_session=True,
        )
        self.get_logger().info(f"Object camera launch started: pid={self._camera_launch.pid}")

    def stop_camera(self):
        if self._camera_launch is None:
            return
        for shutdown_signal, timeout in ((signal.SIGINT, 4), (signal.SIGTERM, 2), (signal.SIGKILL, 2)):
            if self._camera_launch.poll() is not None:
                break
            try:
                os.killpg(self._camera_launch.pid, shutdown_signal)
                self._camera_launch.wait(timeout=timeout)
            except ProcessLookupError:
                break
            except subprocess.TimeoutExpired:
                continue
        self._camera_launch = None


def main(args=None):
    rclpy.init(args=args)
    obj_camd = ObjCamd()
    try:
        rclpy.spin(obj_camd)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        obj_camd.stop_camera()
        obj_camd.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
