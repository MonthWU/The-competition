"""Record only live frames, close AVI indexes on rotation and shutdown, and
keep only the newest video_archive_keep archives."""

import datetime
import os
import subprocess
import time
from glob import glob

import cv2 as cv
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class ObjVidDumper(Node):
    def __init__(self, name="1"):
        super().__init__(f"obj_vid_dumper_{name}")
        self.declare_parameter("video_dir", "/root/dev_ws/appli/_tmp_videos/")
        self.declare_parameter("video_fps", 20)
        self.declare_parameter("video_width", 640)
        self.declare_parameter("video_height", 480)
        self.declare_parameter("video_fourcc", "MJPG")
        self.declare_parameter("video_update_time", 20)
        self.declare_parameter("video_packup_time", 120)
        self.declare_parameter("video_archive_keep", 2)
        if self.get_parameter("video_fps").value <= 0:
            raise ValueError("video_fps must be positive")
        if self.get_parameter("video_archive_keep").value < 1:
            raise ValueError("video_archive_keep must be at least 1")
        os.makedirs(self.get_parameter("video_dir").value, exist_ok=True)
        self.prune_archives()
        self.video_fn = None
        self.video_fd = None
        self._next_write = 0.0
        self._written = 0
        self.timer_vid_fd_update = self.create_timer(
            self.get_parameter("video_update_time").value, self.timer_update_fd)
        self.timer_packup = self.create_timer(
            self.get_parameter("video_packup_time").value, self.timer_packup_callback)
        self.image_sub = self.create_subscription(
            CompressedImage, "image", self.image_callback, 10)

    def get_fd_fname(self):
        stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S_%f")
        return os.path.join(self.get_parameter("video_dir").value, stamp + ".avi")

    def close_video(self):
        if self.video_fd is not None:
            self.video_fd.release()
            self.get_logger().info(
                f"Video closed: {self.video_fn}, frames={self._written}")
        self.video_fd = None
        self._written = 0
        self._next_write = 0.0

    def timer_update_fd(self):
        # The next actual image opens the next segment; waiting for QR does
        # not create empty recordings.
        self.close_video()

    def dump_frame(self, frame):
        now = time.monotonic()
        if now < self._next_write:
            return
        width = self.get_parameter("video_width").value
        height = self.get_parameter("video_height").value
        if frame.shape[:2] != (height, width):
            frame = cv.resize(frame, (width, height))
        fps = self.get_parameter("video_fps").value
        if self.video_fd is None:
            self.video_fn = self.get_fd_fname()
            self.video_fd = cv.VideoWriter(
                self.video_fn,
                cv.VideoWriter.fourcc(*self.get_parameter("video_fourcc").value),
                fps, (width, height))
            if not self.video_fd.isOpened():
                self.close_video()
                raise RuntimeError(f"VIDEO_OPEN_FAILED: {self.video_fn}")
            self.get_logger().info(f"Video opened: {self.video_fn}")
        self.video_fd.write(frame)
        self._written += 1
        self._next_write = now + 1.0 / fps

    def image_callback(self, msg):
        try:
            frame = cv.imdecode(np.frombuffer(msg.data, np.uint8), cv.IMREAD_COLOR)
        except cv.error:
            frame = None
        if frame is None:
            self.get_logger().warn("Invalid image skipped by video recorder")
            return
        self.dump_frame(frame)

    def timer_packup_callback(self):
        self.close_video()
        videos = sorted(glob(os.path.join(self.get_parameter("video_dir").value, "*.avi")))
        if not videos:
            return
        stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S_%f")
        archive = os.path.join(self.get_parameter("video_dir").value, stamp + ".tar.gz")
        try:
            subprocess.run(["tar", "-czf", archive, *videos], check=True)
        except subprocess.CalledProcessError as error:
            self.get_logger().error(f"VIDEO_ARCHIVE_FAILED: {error}; original AVI files retained")
            return
        for video in videos:
            os.remove(video)
        self.prune_archives()

    def prune_archives(self):
        # Archives accumulate at ~60 MB per packup interval; without pruning
        # the root partition fills within roughly two hours of recording.
        keep = self.get_parameter("video_archive_keep").value
        archives = sorted(
            glob(os.path.join(self.get_parameter("video_dir").value, "*.tar.gz")),
            key=os.path.getmtime, reverse=True)
        for old in archives[keep:]:
            try:
                os.remove(old)
                self.get_logger().info(f"Pruned old archive: {os.path.basename(old)}")
            except OSError as error:
                self.get_logger().error(f"ARCHIVE_PRUNE_FAILED: {old}: {error}")


def main(args=None):
    rclpy.init(args=args)
    node = ObjVidDumper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.close_video()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
