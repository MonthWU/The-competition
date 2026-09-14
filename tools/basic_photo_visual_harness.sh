#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
project_root=/home/jetson/ProjectsByMonthWU/VisionJetson/Copyfiles/26E_vision
source "$project_root/install/setup.bash"
set -u

photo_path=${1:-$project_root/test_data/basic_photo/source.jpg}
output_dir=${2:-/tmp/26e_basic_photo_visual}
mkdir -p "$output_dir"
perception_log="$output_dir/perception.log"
solver_log="$output_dir/solver.log"
rm -f "$perception_log" "$solver_log" "$output_dir/plan.json" \
  "$output_dir/solver_debug.jpg"

params1="$project_root/src/vision_bringup/config/vision_system.yaml"
params2="$project_root/src/vision_bringup/config/competition_tuning.yaml"
camera_pipeline="filesrc location=$photo_path ! jpegdec ! imagefreeze ! videoconvert ! video/x-raw,format=BGR,framerate=30/1 ! appsink drop=true max-buffers=1 sync=false"

setsid ros2 run puzzle_perception_node puzzle_perception_node --ros-args \
  --params-file "$params1" --params-file "$params2" \
  -p camera_pipeline:="$camera_pipeline" \
  -p green_a4_h_min:=35 -p green_a4_h_max:=75 \
  -p green_a4_s_min:=80 -p green_a4_s_max:=255 \
  -p green_a4_v_min:=70 -p green_a4_v_max:=255 \
  -p debug_mode:=true -p publish_debug_image:=true \
  >"$perception_log" 2>&1 &
perception_pid=$!

setsid ros2 run puzzle_solver_node puzzle_solver_node --ros-args \
  --params-file "$params1" --params-file "$params2" -p debug_mode:=true \
  >"$solver_log" 2>&1 &
solver_pid=$!

cleanup() {
  for pid in "$perception_pid" "$solver_pid"; do
    kill -TERM -- -"$pid" 2>/dev/null || true
  done
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    if ! kill -0 "$perception_pid" 2>/dev/null && ! kill -0 "$solver_pid" 2>/dev/null; then
      break
    fi
    sleep 0.2
  done
  for pid in "$perception_pid" "$solver_pid"; do
    kill -KILL -- -"$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT
sleep 2

OUTPUT_DIR="$output_dir" python3 - <<'PY'
import json
import os
import pathlib
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_interfaces.msg import PuzzlePlan, PuzzleScene, ScanRequest, TaskSession


class Harness(Node):
    def __init__(self) -> None:
        super().__init__("basic_photo_visual_harness")
        self.session_pub = self.create_publisher(TaskSession, "puzzle/task_session", 10)
        self.scan_pub = self.create_publisher(ScanRequest, "puzzle/scan_request", 10)
        self.plan = None
        self.debug = None
        self.scene = None
        self.create_subscription(PuzzlePlan, "puzzle/plan", self.on_plan, 1)
        self.create_subscription(PuzzleScene, "puzzle/scene", self.on_scene, 1)
        self.create_subscription(
            Image, "puzzle/solver_image_debug", self.on_debug, qos_profile_sensor_data
        )

    def on_plan(self, message: PuzzlePlan) -> None:
        if message.task_id == 1 and message.generation == 1:
            self.plan = message

    def on_debug(self, message: Image) -> None:
        if message.encoding != "bgr8" or message.width == 0 or message.height == 0:
            return
        rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
        self.debug = rows[:, : message.width * 3].reshape(message.height, message.width, 3).copy()

    def on_scene(self, message: PuzzleScene) -> None:
        if message.task_id == 1 and message.generation == 1:
            self.scene = message


rclpy.init()
node = Harness()
deadline = time.monotonic() + 8.0
while time.monotonic() < deadline and (
    node.session_pub.get_subscription_count() < 2
    or node.scan_pub.get_subscription_count() < 1
):
    rclpy.spin_once(node, timeout_sec=0.1)

session = TaskSession(task_id=1, generation=1, reason="BASIC_PHOTO_VISUAL_ONLY")
node.session_pub.publish(session)
for _ in range(5):
    rclpy.spin_once(node, timeout_sec=0.1)
node.scan_pub.publish(ScanRequest(task_id=1, generation=1))

deadline = time.monotonic() + 15.0
while time.monotonic() < deadline and (node.plan is None or node.debug is None):
    rclpy.spin_once(node, timeout_sec=0.2)

if node.plan is None:
    raise SystemExit("PLAN_TIMEOUT")

plan = node.plan
output = {
    "solved": bool(plan.solved),
    "status": plan.status,
    "target_width_mm": float(plan.target_width_mm),
    "target_height_mm": float(plan.target_height_mm),
    "score": float(plan.score),
    "score_margin": float(plan.score_margin),
    "scene": None,
    "placements": [],
}
if node.scene is not None:
    output["scene"] = {
        "valid": bool(node.scene.valid),
        "status": node.scene.status,
        "a4_detected": bool(node.scene.a4_detected),
        "piece_count": len(node.scene.pieces),
        "pieces": [
            {
                "id": int(piece.id),
                "area_mm2": float(piece.area_mm2),
                "vertex_count": len(piece.polygon_a4_mm),
                "confidence": float(piece.confidence),
            }
            for piece in node.scene.pieces
        ],
    }
for placement in plan.placements:
    output["placements"].append(
        {
            "piece_id": int(placement.piece_id),
            "source_center_a4_mm": [
                float(placement.source_center_a4_mm.x),
                float(placement.source_center_a4_mm.y),
            ],
            "target_center_a4_mm": [
                float(placement.target_center_a4_mm.x),
                float(placement.target_center_a4_mm.y),
            ],
            "rotation_delta_deg": float(placement.rotation_delta_deg),
            "target_polygon_a4_mm": [
                [float(point.x), float(point.y)]
                for point in placement.target_polygon_a4_mm
            ],
        }
    )

output_dir = pathlib.Path(os.environ["OUTPUT_DIR"])
(output_dir / "plan.json").write_text(
    json.dumps(output, ensure_ascii=False, indent=2), encoding="utf-8"
)
if node.debug is not None:
    cv2.imwrite(str(output_dir / "solver_debug.jpg"), node.debug)
print(json.dumps(output, ensure_ascii=False, separators=(",", ":")))
node.destroy_node()
rclpy.shutdown()
PY

cleanup
trap - EXIT
echo "---PERCEPTION_LOG---"
tail -n 40 "$perception_log"
echo "---SOLVER_LOG---"
tail -n 40 "$solver_log"
echo "---NODES_AFTER---"
ros2 node list
