#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/jetson/ProjectsByMonthWU/VisionJetson/Copyfiles/26E_vision/install/setup.bash
set -u

project_root=/home/jetson/ProjectsByMonthWU/VisionJetson/Copyfiles/26E_vision
solver_log=/tmp/26e_challenge1_photo_solver_20260730.log
rm -f "$solver_log"

setsid ros2 run puzzle_solver_node puzzle_solver_node --ros-args \
  --params-file "$project_root/src/vision_bringup/config/vision_system.yaml" \
  --params-file "$project_root/src/vision_bringup/config/competition_tuning.yaml" \
  >"$solver_log" 2>&1 &
solver_pid=$!

cleanup() {
  kill -TERM -- -"$solver_pid" 2>/dev/null || true
  for _ in 1 2 3 4 5; do
    kill -0 "$solver_pid" 2>/dev/null || break
    sleep 0.3
  done
  kill -KILL -- -"$solver_pid" 2>/dev/null || true
}
trap cleanup EXIT
sleep 2

python3 - <<'PY'
import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import Point32
from rclpy.node import Node
from vision_interfaces.msg import PuzzlePiece, PuzzlePlan, PuzzleScene, TaskSession


PIECES = [
    (1, [[104.5, 12.0], [180.0, 79.0], [181.75, 15.75]], [155.37397318828909, 35.60791899493254], 2522.21875),
    (2, [[146.5, 84.0], [116.0, 78.25], [44.75, 101.25], [58.75, 128.75]], [88.80987208251989, 100.24384694581232], 2144.9375),
    (3, [[32.75, 74.25], [36.25, 80.0], [110.25, 69.25], [64.25, 51.0]], [66.99701729249456, 67.00528732214141], 1155.96875),
    (4, [[72.0, 24.75], [51.25, 19.5], [33.25, 29.0], [42.25, 47.25]], [49.87714359951498, 31.04390649960544], 541.21875),
]

case_name = os.environ.get("CHALLENGE_PHOTO_CASE", "bypass_all")
if case_name == "strict_filtered":
    selected_pieces = []
    for record in PIECES:
        polygon = record[1]
        lengths = []
        for index, (x0, y0) in enumerate(polygon):
            x1, y1 = polygon[(index + 1) % len(polygon)]
            lengths.append(math.hypot(x1 - x0, y1 - y0))
        if min(lengths) >= 20.0:
            selected_pieces.append(record)
elif case_name == "bypass_all":
    selected_pieces = PIECES
else:
    raise ValueError(f"unknown CHALLENGE_PHOTO_CASE={case_name}")


class Harness(Node):
    def __init__(self) -> None:
        super().__init__("challenge_one_photo_harness")
        self.session_pub = self.create_publisher(TaskSession, "puzzle/task_session", 1)
        self.scene_pub = self.create_publisher(PuzzleScene, "puzzle/scene", 1)
        self.plan = None
        self.create_subscription(PuzzlePlan, "puzzle/plan", self.on_plan, 1)

    def on_plan(self, message: PuzzlePlan) -> None:
        if message.task_id == 2 and message.generation == 1:
            self.plan = message


rclpy.init()
node = Harness()
deadline = time.monotonic() + 5.0
while time.monotonic() < deadline and (
    node.session_pub.get_subscription_count() < 1
    or node.scene_pub.get_subscription_count() < 1
):
    rclpy.spin_once(node, timeout_sec=0.1)

session = TaskSession()
session.task_id = 2
session.generation = 1
session.reason = f"PHOTO_DIAGNOSTIC_{case_name.upper()}"
node.session_pub.publish(session)
for _ in range(4):
    rclpy.spin_once(node, timeout_sec=0.15)

scene = PuzzleScene()
scene.task_id = 2
scene.generation = 1
scene.a4_detected = True
scene.valid = True
scene.workspace_mapping_valid = False
scene.status = f"PHOTO_DIAGNOSTIC_{case_name.upper()}"
scene.a4_to_workspace_homography = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
scene.a4_to_image_homography = [4.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 1.0]
scene.image_width = 840
scene.image_height = 1188
scene.green_frame_center_camera_px = Point32(x=0.0, y=0.0, z=0.0)
scene.rectified_origin_a4_mm = Point32(x=0.0, y=0.0, z=0.0)

for piece_id, polygon, center, area in selected_pieces:
    piece = PuzzlePiece()
    piece.id = piece_id
    piece.region = PuzzlePiece.REGION_UPPER
    piece.polygon_a4_mm = [Point32(x=float(x), y=float(y), z=0.0) for x, y in polygon]
    piece.center_a4_mm = Point32(x=float(center[0]), y=float(center[1]), z=0.0)
    piece.pick_point_a4_mm = Point32(x=float(center[0]), y=float(center[1]), z=0.0)
    edge_angles = []
    for index, (x0, y0) in enumerate(polygon):
        x1, y1 = polygon[(index + 1) % len(polygon)]
        edge_angles.append((math.hypot(x1 - x0, y1 - y0), -math.degrees(math.atan2(y1 - y0, x1 - x0))))
    piece.orientation_deg = float(max(edge_angles)[1])
    piece.area_mm2 = float(area)
    piece.confidence = 1.0
    scene.pieces.append(piece)

node.scene_pub.publish(scene)
deadline = time.monotonic() + 35.0
while time.monotonic() < deadline and node.plan is None:
    rclpy.spin_once(node, timeout_sec=0.2)

if node.plan is None:
    print(json.dumps({"received": False, "status": "TIMEOUT"}))
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(2)

plan = node.plan
output = {
    "case": case_name,
    "input_piece_count": len(selected_pieces),
    "received": True,
    "solved": bool(plan.solved),
    "status": plan.status,
    "target_width_mm": float(plan.target_width_mm),
    "target_height_mm": float(plan.target_height_mm),
    "score": float(plan.score),
    "score_margin": float(plan.score_margin),
    "placements": [],
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
                [float(point.x), float(point.y)] for point in placement.target_polygon_a4_mm
            ],
        }
    )

print(json.dumps(output, separators=(",", ":")))
node.destroy_node()
rclpy.shutdown()
PY

cleanup
trap - EXIT
echo "---SOLVER_LOG---"
tail -n 30 "$solver_log"
echo "---NODES_AFTER---"
ros2 node list
