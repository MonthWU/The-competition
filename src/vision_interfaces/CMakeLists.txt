cmake_minimum_required(VERSION 3.8)
project(vision_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Roi.msg"
  "msg/TargetPoint.msg"
  "msg/PuzzlePiece.msg"
  "msg/PuzzleScene.msg"
  "msg/PuzzlePlacement.msg"
  "msg/PuzzlePlan.msg"
  "msg/TaskSession.msg"
  "msg/ScanRequest.msg"
  "msg/PlacementDone.msg"
  DEPENDENCIES sensor_msgs std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
