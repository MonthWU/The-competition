#!/bin/bash
set -e
LOCK=/tmp/v02.lock
exec 9>"$LOCK"
if ! flock -n 9; then
  echo "V02 already running"
  exit 0
fi
cd /root/Monthwu/RDK_vision/V02
source /opt/tros/humble/setup.bash
source /root/Monthwu/RDK_vision/V02/install/setup.bash
exec ros2 launch /root/Monthwu/RDK_vision/V02/launch/system.launch.py
