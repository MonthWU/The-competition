#!/bin/bash
set -e
LOCK=/tmp/v01.lock
exec 9>"$LOCK"
if ! flock -n 9; then
  echo "V01 already running"
  exit 0
fi
cd /root/Monthwu/RDK_vision/V01
source /opt/tros/humble/setup.bash
source /root/Monthwu/RDK_vision/V01/install/setup.bash
exec ros2 launch /root/Monthwu/RDK_vision/V01/launch/system.launch.py
