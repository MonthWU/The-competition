#!/bin/bash
set -e
LOCK=/tmp/rdk_25_e_done.lock
exec 9>"$LOCK"
if ! flock -n 9; then
  echo "RDK_25_E_Done already running"
  exit 0
fi
cd /root/Monthwu/RDK_vision/RDK_25_E_Done
source /opt/tros/humble/setup.bash
source /root/Monthwu/RDK_vision/RDK_25_E_Done/install/setup.bash
exec ros2 launch /root/Monthwu/RDK_vision/RDK_25_E_Done/launch/system.launch.py
