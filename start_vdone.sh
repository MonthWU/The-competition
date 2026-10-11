#!/bin/bash
set -e

for i in $(seq 1 30); do
  if [ -e /dev/video0 ] || [ -e /dev/video1 ] || [ -e /dev/video2 ]; then
    break
  fi
  sleep 1
done

# USB 摄像头节点出现后再额外等一会，避免 systemd 开机早期刚枚举完但 V4L2 还未完全 ready
sleep 3

cd /root/Monthwu/RDK_vision/RDK_CJLU_26_VDone
source /opt/tros/humble/setup.bash
source /root/Monthwu/RDK_vision/RDK_CJLU_26_VDone/install/setup.bash
exec ros2 launch /root/Monthwu/RDK_vision/RDK_CJLU_26_VDone/launch/system.launch.py
