#!/bin/bash
# systemd and the manual entry point use the same complete vision flow.
exec /bin/bash /root/dev_ws/appli/start_new.sh "$@"
