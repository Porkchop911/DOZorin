#!/bin/zsh
# handback.sh - return the hardware to the factory stack and verify it is healthy again.
# Runs ON the robot. Idempotent. Counterpart: takeover.sh.
set -u

STATE=$HOME/hwr/.takeover
LOG=$HOME/hwr/switchover.log
START=$(date +%s)

ts() { date '+%Y-%m-%d %H:%M:%S' }
note() { print "$(ts) handback $1" >> $LOG }

# 1) stop any hwr processes.  Some vendor executables are renamed only through
# ROS arguments (for example, sclidar ... __node:=hwr_lidar), so matching just
# executable/package names leaves hardware-owning children behind.
if [[ -x $HOME/hwr/deploy/mode.sh ]]; then
    $HOME/hwr/deploy/mode.sh stop || true
fi
# Fallback cleanup also covers legacy/manual sessions and separately launched
# gateway, arm, recorder, or agent nodes.
pkill -TERM -f '__node:=[h]wr_' 2>/dev/null || true
pkill -TERM -f '/home/ubuntu/[h]wr/ros2_ws/install/' 2>/dev/null || true
pkill -TERM -f 'ros2 launch [h]wr_' 2>/dev/null || true
sleep 1

# 2) restart the factory stack
sudo systemctl start start_app_node.service
print "start_app_node.service started, waiting for factory health..."

# vendor rc appends to possibly-unset vars (LD_LIBRARY_PATH) - must source without -u
set +u
source $HOME/ros2_ws/.robotrc >/dev/null 2>&1
set -u

# 3) poll until /scan publishes and the web UI answers (timeout 60 s)
deadline=$(( START + 60 ))
scan_ok=0; web_ok=0
while (( $(date +%s) < deadline )); do
    if (( ! scan_ok )); then
        if timeout 5 ros2 topic echo /scan --once >/dev/null 2>&1; then
            scan_ok=1; print "  /scan alive"
        fi
    fi
    if (( ! web_ok )); then
        if curl -s -m 3 -o /dev/null localhost:8080; then
            web_ok=1; print "  web UI :8080 answering"
        fi
    fi
    if (( scan_ok && web_ok )); then
        break
    fi
    sleep 2
done

ELAPSED=$(( $(date +%s) - START ))
if (( scan_ok && web_ok )); then
    rm -f $STATE
    note "OK elapsed=${ELAPSED}s"
    print "OK - factory stack healthy again (${ELAPSED}s)"
else
    note "FAIL elapsed=${ELAPSED}s scan=$scan_ok web=$web_ok"
    print "ERROR: factory stack not fully healthy after ${ELAPSED}s (scan=$scan_ok web=$web_ok)"
    print "recovery: sudo systemctl restart start_app_node.service; if wedged: sudo fuser -k /dev/ttyACM0 then restart"
    exit 1
fi
