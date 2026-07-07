#!/bin/zsh
# handback.sh - return the hardware to the factory stack and verify it is healthy again.
# Runs ON the robot. Idempotent. Counterpart: takeover.sh.
set -u

STATE=$HOME/hwr/.takeover
LOG=$HOME/hwr/switchover.log
START=$(date +%s)

ts() { date '+%Y-%m-%d %H:%M:%S' }
note() { print "$(ts) handback $1" >> $LOG }

# 1) stop any hwr processes (guarded pattern: only our node/launch names)
pkill -f 'hwr_(driver|base|teleop|sensors|odom|slam|loc|nav|explore|arm|gateway|agent)' 2>/dev/null
sleep 1

# 2) restart the factory stack
sudo systemctl start start_app_node.service
print "start_app_node.service started, waiting for factory health..."

source $HOME/ros2_ws/.robotrc >/dev/null 2>&1

# 3) poll until /scan and the battery topic publish and the web UI answers (timeout 60 s)
deadline=$(( START + 60 ))
scan_ok=0; batt_ok=0; web_ok=0; BATT=""
while (( $(date +%s) < deadline )); do
    if (( ! scan_ok )); then
        if timeout 5 ros2 topic echo /scan --once >/dev/null 2>&1; then
            scan_ok=1; print "  /scan alive"
        fi
    fi
    if (( ! batt_ok )); then
        BATT=$(timeout 5 ros2 topic echo /ros_robot_controller/battery --once 2>/dev/null | awk '/data:/{print $2; exit}')
        if [[ -n $BATT ]]; then
            batt_ok=1; print "  battery topic alive (${BATT} mV)"
        fi
    fi
    if (( ! web_ok )); then
        if curl -s -m 3 -o /dev/null localhost:8080; then
            web_ok=1; print "  web UI :8080 answering"
        fi
    fi
    if (( scan_ok && batt_ok && web_ok )); then
        break
    fi
    sleep 2
done

ELAPSED=$(( $(date +%s) - START ))
if (( scan_ok && batt_ok && web_ok )); then
    rm -f $STATE
    note "OK battery=${BATT}mV elapsed=${ELAPSED}s"
    print "OK - factory stack healthy again (${ELAPSED}s)"
else
    note "FAIL battery=${BATT:-?}mV elapsed=${ELAPSED}s scan=$scan_ok batt=$batt_ok web=$web_ok"
    print "ERROR: factory stack not fully healthy after ${ELAPSED}s (scan=$scan_ok batt=$batt_ok web=$web_ok)"
    print "recovery: sudo systemctl restart start_app_node.service; if wedged: sudo fuser -k /dev/ttyACM0 then restart"
    exit 1
fi
