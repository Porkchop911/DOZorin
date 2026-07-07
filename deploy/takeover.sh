#!/bin/zsh
# takeover.sh - take exclusive ownership of the robot hardware from the factory stack.
# Runs ON the robot. Idempotent: re-running while taken over is a no-op.
# Never commands motion. Counterpart: handback.sh.
set -u

STATE=$HOME/hwr/.takeover
LOG=$HOME/hwr/switchover.log
PORT=/dev/ttyACM0

ts() { date '+%Y-%m-%d %H:%M:%S' }
note() { print "$(ts) takeover $1" >> $LOG }

if [[ -f $STATE ]]; then
    print "already taken over since $(cat $STATE) - no-op"
    exit 0
fi

# vendor rc appends to possibly-unset vars (LD_LIBRARY_PATH) - must source without -u
set +u
source $HOME/ros2_ws/.robotrc >/dev/null 2>&1
set -u

# 1) battery gate: read from the factory topic while the factory stack still runs
BATT=""
if systemctl is-active --quiet start_app_node.service; then
    BATT=$(timeout 12 ros2 topic echo /ros_robot_controller/battery --once 2>/dev/null | awk '/data:/{print $2; exit}')
    if [[ -z $BATT ]]; then
        print "ERROR: factory service active but no battery reading - not taking over"
        note "FAIL reason=no-battery-reading"
        exit 1
    fi
    if (( BATT < 11000 )); then
        print "ERROR: battery ${BATT} mV < 11000 - charge first"
        note "FAIL battery=${BATT}mV reason=low-battery"
        exit 1
    fi
    print "battery ${BATT} mV - ok"
else
    print "WARN: factory service not running, battery gate skipped - verify charge yourself"
fi

# 2) stop the factory stack (button_scan is a separate service and stays up - by design)
sudo systemctl stop start_app_node.service
print "start_app_node.service stopped"

# 3) wait until nothing except button_scan holds the serial port (timeout 15 s).
#    Both the factory driver and button_scan appear as python3, so match cmdline per PID.
deadline=$(( $(date +%s) + 15 ))
while true; do
    busy=0
    for pid in $(sudo fuser $PORT 2>/dev/null); do
        if ! grep -qa button_scan /proc/$pid/cmdline 2>/dev/null; then
            busy=1
        fi
    done
    if (( busy == 0 )); then
        break
    fi
    if (( $(date +%s) >= deadline )); then
        print "ERROR: $PORT still held by a non-button_scan process after 15 s:"
        sudo fuser -v $PORT
        note "FAIL battery=${BATT:-?}mV reason=port-busy"
        exit 1
    fi
    sleep 1
done
print "$PORT free (button_scan may still hold it - that is fine)"

# 4) state file + log
ts > $STATE
note "OK battery=${BATT:-?}mV"
print "OK - hardware taken over. Restore with deploy/handback.sh"
