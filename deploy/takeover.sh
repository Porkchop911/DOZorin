#!/bin/zsh
# takeover.sh - take exclusive ownership of the robot hardware from the factory stack.
# Runs ON the robot. Idempotent: re-running while taken over is a no-op.
# Never commands motion. Counterpart: handback.sh.
set -u

STATE=$HOME/hwr/.takeover
LOG=$HOME/hwr/switchover.log
# The udev-stable robot-controller link may resolve to ttyACM0 or ttyACM1
# after a reboot.  Never hard-code the transient kernel enumeration.
PORT=/dev/rrc
BOOT_ID=$(</proc/sys/kernel/random/boot_id)

ts() { date '+%Y-%m-%d %H:%M:%S' }
note() { print "$(ts) takeover $1" >> $LOG }

if [[ -f $STATE ]]; then
    state_boot=$(awk 'NR == 1 { print $1 }' $STATE 2>/dev/null)
    if [[ $state_boot == $BOOT_ID ]] && ! systemctl is-active --quiet start_app_node.service; then
        print "already taken over in this boot since $(cut -d' ' -f2- $STATE) - no-op"
        exit 0
    fi
    print "discarding stale takeover marker from an earlier boot or active factory state"
    note "STALE previous=$(tr ' ' '_' <$STATE)"
    rm -f $STATE
fi

# 1) stop the factory stack if it is running. Telemetry never controls takeover.
if systemctl is-active --quiet start_app_node.service; then
    sudo systemctl stop start_app_node.service
    print "start_app_node.service stopped"
else
    print "start_app_node.service already inactive"
fi

# 2) wait until nothing except button_scan holds the serial port (timeout 15 s).
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
        note "FAIL reason=port-busy"
        exit 1
    fi
    sleep 1
done
print "$PORT free (button_scan may still hold it - that is fine)"

# 3) state file + log
print "$BOOT_ID $(ts)" > $STATE
note "OK"
print "OK - hardware taken over. Restore with deploy/handback.sh"
