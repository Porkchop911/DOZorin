#!/bin/zsh
# Static (no-motion) lidar bring-up check (plan 4.1/4.2). Run on the robot
# inside a takeover window. Launches our lidar stack, probes it, TERMs it.
source $HOME/hwr/deploy/env.zsh

print "=== launching hwr_sensors lidar ==="
setsid ros2 launch hwr_sensors lidar.launch.py > /tmp/hwr_lidar.log 2>&1 &
LPID=$!
cleanup() {
    if kill -0 $LPID 2>/dev/null; then
        # Signal the launch supervisor only; it forwards once to its children.
        kill -INT $LPID 2>/dev/null || true
        for _ in {1..20}; do
            kill -0 $LPID 2>/dev/null || return
            sleep 0.25
        done
        # Last resort: terminate the entire isolated process group.
        kill -TERM -$LPID 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM
sleep 8

print "=== nodes ==="
ros2 node list

print "=== topic rates (expect ~10 Hz both) ==="
for t in /scan_raw /scan; do
    print -- "--- $t"
    timeout 8 ros2 topic hz $t 2>/dev/null | head -2
done

print "=== geometry + matcher checks ==="
python3 $HOME/hwr/tools/live_phase4_lidar_check.py
RC=$?

print "=== stopping launch (SIGTERM) ==="
cleanup
wait $LPID 2>/dev/null || true
pkill -TERM -f '/home/ubuntu/hwr/ros2_ws/install/hwr_' 2>/dev/null || true
trap - EXIT INT TERM
tail -5 /tmp/hwr_lidar.log
print "check rc=$RC"
exit $RC
