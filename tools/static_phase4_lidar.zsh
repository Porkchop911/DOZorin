#!/bin/zsh
# Static (no-motion) lidar bring-up check (plan 4.1/4.2). Run on the robot
# inside a takeover window. Launches our lidar stack, probes it, TERMs it.
source $HOME/hwr/deploy/env.zsh

print "=== launching hwr_sensors lidar ==="
ros2 launch hwr_sensors lidar.launch.py > /tmp/hwr_lidar.log 2>&1 &
LPID=$!
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
kill -TERM $LPID
wait $LPID 2>/dev/null
tail -5 /tmp/hwr_lidar.log
print "check rc=$RC"
exit $RC
