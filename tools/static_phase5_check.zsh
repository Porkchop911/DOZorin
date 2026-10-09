#!/bin/zsh
# Stationary full sensor/odometry graph acceptance. Never commands motion and
# deliberately does not surface power telemetry.
set -u
source $HOME/hwr/deploy/env.zsh

LOG=/tmp/hwr_phase5_static.log
setsid ros2 launch hwr_odom odometry.launch.py >$LOG 2>&1 &
LPID=$!
cleanup() {
    if kill -0 $LPID 2>/dev/null; then
        kill -INT $LPID 2>/dev/null || true
        for _ in {1..20}; do
            kill -0 $LPID 2>/dev/null || return
            sleep 0.25
        done
        kill -TERM -$LPID 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM
sleep 14

print "=== nodes ==="
ros2 node list | sort
print "=== stationary topic rates ==="
for topic in /scan /camera/depth/image_raw /depth_ranges /imu_filtered /odom_scan /odom; do
    print -- "--- $topic"
    timeout 7 ros2 topic hz $topic 2>/dev/null | head -2
done
print "=== applied command (must be zero) ==="
timeout 5 ros2 topic echo /cmd_vel_applied --once 2>/dev/null | grep -A6 linear
print "=== wheel command (must be zero) ==="
timeout 5 ros2 topic echo /wheel_speeds --once 2>/dev/null | grep -A4 rps
print "=== fused odom TF ==="
timeout 5 ros2 run tf2_ros tf2_echo odom base_footprint 2>/dev/null | head -6

cleanup
wait $LPID 2>/dev/null || true
pkill -TERM -f '/home/ubuntu/hwr/ros2_ws/install/hwr_' 2>/dev/null || true
pkill -TERM -f '__node:=[h]wr_' 2>/dev/null || true
trap - EXIT INT TERM
print "stationary phase-5 graph complete"
