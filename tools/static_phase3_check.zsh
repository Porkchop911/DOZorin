#!/bin/zsh
# Static (no-motion) verification of the Phase 3 ROS layer. Run on the robot
# inside a takeover window. Launches bringup, probes the graph, SIGINTs it.
source $HOME/hwr/deploy/env.zsh

print "=== launching hwr bringup ==="
setsid ros2 launch hwr_base bringup.launch.py > /tmp/hwr_bringup.log 2>&1 &
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
sleep 10

print "=== nodes ==="
ros2 node list

print "=== topic rates (5 s samples) ==="
for t in /imu /wheel_speeds /cmd_vel_applied /odom_model; do
    print -- "--- $t"
    timeout 6 ros2 topic hz $t 2>/dev/null | head -2
done
print -- "--- /joint_states"
timeout 6 ros2 topic hz /joint_states 2>/dev/null | head -2

print "=== wheel_speeds sample (must be all zero) ==="
timeout 5 ros2 topic echo /wheel_speeds --once 2>/dev/null | grep -A4 rps

print "=== joint_states sample ==="
timeout 5 ros2 topic echo /joint_states --once 2>/dev/null | grep -A7 "name:"

print "=== TF: base_link -> lidar_frame (expect yaw pi = quat z~1) ==="
timeout 5 ros2 run tf2_ros tf2_echo base_link lidar_frame 2>/dev/null | head -6

print "=== TF: odom -> base_footprint (model odom at rest) ==="
timeout 5 ros2 run tf2_ros tf2_echo odom base_footprint 2>/dev/null | head -6

print "=== estop service ==="
ros2 service list | grep estop

print "=== stopping launch process group ==="
cleanup
wait $LPID 2>/dev/null || true
# Non-interactive ros2 launch can orphan grandchildren on this Humble image.
# This check owns the HWR graph exclusively, so remove any such children.
pkill -TERM -f '/home/ubuntu/hwr/ros2_ws/install/hwr_' 2>/dev/null || true
trap - EXIT INT TERM
print "launch exited rc=$?"
tail -5 /tmp/hwr_bringup.log
