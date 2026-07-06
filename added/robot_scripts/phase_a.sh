#!/bin/zsh
# phase_a.sh — post-boot bring-up + localization ONLY. No driving.
# Ends with scan_overlay at /tmp/overlay.png for off-board verification.
source ~/.zshrc
L=~/scout_out/assignment.log
log(){ echo "$(date +%H:%M:%S) $1" >> $L }
batt(){ timeout 6 ros2 topic echo /ros_robot_controller/battery --once 2>/dev/null | grep -o '[0-9]*' | head -1 }
log "PHASE-A START batt=$(batt)mV"

# 1. kill CPU hogs that autostart resurrects on every boot
pkill -9 -f "lib/example/yolo_node" 2>/dev/null
pkill -9 -f "lib/example/self_driving" 2>/dev/null
pkill -9 -f "lib/app/line_following" 2>/dev/null
pkill -9 -f "lib/app/object_tracking" 2>/dev/null
log "cpu hogs killed"

# 2. arm forward-lock
python3 - <<'PYEOF' >> $L 2>&1
import time, rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import ServosPosition, ServoPosition
rclpy.init(); n=Node("aimf")
pub=n.create_publisher(ServosPosition,"/ros_robot_controller/bus_servo/set_position",1)
time.sleep(0.8)
m=ServosPosition(); m.duration=1.5
for sid,pos in ((1,500),(2,701),(3,120),(4,300),(5,500)):
    sp=ServoPosition(); sp.id=sid; sp.position=pos; m.position.append(sp)
for _ in range(3): pub.publish(m); time.sleep(0.1)
time.sleep(2.2); print("arm locked")
n.destroy_node(); rclpy.shutdown()
PYEOF

# 3. nav bring-up with half-load detection
if ! pgrep -f "bringup.launch.py map:=" > /dev/null; then
  nohup ros2 launch /home/ubuntu/ros2_ws/src/navigation/launch/include/bringup.launch.py map:=/home/ubuntu/ros2_ws/src/slam/maps/map_04.yaml use_teb:=true > /tmp/nav.log 2>&1 &
  log "nav launching, waiting 45s"; sleep 45
fi
if [ "$(grep -c 'Managed nodes are active' /tmp/nav.log)" -lt 2 ]; then
  log "nav half-loaded, relaunching"
  pkill -f "bringup.launch.py map:="; sleep 4
  pkill -9 -f "component_container_isolated.*nav2_container"; sleep 3
  nohup ros2 launch /home/ubuntu/ros2_ws/src/navigation/launch/include/bringup.launch.py map:=/home/ubuntu/ros2_ws/src/slam/maps/map_04.yaml use_teb:=true > /tmp/nav.log 2>&1 &
  sleep 45
fi
log "nav-active-count $(grep -c 'Managed nodes are active' /tmp/nav.log)"
pgrep -f depth_guard.py > /dev/null || nohup python3 ~/depth_guard.py > /tmp/guard.log 2>&1 &

# 4. localize, STRICT: accept only >=95%
PCT=0
for i in 1 2 3 4 5; do
  S=$(timeout 90 python3 ~/global_loc.py 2>/dev/null | grep REFINED | sed 's/.*score=\([0-9.]*\).*max possible=\([0-9]*\).*/\1 \2/')
  PCT=$(echo $S | awk '{if ($2>0) printf "%d", 100*$1/$2; else print 0}')
  log "localize try $i: $S -> ${PCT}%"
  [ "$PCT" -ge 95 ] && break
  sleep 4
done
log "PHASE-A LOCALIZE FINAL ${PCT}%"

# 5. evidence: overlay + camera snapshot
timeout 60 python3 ~/scan_overlay.py > /tmp/ovl.log 2>&1
curl -s "http://127.0.0.1:8080/snapshot?topic=/depth_cam/rgb0/image_raw" -o ~/scout_out/phase_a_cam.jpg
log "PHASE-A DONE batt=$(batt)mV overlay=/tmp/overlay.png"
