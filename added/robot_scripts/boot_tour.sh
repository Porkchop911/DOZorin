#!/bin/zsh
# boot_tour.sh — detached full chain: bring-up -> verify -> localize (gated) -> tour.
# All progress to ~/scout_out/boot_tour.log; survives SSH drops.
source ~/.zshrc
L=~/scout_out/boot_tour.log
: > $L
log(){ echo "$(date +%H:%M:%S) $1" >> $L }
log "BOOT-TOUR START"

timeout 8 ros2 topic echo /ros_robot_controller/battery --once 2>/dev/null | head -1 >> $L

# arm forward-lock
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

pgrep -f "bringup.launch.py map:=" > /dev/null || nohup ros2 launch /home/ubuntu/ros2_ws/src/navigation/launch/include/bringup.launch.py map:=/home/ubuntu/ros2_ws/src/slam/maps/map_04.yaml use_teb:=true > /tmp/nav.log 2>&1 &
pgrep -f depth_guard.py > /dev/null || nohup python3 ~/depth_guard.py > /tmp/guard.log 2>&1 &
log "stack launching, waiting 45s"
sleep 45
log "nav-active-count $(grep -c 'Managed nodes are active' /tmp/nav.log)"

# localize, gate on fit score >=90%
SCORE=$(python3 ~/global_loc.py 2>/dev/null | grep REFINED | sed 's/.*score=\([0-9.]*\).*max possible=\([0-9]*\).*/\1 \2/')
log "global_loc: $SCORE"
PCT=$(echo $SCORE | awk '{if ($2>0) printf "%d", 100*$1/$2; else print 0}')
log "fit ${PCT}%"
if [ "$PCT" -lt 90 ]; then
  log "LOCALIZATION FAILED (<90%) — NOT DRIVING"
  exit 1
fi
sleep 2

for t in drums door window; do
  log "LEG $t BEGIN"
  python3 -u ~/goto_place.py $t >> $L 2>&1
  sleep 1.5
  curl -s "http://127.0.0.1:8080/snapshot?topic=/depth_cam/rgb0/image_raw" -o ~/scout_out/tour2_$t.jpg
  log "LEG $t END"
done
log "LEG start BEGIN"
python3 -u ~/goto.py -0.22 0.05 44 >> $L 2>&1
sleep 1.5
curl -s "http://127.0.0.1:8080/snapshot?topic=/depth_cam/rgb0/image_raw" -o ~/scout_out/tour2_start.jpg
log "battery-end $(timeout 6 ros2 topic echo /ros_robot_controller/battery --once 2>/dev/null | head -1)"
log "BOOT-TOUR DONE"
