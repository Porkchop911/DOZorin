#!/bin/zsh
# assignment.sh — drums -> door -> window -> start, retried until ALL succeed.
# No battery gate (user order; battery logged as info only). Photos per station.
source ~/.zshrc
L=~/scout_out/assignment.log
: > $L
log(){ echo "$(date +%H:%M:%S) $1" >> $L }
batt(){ timeout 6 ros2 topic echo /ros_robot_controller/battery --once 2>/dev/null | grep -o '[0-9]*' | head -1 }
log "ASSIGNMENT START batt=$(batt)mV"

# stack (pgrep-guarded) + arm forward-lock
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
if ! pgrep -f "bringup.launch.py map:=" > /dev/null; then
  nohup ros2 launch /home/ubuntu/ros2_ws/src/navigation/launch/include/bringup.launch.py map:=/home/ubuntu/ros2_ws/src/slam/maps/map_04.yaml use_teb:=true > /tmp/nav.log 2>&1 &
  log "nav launching, waiting 45s"; sleep 45
fi
pgrep -f depth_guard.py > /dev/null || nohup python3 ~/depth_guard.py > /tmp/guard.log 2>&1 &
# verify BOTH lifecycle groups; relaunch once if half-loaded
if [ "$(grep -c 'Managed nodes are active' /tmp/nav.log)" -lt 2 ]; then
  log "nav half-loaded, relaunching"
  pkill -f "bringup.launch.py map:="; sleep 4
  pkill -9 -f "component_container_isolated.*nav2_container"; sleep 3
  nohup ros2 launch /home/ubuntu/ros2_ws/src/navigation/launch/include/bringup.launch.py map:=/home/ubuntu/ros2_ws/src/slam/maps/map_04.yaml use_teb:=true > /tmp/nav.log 2>&1 &
  sleep 45
fi
log "nav-active-count $(grep -c 'Managed nodes are active' /tmp/nav.log)"

# localize (retry x3, refuse only if all fail — driving unlocalized helps nobody)
PCT=0
for i in 1 2 3; do
  S=$(python3 ~/global_loc.py 2>/dev/null | grep REFINED | sed 's/.*score=\([0-9.]*\).*max possible=\([0-9]*\).*/\1 \2/')
  PCT=$(echo $S | awk '{if ($2>0) printf "%d", 100*$1/$2; else print 0}')
  log "localize try $i: $S -> ${PCT}%"
  [ "$PCT" -ge 90 ] && break
  sleep 4
done
if [ "$PCT" -lt 90 ]; then log "LOCALIZATION FAILED 3x — ABORTING (would drive blind)"; exit 1; fi
sleep 2

clearmaps(){
  ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}" > /dev/null 2>&1
  ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}" > /dev/null 2>&1
}

typeset -A DONE
STATIONS=(drums door window start)
coords_for(){
  case $1 in
    drums)  echo "-0.01 -0.79 150" ;;
    window) echo "0.84 -1.08 353" ;;
    start)  echo "-0.22 0.05 44" ;;
    door)
      case $2 in
        1) echo "-1.55 -1.26 267" ;;
        2) echo "-1.50 -1.05 267" ;;
        *) echo "-1.42 -0.85 267" ;;
      esac ;;
  esac
}

for round in 1 2 3 4 5; do
  ALLDONE=1
  for st in $STATIONS; do [ -z "${DONE[$st]}" ] && ALLDONE=0; done
  [ $ALLDONE -eq 1 ] && break
  for st in $STATIONS; do
    [ -n "${DONE[$st]}" ] && continue
    C=$(coords_for $st $round)
    clearmaps; sleep 1
    log "ROUND $round LEG $st -> $C batt=$(batt)mV"
    timeout 150 python3 -u ~/goto.py ${=C} >> $L 2>&1
    RC=$?
    if grep -q "RESULT: SUCCEEDED" <(tail -3 $L); then
      DONE[$st]=1
      sleep 1.5
      curl -s "http://127.0.0.1:8080/snapshot?topic=/depth_cam/rgb0/image_raw" -o ~/scout_out/asgn_$st.jpg
      log "LEG $st SUCCESS (photo asgn_$st.jpg)"
    elif [ $RC -eq 124 ]; then
      log "LEG $st HUNG (150s timeout) — killing and continuing"
      pkill -f "goto.py" 2>/dev/null
    else
      log "LEG $st FAILED round $round"
      # re-localize between rounds if a leg keeps failing
      [ $round -ge 2 ] && python3 ~/global_loc.py > /dev/null 2>&1
    fi
  done
done

MISSING=""
for st in $STATIONS; do [ -z "${DONE[$st]}" ] && MISSING="$MISSING $st"; done
if [ -z "$MISSING" ]; then
  log "ASSIGNMENT COMPLETE batt=$(batt)mV"
else
  log "ASSIGNMENT INCOMPLETE after 5 rounds — missing:$MISSING batt=$(batt)mV"
fi
