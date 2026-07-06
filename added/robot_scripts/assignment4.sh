#!/bin/zsh
# assignment4.sh — remaining legs with ESCAPE maneuver: stations self-trap the
# robot (arrival pose sits in inflated cost -> TEB "not feasible" for the next
# goal). Escape = rotate ~180 in place, creep 0.4m forward into the space just
# driven through, clear costmaps.
source ~/.zshrc
L=~/scout_out/assignment.log
log(){ echo "$(date +%H:%M:%S) $1" >> $L }
batt(){ timeout 6 ros2 topic echo /ros_robot_controller/battery --once 2>/dev/null | grep -o '[0-9]*' | head -1 }
log "V4 START batt=$(batt)mV"

clearmaps(){
  timeout 8 ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}" > /dev/null 2>&1
  timeout 8 ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}" > /dev/null 2>&1
}
escape(){
  log "escape: rotate ~180 + forward 0.4m"
  timeout 8 ros2 topic pub -r 5 -t 20 /controller/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.9}}" > /dev/null 2>&1
  timeout 3 ros2 topic pub -t 2 /controller/cmd_vel geometry_msgs/msg/Twist "{}" > /dev/null 2>&1
  timeout 6 ros2 topic pub -r 5 -t 17 /controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.12}}" > /dev/null 2>&1
  timeout 3 ros2 topic pub -t 2 /controller/cmd_vel geometry_msgs/msg/Twist "{}" > /dev/null 2>&1
  sleep 1
  clearmaps
  sleep 1
}

typeset -A DONE
STATIONS=(door home)
coords_for(){
  case $1 in
    door)
      case $2 in
        1|2) echo "-1.42 -0.98 267" ;;
        *)   echo "-1.35 -0.90 267" ;;
      esac ;;
    home)  echo "-0.22 0.05 44" ;;
  esac
}

for round in 1 2 3 4; do
  ALL=1
  for st in $STATIONS; do [ -z "${DONE[$st]}" ] && ALL=0; done
  [ $ALL -eq 1 ] && break
  for st in $STATIONS; do
    [ -n "${DONE[$st]}" ] && continue
    C=$(coords_for $st $round)
    clearmaps; sleep 1
    log "V4 ROUND $round LEG $st -> $C batt=$(batt)mV"
    timeout 150 python3 -u ~/goto.py ${=C} >> $L 2>&1
    RC=$?
    if grep -q "RESULT: SUCCEEDED" <(tail -3 $L); then
      DONE[$st]=1
      sleep 1.5
      PH=$st; [ "$st" = "home" ] && PH=start
      curl -s "http://127.0.0.1:8080/snapshot?topic=/depth_cam/rgb0/image_raw" -o ~/scout_out/asgn_$PH.jpg
      log "V4 LEG $st SUCCESS (photo asgn_$PH.jpg)"
      # escape the arrival pocket unless everything is finished
      REMAIN=0
      for s2 in $STATIONS; do [ -z "${DONE[$s2]}" ] && REMAIN=1; done
      [ $REMAIN -eq 1 ] && escape
    elif [ $RC -eq 124 ]; then
      log "V4 LEG $st HUNG (150s) — killing goto"
      pkill -f "goto.py" 2>/dev/null
    else
      ERR=$(grep -E "not feasible|failed to generate|Extrapolation" /tmp/nav.log | tail -1 | sed 's/.*\] //' | cut -c1-90)
      log "V4 LEG $st FAILED round $round: $ERR"
      case $ERR in *"not feasible"*) escape ;; esac
    fi
  done
done

MISSING=""
for st in $STATIONS; do [ -z "${DONE[$st]}" ] && MISSING="$MISSING $st"; done
if [ -z "$MISSING" ]; then
  log "ASSIGNMENT COMPLETE batt=$(batt)mV"
else
  log "ASSIGNMENT INCOMPLETE missing:$MISSING batt=$(batt)mV"
fi
