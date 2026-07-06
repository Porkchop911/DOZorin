#!/bin/zsh
# assignment3.sh — final legs: door (corrected to free cell) -> window -> start.
# Costmap-verified goals; every call hard-timeouted; photo per success.
source ~/.zshrc
L=~/scout_out/assignment.log
log(){ echo "$(date +%H:%M:%S) $1" >> $L }
batt(){ timeout 6 ros2 topic echo /ros_robot_controller/battery --once 2>/dev/null | grep -o '[0-9]*' | head -1 }
log "V3 START batt=$(batt)mV"

clearmaps(){
  timeout 8 ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}" > /dev/null 2>&1
  timeout 8 ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}" > /dev/null 2>&1
}

typeset -A DONE
STATIONS=(door window start)
coords_for(){
  case $1 in
    door)
      case $2 in
        1|2) echo "-1.42 -0.98 267" ;;
        *)   echo "-1.35 -0.90 267" ;;
      esac ;;
    window) echo "0.84 -1.08 353" ;;
    start)  echo "-0.22 0.05 44" ;;
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
    log "V3 ROUND $round LEG $st -> $C batt=$(batt)mV"
    timeout 150 python3 -u ~/goto.py ${=C} >> $L 2>&1
    RC=$?
    if grep -q "RESULT: SUCCEEDED" <(tail -3 $L); then
      DONE[$st]=1
      sleep 1.5
      curl -s "http://127.0.0.1:8080/snapshot?topic=/depth_cam/rgb0/image_raw" -o ~/scout_out/asgn_$st.jpg
      log "V3 LEG $st SUCCESS (photo asgn_$st.jpg)"
    elif [ $RC -eq 124 ]; then
      log "V3 LEG $st HUNG (150s) — killing goto"
      pkill -f "goto.py" 2>/dev/null
    else
      ERR=$(grep -E "not feasible|failed to generate|Extrapolation" /tmp/nav.log | tail -1 | sed 's/.*\] //' | cut -c1-90)
      log "V3 LEG $st FAILED round $round: $ERR"
      [ $round -ge 2 ] && timeout 90 python3 ~/global_loc.py > /dev/null 2>&1
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
