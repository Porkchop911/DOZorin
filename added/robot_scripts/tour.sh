#!/bin/zsh
# tour.sh — THE assignment, one run, strict order: drums -> door -> window -> start.
# Photo at every station (asgn_1..4_*.jpg). ALL motion through Nav2; goals picked
# live from the costmap with clearance so arrival never traps the robot.
source ~/.zshrc
L=~/scout_out/assignment.log
log(){ echo "$(date +%H:%M:%S) $1" >> $L }
batt(){ timeout 6 ros2 topic echo /ros_robot_controller/battery --once 2>/dev/null | grep -o '[0-9]*' | head -1 }
log "TOUR START batt=$(batt)mV"

clearmaps(){
  timeout 8 ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}" > /dev/null 2>&1
  timeout 8 ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}" > /dev/null 2>&1
}

# fresh localization
PCT=0
for i in 1 2 3; do
  S=$(timeout 90 python3 ~/global_loc.py 2>/dev/null | grep REFINED | sed 's/.*score=\([0-9.]*\).*max possible=\([0-9]*\).*/\1 \2/')
  PCT=$(echo $S | awk '{if ($2>0) printf "%d", 100*$1/$2; else print 0}')
  log "TOUR localize try $i: ${PCT}%"
  [ "$PCT" -ge 85 ] && break
  sleep 4
done
if [ "$PCT" -lt 85 ]; then log "TOUR ABORT: localization ${PCT}%"; exit 1; fi
sleep 2

i=0
for st in drums door window start; do
  i=$((i+1))
  OK=""
  for try in 1 2 3 4 5; do
    clearmaps; sleep 2
    G=$(timeout 30 python3 ~/goal_pick.py $st 2>/dev/null)
    if [ -z "$G" ]; then log "TOUR $st try $try: goal_pick failed"; sleep 3; continue; fi
    log "TOUR LEG $i/$st try $try -> $G batt=$(batt)mV"
    timeout 150 python3 -u ~/goto.py ${=G} >> $L 2>&1
    RC=$?
    if grep -q "RESULT: SUCCEEDED" <(tail -3 $L); then
      OK=1
      sleep 1.5
      curl -s "http://127.0.0.1:8080/snapshot?topic=/depth_cam/rgb0/image_raw" -o ~/scout_out/asgn_${i}_$st.jpg
      log "TOUR LEG $i/$st SUCCESS (asgn_${i}_$st.jpg)"
      break
    elif [ $RC -eq 124 ]; then
      log "TOUR LEG $i/$st try $try HUNG — killing goto"
      pkill -f "goto.py" 2>/dev/null
    else
      ERR=$(grep -E "not feasible|failed to generate|Extrapolation" /tmp/nav.log | tail -1 | sed 's/.*\] //' | cut -c1-80)
      log "TOUR LEG $i/$st try $try FAILED: $ERR"
      sleep 4
      [ $try -ge 3 ] && timeout 90 python3 ~/global_loc.py > /dev/null 2>&1
    fi
  done
  if [ -z "$OK" ]; then
    log "ASSIGNMENT INCOMPLETE — tour died at leg $i/$st batt=$(batt)mV"
    exit 1
  fi
done
log "ASSIGNMENT COMPLETE — full ordered tour, 4 photos batt=$(batt)mV"
