#!/bin/zsh
# tour_down1.sh — downstairs legs 1+2: big plant, kitchen (vision-discovered poses).
source ~/.zshrc
L=~/scout_out/downstairs.log
log(){ echo "$(date +%H:%M:%S) $1" >> $L }
batt(){ timeout 6 ros2 topic echo /ros_robot_controller/battery --once 2>/dev/null | grep -o '[0-9]*' | head -1 }
log "TOUR-DOWN-1 START batt=$(batt)mV"

clearmaps(){
  timeout 8 ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}" > /dev/null 2>&1
  timeout 8 ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}" > /dev/null 2>&1
}

leg(){
  local name=$1 gx=$2 gy=$3 gyaw=$4
  for try in 1 2 3; do
    clearmaps; sleep 2
    log "LEG $name try $try -> $gx $gy $gyaw batt=$(batt)mV"
    timeout 150 python3 -u ~/goto.py $gx $gy $gyaw >> $L 2>&1
    local RC=$?
    if grep -q "RESULT: SUCCEEDED" <(tail -3 $L); then
      sleep 1.5
      curl -s "http://127.0.0.1:8080/snapshot?topic=/depth_cam/rgb0/image_raw" -o ~/scout_out/asgn_d_$name.jpg
      log "LEG $name SUCCESS (asgn_d_$name.jpg)"
      return 0
    elif [ $RC -eq 124 ]; then
      log "LEG $name try $try HUNG — killing goto"; pkill -f "goto.py" 2>/dev/null
    else
      ERR=$(grep -E "not feasible|failed to generate|Extrapolation" /tmp/nav.log | tail -1 | sed 's/.*\] //' | cut -c1-80)
      log "LEG $name try $try FAILED: $ERR"
      sleep 4
    fi
  done
  log "LEG $name EXHAUSTED"
  return 1
}

leg plant 3.46 -1.8 337 || { log "TOUR-DOWN-1 DEAD at plant"; exit 1 }
leg kitchen 4.35 -2.78 317 || { log "TOUR-DOWN-1 DEAD at kitchen"; exit 1 }
log "TOUR-DOWN-1 COMPLETE batt=$(batt)mV"
