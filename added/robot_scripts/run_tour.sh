#!/bin/zsh
# run_tour.sh — detached waypoint tour; survives SSH drops. Progress -> ~/scout_out/tour.log
source ~/.zshrc
LOG=~/scout_out/tour.log
: > $LOG
echo "TOUR START $(date +%H:%M:%S)" >> $LOG
for t in drums door window; do
  echo "LEG $t BEGIN $(date +%H:%M:%S)" >> $LOG
  python3 -u ~/goto_place.py $t >> $LOG 2>&1
  sleep 1.5
  curl -s "http://127.0.0.1:8080/snapshot?topic=/depth_cam/rgb0/image_raw" -o ~/scout_out/tour2_$t.jpg
  echo "LEG $t END" >> $LOG
done
echo "LEG start BEGIN $(date +%H:%M:%S)" >> $LOG
python3 -u ~/goto.py -0.22 0.05 44 >> $LOG 2>&1
sleep 1.5
curl -s "http://127.0.0.1:8080/snapshot?topic=/depth_cam/rgb0/image_raw" -o ~/scout_out/tour2_start.jpg
echo "TOUR DONE $(date +%H:%M:%S)" >> $LOG
