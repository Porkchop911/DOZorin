ps aux | grep ros2 | grep -v grep | awk '{ print "kill -9", $2 }' | sh
ps aux | grep ros-args | grep -v grep | awk '{ print "kill -9", $2 }' | sh

