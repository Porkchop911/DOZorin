#!/bin/zsh
# Start/stop exactly one HWR mapping, localization, navigation, or exploration process group.
set -eu

ROOT=$HOME/hwr
STATE=$ROOT/state
PID_FILE=$STATE/mode_session.pid
MODE_FILE=$STATE/mode_session.name
MAP_FILE=$STATE/mode_session.map
LOG_FILE=$STATE/mode_session.log

mkdir -p $STATE
source $ROOT/deploy/env.zsh

group_alive() {
  [[ -f $PID_FILE ]] || return 1
  session_pid=$(<$PID_FILE)
  [[ $session_pid == <-> ]] || return 1
  kill -0 -- -$session_pid 2>/dev/null
}

stop_mode() {
  if ! group_alive; then
    rm -f $PID_FILE $MODE_FILE
    print "no HWR autonomy mode is running"
    return 0
  fi
  session_mode=unknown
  [[ -f $MODE_FILE ]] && session_mode=$(<$MODE_FILE)
  print "stopping HWR $session_mode mode (process group $session_pid)"
  kill -TERM -- -$session_pid
  for attempt in {1..40}; do
    kill -0 -- -$session_pid 2>/dev/null || break
    sleep 0.25
  done
  if kill -0 -- -$session_pid 2>/dev/null; then
    print "mode did not stop after 10 s; terminating remaining group processes"
    kill -KILL -- -$session_pid 2>/dev/null || true
  fi
  rm -f $PID_FILE $MODE_FILE $MAP_FILE
}

start_mode() {
  requested=$1
  map_yaml=${2:-}
  if group_alive; then
    print -u2 "HWR $(<$MODE_FILE) mode is already running (process group $session_pid)"
    return 1
  fi
  rm -f $PID_FILE $MODE_FILE

  case $requested in
    mapping)
      launch_command="ros2 launch hwr_slam mapping.launch.py"
      ;;
    localization)
      [[ -n $map_yaml && -f $map_yaml ]] || {
        print -u2 "localization requires an existing absolute map YAML path"
        return 2
      }
      launch_command="ros2 launch hwr_loc localization.launch.py map_yaml:=$map_yaml"
      ;;
    navigation)
      [[ -n $map_yaml && -f $map_yaml ]] || {
        print -u2 "navigation requires an existing absolute map YAML path"
        return 2
      }
      launch_command="ros2 launch hwr_gateway autonomy.launch.py map_yaml:=$map_yaml"
      ;;
    exploration)
      [[ -n $map_yaml ]] || map_yaml=$ROOT/maps/downstairs_map
      mkdir -p ${map_yaml:h}
      launch_command="ros2 launch hwr_explore exploration.launch.py map_path:=$map_yaml"
      ;;
    *)
      print -u2 "usage: $0 start mapping|localization|navigation|exploration [map path]"
      return 2
      ;;
  esac

  # A dedicated session/process group makes one signal reach ros2 launch and
  # every child node. This prevents orphaned hardware nodes during mode swaps.
  setsid zsh -lc "source $ROOT/deploy/env.zsh; exec $launch_command" \
    >$LOG_FILE 2>&1 < /dev/null &
  session_pid=$!
  print $session_pid >$PID_FILE
  print $requested >$MODE_FILE
  print -r -- "$map_yaml" >$MAP_FILE
  sleep 2
  if ! kill -0 -- -$session_pid 2>/dev/null; then
    print -u2 "HWR $requested mode exited during startup; inspect $LOG_FILE"
    rm -f $PID_FILE $MODE_FILE
    return 1
  fi
  print "started HWR $requested mode (process group $session_pid)"
}

case ${1:-status} in
  start)
    start_mode "${2:-}" "${3:-}"
    ;;
  stop)
    stop_mode
    ;;
  restart)
    stop_mode
    start_mode "${2:-}" "${3:-}"
    ;;
  status)
    if group_alive; then
      print "HWR $(<$MODE_FILE) mode is running (process group $session_pid)"
    else
      rm -f $PID_FILE $MODE_FILE
      print "no HWR autonomy mode is running"
    fi
    ;;
  *)
    print -u2 "usage: $0 start|restart mapping|localization|navigation|exploration [map path] | stop | status"
    exit 2
    ;;
esac
