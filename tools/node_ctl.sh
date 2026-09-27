#!/usr/bin/env bash
# Start/stop/restart harness nodes on the PC as detached processes.
#   tools/node_ctl.sh arm|camera start|stop|restart|status
set -e
source "$HOME/bunny-harness-dev/rosenv.sh"
case "$1" in
  arm)    PKG=harness_arm;    LAUNCH=piper.launch.py;  PAT='harness_arm/piper_nod[e]';    LOG=$HOME/piper.log ;;
  camera) PKG=harness_vision; LAUNCH=camera.launch.py; PAT='harness_vision/tag_nod[e]';   LOG=$HOME/tag_node.log ;;
  *) echo "usage: $0 arm|camera start|stop|restart|status"; exit 2 ;;
esac
stop() { pkill -f "$PAT" 2>/dev/null || true; pkill -f "ros2 launch $PKG" 2>/dev/null || true; sleep 1.5; }
start() { (setsid nohup ros2 launch "$PKG" "$LAUNCH" > "$LOG" 2>&1 < /dev/null &); sleep 5; }
status() { if pgrep -f "$PAT" > /dev/null; then echo "$1: running (pid $(pgrep -f "$PAT" | head -1))"; else echo "$1: stopped"; fi; }
case "$2" in
  start) start; status "$1" ;;
  stop) stop; status "$1" ;;
  restart) stop; start; status "$1" ;;
  status) status "$1" ;;
  *) echo "usage: $0 arm|camera start|stop|restart|status"; exit 2 ;;
esac
