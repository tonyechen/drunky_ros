#!/usr/bin/env bash
# One command to run the whole bartender setup:
#   - bimanual bringup (MoveIt, both arms, cameras, overhead YOLO, servers)
#   - the bartender GUI, which launches/owns the left & right wrist YOLOs and
#     drives the drinks.
# Bringup runs in the background (logged to /tmp/bartender_bringup.log); the GUI
# runs in the foreground. Closing the GUI window shuts everything down.

# Note: no `set -u` — the ROS setup.bash scripts reference unset vars.
set -eo pipefail

WS="$HOME/techin517/ros2_ws"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

BRINGUP_LOG=/tmp/bartender_bringup.log

cleanup() {
  echo "Shutting down bringup..."
  [[ -n "${BRINGUP_PID:-}" ]] && kill -INT "-$BRINGUP_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "Starting bringup (log: $BRINGUP_LOG)..."
setsid ros2 launch soa_bringup go_to_drink_bi.launch.py > "$BRINGUP_LOG" 2>&1 &
BRINGUP_PID=$!

# Wait until the wrist cameras are publishing before opening the GUI.
echo "Waiting for cameras to come up..."
for i in $(seq 1 60); do
  if ros2 topic list 2>/dev/null | grep -q '/left_follower/image_raw'; then
    break
  fi
  sleep 1
done
sleep 2

echo "Launching bartender web UI..."
# The bridge auto-picks a free port (8088+) and writes it here; open that.
rm -f /tmp/bartender_web_port
(
  for _ in $(seq 1 30); do [ -f /tmp/bartender_web_port ] && break; sleep 0.5; done
  uiport=$(cat /tmp/bartender_web_port 2>/dev/null || echo 8088)
  echo "Bartender UI at http://localhost:${uiport}"
  xdg-open "http://localhost:${uiport}" >/dev/null 2>&1 || true
) &
ros2 run soa_apps bartender_web
