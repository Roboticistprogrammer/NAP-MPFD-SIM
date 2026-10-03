#!/usr/bin/env bash
# Stop everything launch_sim.sh started (PX4, Gazebo server + GUI, XRCE-DDS
# agent, ros_gz bridge). Also cleans up a PX4 started with `make px4_sitl ...`.
#
# usage: ./stop_all.sh [--keep-agent]
set -o pipefail
source "$(dirname "$0")/common.sh"

keep_agent=0
[ "${1:-}" = "--keep-agent" ] && keep_agent=1

stop() {  # stop <pattern> <label>
  if pgrep -f "$1" >/dev/null; then
    pkill -INT -f "$1"
    for _ in 1 2 3 4 5; do pgrep -f "$1" >/dev/null || break; sleep 1; done
    pgrep -f "$1" >/dev/null && pkill -KILL -f "$1"
    ok "stopped $2"
  fi
}

# PX4 first so it doesn't log a flood of lost-sim errors.
if px4_running; then
  px4 shutdown >/dev/null 2>&1 || true
  sleep 1
  px4_running || ok "stopped PX4 SITL"
fi
stop "$PX4_BIN/px4( |$)" "PX4 SITL"
stop "make px4_sitl" "make wrapper"
stop "gz sim -g" "Gazebo GUI"
stop "gz sim .*-s " "Gazebo server"
stop "parameter_bridge" "ros_gz bridge"
[ "$keep_agent" = 1 ] || stop "MicroXRCEAgent udp4" "XRCE-DDS agent"
ok "all stopped"
