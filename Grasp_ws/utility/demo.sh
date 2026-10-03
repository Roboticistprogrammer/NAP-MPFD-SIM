#!/usr/bin/env bash
# End-to-end demo: bring up the sim (if not already up), arm + take off,
# run the gripper choreography while hovering, then land.
#
# usage: ./demo.sh [--headless] [--no-land] [--alt <m>]
set -eo pipefail
source "$(dirname "$0")/common.sh"

launch_args=(); land=1; alt=2.5
while [ $# -gt 0 ]; do
  case "$1" in
    --headless) launch_args+=(--headless) ;;
    --no-land)  land=0 ;;
    --alt)      alt="$2"; shift ;;
    -h|--help)  sed -n '2,5p' "$0"; exit 0 ;;
    *) die "unknown option: $1" ;;
  esac
  shift
done

if px4_running; then
  ok "sim already running - reusing it"
else
  "$UTIL_DIR/launch_sim.sh" "${launch_args[@]}"
  # let the GUI finish loading so the takeoff is actually visible
  [ ${#launch_args[@]} -eq 0 ] && sleep 5
fi

echo; info "=== 1/3 arm + takeoff ==="
"$UTIL_DIR/takeoff.sh" "$alt"
sleep 2

echo; info "=== 2/3 gripper choreography ==="
"$UTIL_DIR/gripper.sh" demo

if [ "$land" = 1 ]; then
  echo; info "=== 3/3 land ==="
  "$UTIL_DIR/land.sh"
else
  info "staying airborne (./land.sh when done)"
fi
ok "demo complete. ./stop_all.sh to shut everything down."
