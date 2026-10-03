#!/usr/bin/env bash
# Run a PX4 shell (pxh) command against the running SITL instance, from any
# terminal - works with the daemon started by launch_sim.sh.
#
# usage: ./px4.sh <module> [args...]
#   ./px4.sh commander status
#   ./px4.sh listener vehicle_local_position
#   ./px4.sh param set MIS_TAKEOFF_ALT 3
set -eo pipefail
source "$(dirname "$0")/common.sh"

[ $# -ge 1 ] || { sed -n '2,8p' "$0"; exit 1; }
px4_running || die "PX4 is not running"
px4 "$@"
