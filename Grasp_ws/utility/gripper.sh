#!/usr/bin/env bash
# Drive the Armbot gripper hanging under the x500 (Gazebo transport, so no
# ros_gz bridge needed). Angles in radians.
#
# usage: ./gripper.sh <command> [args]
#   open | close [N]          fingers open / squeeze with N newtons (default 1.0)
#   home                      arm forward under the belly, jaw open
#   reach                     arm straight down, jaw open
#   grab                      reach -> close -> lift to carry pose
#   carry                     arm tucked down-forward (jaw unchanged)
#   pose <yaw> <shoulder> <elbow>
#   joint <base_yaw|shoulder_pitch|elbow_pitch> <rad>
#   sweep                     swing the turntable left/right
#   demo                      full choreography: reach, grab, carry, sweep, release, home
#   state                     print current joint positions
#
# Mount note: Armbot hangs upside-down (roll = pi), so NEGATIVE shoulder/
# elbow pitch moves the gripper DOWN. Positive values swing it up into the
# airframe/props; they are clamped to MAX_UP_PITCH (0.2) unless FORCE=1.
set -eo pipefail
source "$(dirname "$0")/common.sh"

# Two-finger force-controlled gripper (see Armbot/model.sdf): the command is
# a force per finger, + closes, - opens.
GRIP_OPEN_N="${GRIP_OPEN_N:--2.0}"
GRIP_CLOSE_N="${GRIP_CLOSE_N:-1.0}"
MAX_UP_PITCH="${MAX_UP_PITCH:-0.2}"
STEP_S="${STEP_S:-1.5}"   # dwell after each move (no joint-state feedback loop)

gz_server_running || die "Gazebo world '$SIM_WORLD' is not running (./launch_sim.sh first)"

clamp_pitch() {  # clamp_pitch <name> <rad>
  if [ "${FORCE:-0}" = 1 ]; then echo "$2"; return; fi
  awk -v v="$2" -v up="$MAX_UP_PITCH" -v n="$1" 'BEGIN {
    if (v > up)     { printf "[warn] %s %.2f clamped to %.2f (FORCE=1 to override)\n", n, v, up > "/dev/stderr"; v = up }
    if (v < -1.57)  { v = -1.57 }
    print v }'
}

set_joint() {  # set_joint <joint> <rad>
  local j=$1 v=$2
  case "$j" in
    shoulder_pitch|elbow_pitch) v=$(clamp_pitch "$j" "$v") ;;
    base_yaw) ;;
    *) die "unknown joint '$j' (base_yaw|shoulder_pitch|elbow_pitch)" ;;
  esac
  arm_joint "$j" "$v"
}

pose() {  # pose <yaw> <shoulder> <elbow>
  info "pose  yaw=$1 shoulder=$2 elbow=$3"
  set_joint base_yaw "$1"
  set_joint shoulder_pitch "$2"
  set_joint elbow_pitch "$3"
  sleep "$STEP_S"
}
jaw_open()  { info "fingers open";  grip_force "$GRIP_OPEN_N"; sleep "$STEP_S"; }
jaw_close() { info "fingers close (${1:-$GRIP_CLOSE_N} N)"; grip_force "${1:-$GRIP_CLOSE_N}"; sleep "$STEP_S"; }

home()  { pose 0 0 0; jaw_open; }
reach() { pose 0 -1.45 -0.1; }
carry() { pose 0 -0.9 -0.6; }
grab()  { jaw_open; reach; jaw_close; carry; }
sweep() {
  local y
  for y in 0.9 -0.9 0.9 0; do pose "$y" -0.9 -0.6; done
}

state() {
  timeout 5 gz topic -e -n 1 -t "/model/${ARM_MODEL_NAME}/joint_states" | python3 -c '
import re, sys
s = sys.stdin.read()
for blk in s.split("joint {")[1:]:
    name = re.search(r"name: \"([^\"]+)\"", blk)
    pos = re.search(r"position: ([-\d.e]+)", blk)
    if name:
        print(f"  {name.group(1):22s} {float(pos.group(1)) if pos else 0.0:+.3f} rad")
' || die "no joint_states received"
}

cmd="${1:-}"; shift || true
case "$cmd" in
  open)  jaw_open ;;
  close) jaw_close "${1:-}" ;;
  home)  home ;;
  reach) reach ;;
  carry) carry ;;
  grab)  grab ;;
  sweep) sweep ;;
  pose)  [ $# -eq 3 ] || die "usage: $0 pose <yaw> <shoulder> <elbow>"; pose "$@" ;;
  joint) [ $# -eq 2 ] || die "usage: $0 joint <name> <rad>"; set_joint "$1" "$2"; ok "$1 -> $2" ;;
  state) state ;;
  demo)
    info "gripper demo: reach -> grab -> carry -> sweep -> release -> home"
    grab; sweep; jaw_open; home
    ok "demo finished" ;;
  ""|-h|--help) sed -n '2,20p' "$0" ;;
  *) die "unknown command '$cmd' (see $0 --help)" ;;
esac
