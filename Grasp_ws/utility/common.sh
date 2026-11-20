#!/usr/bin/env bash
# Shared settings + helpers for the utility scripts. Sourced, not executed.
# Every value can be overridden from the environment, e.g.
#   PX4_DIR=~/src/PX4-Autopilot ./launch_sim.sh

UTIL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$UTIL_DIR/.." && pwd)"

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_BUILD_DIR="$PX4_DIR/build/px4_sitl_default"
PX4_BIN="$PX4_BUILD_DIR/bin"

# The combined x500_gripper + Armbot world. PX4 attaches to the model that
# the world already contains (no runtime spawn), see the world's header.
SIM_WORLD="${SIM_WORLD:-x500_gripper_armbot}"
SIM_AIRFRAME="${SIM_AIRFRAME:-gz_x500_gripper}"
SIM_MODEL_NAME="${SIM_MODEL_NAME:-x500_gripper}"
ARM_MODEL_NAME="${ARM_MODEL_NAME:-armbot}"

# Project models (grasp cylinder, ...) referenced by the world as model://<dir>.
# Exported here so PX4 -> gz server and the GUI both inherit it.
case ":${GZ_SIM_RESOURCE_PATH:-}:" in
  *":$WS_DIR/model:"*) ;;
  *) export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:+$GZ_SIM_RESOURCE_PATH:}$WS_DIR/model" ;;
esac

# Gazebo transport on localhost for every process (server, GUI, experiment): with a VPN
# interface up (e.g. tailscale) gz may otherwise bind publishers to it and some
# subscribers never receive.
export GZ_IP="${GZ_IP:-127.0.0.1}"

XRCE_PORT="${XRCE_PORT:-8888}"
LOG_DIR="${LOG_DIR:-$UTIL_DIR/logs}"
mkdir -p "$LOG_DIR"

# ---------------------------------------------------------------- output
if [ -t 1 ]; then
  _c_info=$'\e[1;34m'; _c_ok=$'\e[1;32m'; _c_warn=$'\e[1;33m'; _c_err=$'\e[1;31m'; _c_off=$'\e[0m'
else
  _c_info=; _c_ok=; _c_warn=; _c_err=; _c_off=
fi
info() { echo "${_c_info}[info]${_c_off} $*"; }
ok()   { echo "${_c_ok}[ ok ]${_c_off} $*"; }
warn() { echo "${_c_warn}[warn]${_c_off} $*" >&2; }
die()  { echo "${_c_err}[fail]${_c_off} $*" >&2; exit 1; }

# ---------------------------------------------------------------- env
source_ros() {
  # shellcheck disable=SC1091
  [ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  [ -f "$WS_DIR/install/setup.bash" ] && source "$WS_DIR/install/setup.bash"
  return 0
}

# Run Gazebo on the NVIDIA dGPU when the laptop is in PRIME on-demand mode
# (otherwise the GUI renders on the iGPU and becomes sluggish/unresponsive).
# Opt out with USE_NVIDIA=0.
use_nvidia_if_available() {
  [ "${USE_NVIDIA:-1}" = "1" ] || return 0
  command -v nvidia-smi >/dev/null 2>&1 || return 0
  nvidia-smi >/dev/null 2>&1 || return 0
  export __NV_PRIME_RENDER_OFFLOAD=1
  export __GLX_VENDOR_LIBRARY_NAME=nvidia
  export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
}

# Same GZ_SIM_RESOURCE_PATH / plugin path / server config PX4 uses.
source_gz_env() {
  # shellcheck disable=SC1091
  [ -f "$PX4_BUILD_DIR/rootfs/gz_env.sh" ] && source "$PX4_BUILD_DIR/rootfs/gz_env.sh"
  export GZ_IP=127.0.0.1
}

# ---------------------------------------------------------------- state
is_running() { pgrep -f "$1" >/dev/null 2>&1; }
agent_running() { is_running "MicroXRCEAgent udp4 -p $XRCE_PORT"; }
# anchored so the px4-<module> client binaries don't count as the server
px4_running()   { is_running "$PX4_BIN/px4( |$)"; }
gz_server_running() { is_running "gz sim .*-s .*${SIM_WORLD}.sdf"; }
gz_gui_running()    { is_running "gz sim -g"; }

# wait_for <timeout_s> <description> <command...>
wait_for() {
  local timeout=$1 what=$2; shift 2
  local t=0
  until "$@" >/dev/null 2>&1; do
    sleep 1; t=$((t + 1))
    if [ "$t" -ge "$timeout" ]; then
      warn "timed out after ${timeout}s waiting for: $what"
      return 1
    fi
  done
  ok "$what (${t}s)"
}

# ---------------------------------------------------------------- PX4
# Run a PX4 shell command against the running instance, e.g. px4 commander status
px4() {
  local mod=$1; shift
  "$PX4_BIN/px4-$mod" "$@"
}

# Print one field of the latest message on a uORB topic, e.g.
#   px4_field vehicle_status arming_state
px4_field() {
  px4 listener "$1" -n 1 2>/dev/null | awk -v f="$2" '$1 == f":" { print $2; exit }'
}

# Current value of a PX4 parameter, e.g. px4_param EKF2_GPS_CTRL
px4_param() {
  px4 param show "$1" 2>/dev/null | awk -v n="$1" '$0 ~ n" \\[" { print $NF; exit }'
}

px4_armed()  { [ "$(px4_field vehicle_status arming_state)" = "2" ]; }
# -z is NED, so altitude above home = -z
px4_alt()    { px4_field vehicle_local_position z | awk '{ printf "%.2f", -$1 }'; }
px4_landed() { [ "$(px4_field vehicle_land_detected landed)" = "True" ]; }

# ---------------------------------------------------------------- arm
# Grip force [N] on both fingers of the Armbot gripper (+ closes, - opens).
grip_force() {
  local f=$1 j
  for j in finger_left_joint finger_right_joint; do
    gz topic -t "/model/${ARM_MODEL_NAME}/joint/${j}/cmd_force" -m gz.msgs.Double -p "data: ${f}"
  done
}

# Publish a joint position (rad) to Armbot's JointPositionController.
arm_joint() {
  local joint=$1 pos=$2
  gz topic -t "/model/${ARM_MODEL_NAME}/joint/${joint}_joint/cmd_pos" \
    -m gz.msgs.Double -p "data: ${pos}"
}
