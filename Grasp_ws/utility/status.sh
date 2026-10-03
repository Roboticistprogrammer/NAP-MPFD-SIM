#!/usr/bin/env bash
# One-shot health check of the sim stack.
#
# usage: ./status.sh
set -o pipefail
source "$(dirname "$0")/common.sh"

row() { printf '  %-22s %s\n' "$1" "$2"; }
yn()  { if "$@" >/dev/null 2>&1; then echo "${_c_ok}running${_c_off}"; else echo "${_c_err}down${_c_off}"; fi; }

echo "Processes"
row "XRCE-DDS agent"  "$(yn agent_running)"
row "Gazebo server"   "$(yn gz_server_running)"
row "Gazebo GUI"      "$(yn gz_gui_running)"
row "PX4 SITL"        "$(yn px4_running)"
row "ros_gz bridge"   "$(yn is_running parameter_bridge)"

if px4_running; then
  echo "Vehicle"
  row "armed"         "$(px4_armed && echo yes || echo no)"
  row "landed"        "$(px4_field vehicle_land_detected landed)"
  row "nav_state"     "$(px4_field vehicle_status nav_state)"
  row "altitude [m]"  "$(px4_alt)"
  row "EKF xy/z valid" "$(px4_field vehicle_local_position xy_valid)/$(px4_field vehicle_local_position z_valid)"
  row "DDS client"    "$("$PX4_BIN/px4-uxrce_dds_client" status 2>&1 | grep -o 'Running, connected\|Running, disconnected' | head -1)"
fi

echo "Resources"
row "RAM available"   "$(awk '/MemAvailable/ { printf "%d MB", $2/1024 }' /proc/meminfo)"
row "swap used"       "$(awk '/SwapTotal/ {t=$2} /SwapFree/ {f=$2} END { if (t) printf "%d%%", 100*(t-f)/t; else print "n/a" }' /proc/meminfo)"
if command -v nvidia-smi >/dev/null; then
  row "gz on NVIDIA"  "$(nvidia-smi 2>/dev/null | grep -q -E 'ruby|gz' && echo yes || echo no)"
fi
