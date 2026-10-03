#!/usr/bin/env bash
# (Re)start the Gazebo GUI client against the running sim server.
# Safe to run any time, e.g. if the GUI freezes or was closed.
#
# The GUI is started here rather than by PX4 because PX4's launcher would
# (a) render on the iGPU on PRIME laptops and (b) send /gui/track FOLLOW,
# which locks the camera to the drone so the mouse can't orbit/pan.
#
# usage: ./start_gui.sh            (USE_NVIDIA=0 to force the default GPU)
set -eo pipefail
source "$(dirname "$0")/common.sh"

gz_server_running || die "no Gazebo server for '$SIM_WORLD' is running (start ./launch_sim.sh first)"

if gz_gui_running; then
  info "stopping existing Gazebo GUI"
  pkill -f "gz sim -g" || true
  sleep 1
fi

source_gz_env
use_nvidia_if_available
nohup gz sim -g >"$LOG_DIR/gz_gui.log" 2>&1 &
disown
ok "Gazebo GUI started (log: $LOG_DIR/gz_gui.log)"
[ -n "${__NV_PRIME_RENDER_OFFLOAD:-}" ] && info "rendering on NVIDIA dGPU (PRIME offload)"
info "camera: left-drag = pan, right-drag/scroll = zoom, shift+left-drag = orbit"
