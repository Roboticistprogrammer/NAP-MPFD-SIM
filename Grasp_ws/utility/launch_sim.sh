#!/usr/bin/env bash
# Bring up the whole sim stack for the x500_gripper + Armbot world:
#   1. micro-XRCE-DDS agent        (PX4 uORB <-> ROS 2, /fmu/*)
#   2. PX4 SITL, which starts the Gazebo server with the world and attaches
#      to the x500_gripper model already in it (no runtime spawn)
#   3. Gazebo GUI (dGPU, free camera)
# then waits until PX4 has a valid local position estimate.
#
# usage: ./launch_sim.sh [--headless] [--fg] [--build]
#   --headless  no Gazebo GUI (much lighter on RAM/CPU)
#   --fg        run PX4 in the foreground with its interactive pxh> shell
#               (Ctrl+C stops PX4; run ./stop_all.sh afterwards)
#   --build     run `make px4_sitl` first (needed after PX4 source changes)
#
# Default runs PX4 as a daemon; its output goes to logs/px4.log and you
# can still run PX4 shell commands, e.g. ../utility/px4.sh commander status
set -eo pipefail
source "$(dirname "$0")/common.sh"

headless=0; foreground=0; build=0
for a in "$@"; do
  case "$a" in
    --headless) headless=1 ;;
    --fg)       foreground=1 ;;
    --build)    build=1 ;;
    -h|--help)  sed -n '2,17p' "$0"; exit 0 ;;
    *) die "unknown option: $a" ;;
  esac
done

# ---- preflight
if px4_running; then
  die "PX4 is already running. Use ./stop_all.sh first (or ./status.sh to inspect)."
fi
if pgrep -f "gz sim .*-s " >/dev/null && ! gz_server_running; then
  # PX4 would attach to whatever world is up and then fail to find our model.
  die "a different Gazebo world is already running. Run ./stop_all.sh first."
fi
command -v MicroXRCEAgent >/dev/null || die "MicroXRCEAgent not found (see 01_environment_setup.md)"
command -v gz >/dev/null || die "gz not found (Gazebo Harmonic not installed?)"

world_sdf="$PX4_DIR/Tools/simulation/gz/worlds/${SIM_WORLD}.sdf"
[ -f "$world_sdf" ] || die "world not found: $world_sdf"

avail_mb=$(awk '/MemAvailable/ { print int($2 / 1024) }' /proc/meminfo)
if [ "$avail_mb" -lt 2500 ]; then
  warn "only ${avail_mb} MB RAM available - Gazebo GUI may lag/ignore the mouse."
  warn "close browsers / extra VS Code windows, or use --headless."
fi

if [ "$build" = 1 ] || [ ! -x "$PX4_BIN/px4" ]; then
  info "building PX4 SITL (make px4_sitl)..."
  make -C "$PX4_DIR" px4_sitl
fi
[ -f "$PX4_BUILD_DIR/etc/init.d-posix/airframes/4022_${SIM_AIRFRAME}" ] || \
  die "airframe 4022_${SIM_AIRFRAME} not in build - run with --build"

# Fuel models must be cached or world load can time out.
"$UTIL_DIR/fetch_assets.sh" "$world_sdf" >"$LOG_DIR/fetch_assets.log" 2>&1 || \
  warn "could not pre-fetch Fuel models (offline?) - see $LOG_DIR/fetch_assets.log"

# ---- 1. XRCE-DDS agent
if agent_running; then
  ok "XRCE-DDS agent already running"
else
  nohup MicroXRCEAgent udp4 -p "$XRCE_PORT" >"$LOG_DIR/agent.log" 2>&1 &
  disown
  ok "XRCE-DDS agent started on udp/$XRCE_PORT (log: $LOG_DIR/agent.log)"
fi

# ---- 2. PX4 SITL (+ Gazebo server)
export PX4_SIM_MODEL="$SIM_AIRFRAME"
export PX4_GZ_WORLD="$SIM_WORLD"
export PX4_GZ_MODEL_NAME="$SIM_MODEL_NAME"   # attach to the model in the world
export PX4_GZ_NO_FOLLOW=1                    # never lock the GUI camera
export HEADLESS=1                            # GUI is started by start_gui.sh
export GZ_IP=127.0.0.1
use_nvidia_if_available

# `gz service -i` exits 0 even with no provider, so check its output instead.
world_ready() {
  gz_server_running && \
    gz service -i --service "/world/${SIM_WORLD}/scene/info" 2>&1 | grep -q "Service providers"
}
gui_after_server() {
  wait_for 90 "Gazebo world '$SIM_WORLD' up" world_ready || return 1
  [ "$headless" = 1 ] || "$UTIL_DIR/start_gui.sh"
}

cd "$PX4_BUILD_DIR/rootfs"

if [ "$foreground" = 1 ]; then
  info "starting PX4 in the foreground (pxh> shell)..."
  ( gui_after_server >"$LOG_DIR/gui_launch.log" 2>&1 ) &
  exec "$PX4_BIN/px4"
fi

info "starting PX4 SITL ($SIM_AIRFRAME in world $SIM_WORLD)..."
nohup "$PX4_BIN/px4" -d >"$LOG_DIR/px4.log" 2>&1 &
disown

gui_after_server || die "Gazebo did not come up - see $LOG_DIR/px4.log"
wait_for 30 "PX4 gz_bridge attached to '$SIM_MODEL_NAME'" \
  grep -q "PX4_GZ_MODEL_NAME set, PX4 will attach" "$LOG_DIR/px4.log"
grep -q "ERROR \[init\]" "$LOG_DIR/px4.log" && die "PX4 init error - see $LOG_DIR/px4.log"

# The grasp experiment switches EKF2 to emulated mocap and restores it at
# the end; SITL saves parameters, so a crashed session can leave PX4 with
# GPS/baro fusion off (no position estimate without the experiment running).
# (vehicle_status is published by commander, i.e. after the saved params loaded)
px4_booted() { [ -n "$(px4_field vehicle_status arming_state)" ]; }
wait_for 30 "PX4 booted" px4_booted || true
if [ "$(px4_param EKF2_GPS_CTRL)" = "0" ] && [ "$(px4_param EKF2_EV_CTRL)" != "0" ]; then
  warn "PX4 was left in grasp-experiment (mocap) EKF mode - restoring GPS defaults"
  for p in EKF2_EV_CTRL EKF2_HGT_REF EKF2_GPS_CTRL EKF2_BARO_CTRL EKF2_EV_NOISE_MD \
           EKF2_EVP_NOISE EKF2_EVA_NOISE EKF2_EV_DELAY NAV_DLL_ACT COM_RCL_EXCEPT \
           SDLOG_PROFILE SDLOG_ROTATE SDLOG_MAX_SIZE; do
    px4 param reset "$p" >/dev/null 2>&1
  done
  px4 ekf2 stop >/dev/null 2>&1; sleep 1; px4 ekf2 start >/dev/null 2>&1
fi

ekf_ready() { [ "$(px4_field vehicle_local_position xy_valid)" = "True" ] && \
              [ "$(px4_field vehicle_local_position z_valid)" = "True" ]; }
wait_for 90 "PX4 EKF local position valid" ekf_ready || \
  warn "EKF not ready yet; takeoff.sh will keep waiting for preflight checks"
wait_for 20 "XRCE-DDS client connected" \
  bash -c "'$PX4_BIN/px4-uxrce_dds_client' status 2>&1 | grep -q 'Running, connected'" || true

echo
ok "sim stack is up."
info "next: ./takeoff.sh   then   ./gripper.sh demo   (or ./demo.sh for everything)"
