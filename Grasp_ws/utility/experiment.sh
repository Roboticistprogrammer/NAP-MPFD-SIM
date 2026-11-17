#!/usr/bin/env bash
# Two-scenario grasp experiment: A centred / B off-centre (1/4 length) /
# E empty-gripper control, on the cylinder lying in its jig on the table.
# Settings: src/px4_gripper_missions/config/experiment.yaml
#
# usage: ./experiment.sh <command> [options]
#   single A|B|E               one trial (quick check / debugging)
#   tune A|B --grip F1 F2 ...  sweep grip force [N]; pick the level giving
#            [--trials N]      30-60 % failures, put it in experiment.yaml
#   protocol [--batteries N]   batteries of 10: A/B interleaved + 1 E each
#   export <session dir>       slice the PX4 ULog per trial, merge with ground truth
#   dataset <session dir>      small_lift sessions: 50 Hz SL/CG windows + labels (dataset/)
#   relabel <session dir>      recompute outcomes with the current thresholds
# options: --grip A=0.8 B=2.0 (single/protocol overrides)  --seed N
#          --variant legacy|small_lift (default: protocol.variant)  --empty N (E per battery)
#          --no-land  --config FILE  --debug (alignment trace)
#
# The sim must be running (./launch_sim.sh, --headless recommended for long
# runs). Output: Grasp_ws/sim_data/<timestamp>_<mode>/ (see README.md).
set -eo pipefail
source "$(dirname "$0")/common.sh"

PKG_SRC="$WS_DIR/src/px4_gripper_missions"
cmd="${1:-}"; shift || true
[ -n "$cmd" ] || { sed -n '2,18p' "$0"; exit 1; }

source_ros
# Run the package from source, so edits to the code/config take effect
# without a colcon rebuild.
export PYTHONPATH="$PKG_SRC${PYTHONPATH:+:$PYTHONPATH}"

if [ "$cmd" = "relabel" ]; then
  [ -n "${1:-}" ] || die "usage: $0 relabel <session dir>"
  PYTHONNOUSERSITE=1 exec python3 -m px4_gripper_missions.experiment relabel "$1" \
    --config "$PKG_SRC/config/experiment.yaml"
fi

if [ "$cmd" = "export" ]; then
  [ -n "${1:-}" ] || die "usage: $0 export <session dir>"
  # pyulog lives in the user site-packages, so no PYTHONNOUSERSITE here.
  exec python3 -m px4_gripper_missions.export_trials "$@"
fi

if [ "$cmd" = "dataset" ]; then
  [ -n "${1:-}" ] || die "usage: $0 dataset <session dir>"
  exec python3 -m px4_gripper_missions.windows "$@"
fi

global=(--config "$PKG_SRC/config/experiment.yaml" --output-dir "$WS_DIR/sim_data")
sub=()
case "$cmd" in
  single|tune)
    scen="${1:-}"; shift || true
    [ -n "$scen" ] || die "usage: $0 $cmd <scenario> ...   (single: A|B|E, tune: A|B)"
    sub=(--scenario "$scen") ;;
  protocol) ;;
  -h|--help) sed -n '2,18p' "$0"; exit 0 ;;
  *) die "unknown command '$cmd' (single | tune | protocol | export | dataset | relabel)" ;;
esac

while [ $# -gt 0 ]; do
  case "$1" in
    --grip) shift; grip=()
            while [ $# -gt 0 ] && [ "${1#--}" = "$1" ]; do grip+=("$1"); shift; done
            global+=(--grip "${grip[@]}"); continue ;;
    --seed|--config|--variant) global+=("$1" "$2"); shift ;;
    --no-land) global+=("$1") ;;
    --trials|--batteries|--empty) sub+=("$1" "$2"); shift ;;
    --debug) export EXP_DEBUG=1 ;;
    *) die "unknown option: $1" ;;
  esac
  shift
done

px4_running && gz_server_running || die "sim not running - start it with ./launch_sim.sh --headless"
if px4_armed; then
  die "vehicle is armed - land it first (./land.sh); the experiment takes off itself"
fi

info "grasp experiment: $cmd ${sub[*]}"
# PYTHONNOUSERSITE: the distro gz-msgs bindings need the distro protobuf.
PYTHONNOUSERSITE=1 exec python3 -u -m px4_gripper_missions.experiment \
  "$cmd" "${sub[@]}" "${global[@]}"
