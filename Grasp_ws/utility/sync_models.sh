#!/usr/bin/env bash
# Copy this repo's simulation assets (px4/Tools/simulation/gz: Armbot, x500_gripper and the
# x500_gripper_armbot world) into the PX4 tree the simulator loads them from. Run it after
# switching branches: feature/servo-claw has the two-jaw servo claw, feature/small-lift-protocol
# the earlier force-controlled fingers.
set -euo pipefail
source "$(dirname "$0")/common.sh"
SRC="$(cd "$(dirname "$0")/../.." && pwd)/px4/Tools/simulation/gz"
DST="$PX4_DIR/Tools/simulation/gz"
for m in Armbot x500_gripper; do
  rsync -a "$SRC/models/$m/" "$DST/models/$m/"   # no --delete: keep extra files in the PX4 copy
done
cp "$SRC/worlds/x500_gripper_armbot.sdf" "$DST/worlds/x500_gripper_armbot.sdf"
info "synced Armbot, x500_gripper and the x500_gripper_armbot world into $DST"
