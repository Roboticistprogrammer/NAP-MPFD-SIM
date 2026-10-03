#!/usr/bin/env bash
# Install this project's PX4-side files into a PX4-Autopilot checkout:
#   Tools/simulation/gz/models/{Armbot,x500_gripper}
#   Tools/simulation/gz/worlds/x500_gripper_armbot.sdf
#   ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_x500_gripper
#   + registers the airframe in that directory's CMakeLists.txt
#
# usage: px4/install.sh [--link] [PX4_DIR]     (default PX4_DIR: ~/PX4-Autopilot)
#   --link   symlink instead of copy, so edits in this repo apply directly
# Existing files that differ are kept as <name>.bak. Rebuild afterwards:
#   make -C ~/PX4-Autopilot px4_sitl
set -eo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
mode=copy
if [ "${1:-}" = "--link" ]; then mode=link; shift; fi
px4="${1:-${PX4_DIR:-$HOME/PX4-Autopilot}}"
[ -f "$px4/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt" ] || {
  echo "not a PX4-Autopilot checkout: $px4" >&2; exit 1; }

place() {  # place <path relative to PX4 root>
  local rel=$1 src="$here/$1" dst="$px4/$1"
  if [ -e "$dst" ] && [ ! -L "$dst" ] && ! diff -rq "$src" "$dst" >/dev/null 2>&1; then
    rm -rf "$dst.bak"; mv "$dst" "$dst.bak"; echo "  backed up $rel -> $rel.bak"
  fi
  rm -rf "$dst"
  mkdir -p "$(dirname "$dst")"
  if [ "$mode" = link ]; then ln -s "$src" "$dst"; else cp -r "$src" "$dst"; fi
  echo "  $mode $rel"
}

place Tools/simulation/gz/models/Armbot
place Tools/simulation/gz/models/x500_gripper
place Tools/simulation/gz/worlds/x500_gripper_armbot.sdf
place ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_x500_gripper

cm="$px4/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt"
if grep -q "4022_gz_x500_gripper" "$cm"; then
  echo "  airframe already registered in CMakeLists.txt"
else
  sed -i 's/^\(\s*\)4021_gz_x500_flow$/&\n\14022_gz_x500_gripper/' "$cm"
  grep -q "4022_gz_x500_gripper" "$cm" || { echo "could not register the airframe - add 4022_gz_x500_gripper to $cm by hand (see airframes_CMakeLists.patch)" >&2; exit 1; }
  echo "  registered 4022_gz_x500_gripper in CMakeLists.txt"
fi
echo "done - now rebuild: make -C \"$px4\" px4_sitl"
