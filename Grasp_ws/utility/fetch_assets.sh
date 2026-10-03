#!/usr/bin/env bash
# Pre-download every Gazebo Fuel model referenced by the sim world into
# ~/.gz/fuel, so world load never stalls on the network (PX4 only waits
# ~30 s for the world to come up). Safe to re-run; cached models are skipped.
#
# usage: ./fetch_assets.sh [world.sdf]
set -eo pipefail
source "$(dirname "$0")/common.sh"

world_sdf="${1:-$PX4_DIR/Tools/simulation/gz/worlds/${SIM_WORLD}.sdf}"
[ -f "$world_sdf" ] || die "world not found: $world_sdf"

mapfile -t uris < <(grep -o '<uri>https://fuel[^<]*</uri>' "$world_sdf" \
  | sed -e 's|<uri>||' -e 's|</uri>||' | sort -u)

if [ "${#uris[@]}" -eq 0 ]; then
  ok "no Fuel models referenced in $(basename "$world_sdf")"
  exit 0
fi

for uri in "${uris[@]}"; do
  info "fetching: $uri"
  gz fuel download -u "$uri" || die "download failed: $uri"
done
ok "${#uris[@]} Fuel model(s) cached in ~/.gz/fuel"
