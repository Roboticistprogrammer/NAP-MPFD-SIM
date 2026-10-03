#!/usr/bin/env bash
# Land in place and wait for auto-disarm.
#
# usage: ./land.sh
set -eo pipefail
source "$(dirname "$0")/common.sh"

px4_running || die "PX4 is not running"
if ! px4_armed; then
  ok "already disarmed"
  exit 0
fi

px4 commander land >/dev/null 2>&1 || die "land command rejected"
info "landing from $(px4_alt) m..."
wait_for 60 "landed" px4_landed || die "still airborne at $(px4_alt) m"
disarmed() { ! px4_armed; }
wait_for 15 "disarmed" disarmed || px4 commander disarm >/dev/null 2>&1 || true
