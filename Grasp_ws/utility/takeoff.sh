#!/usr/bin/env bash
# Arm and take off to a hover altitude, then wait until it is reached.
#
# usage: ./takeoff.sh [altitude_m]     (default 2.5)
set -eo pipefail
source "$(dirname "$0")/common.sh"

alt="${1:-2.5}"
px4_running || die "PX4 is not running (./launch_sim.sh first)"

if px4_armed && ! px4_landed; then
  ok "already airborne at $(px4_alt) m"
  exit 0
fi

px4 param set MIS_TAKEOFF_ALT "$alt" >/dev/null
info "takeoff altitude set to ${alt} m"

# Arming is rejected until preflight checks pass (EKF converged etc.),
# so keep retrying for a while instead of failing on the first attempt.
try_arm() { px4 commander arm >/dev/null 2>&1 || true; sleep 1; px4_armed; }
wait_for 60 "armed" try_arm || {
  px4 commander check 2>&1 | tail -15
  die "arming rejected - see preflight output above"
}

px4 commander takeoff >/dev/null 2>&1 || die "takeoff command rejected"
info "taking off..."

reached() { awk -v a="$(px4_alt)" -v t="$alt" 'BEGIN { exit !(a >= 0.9 * t) }'; }
wait_for 45 "hovering at $(printf '%.1f' "$alt") m" reached || \
  die "did not reach altitude (now $(px4_alt) m) - check logs/px4.log"
ok "altitude $(px4_alt) m"
