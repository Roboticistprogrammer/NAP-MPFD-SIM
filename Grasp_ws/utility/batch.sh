#!/usr/bin/env bash
# Main small-lift batch in several sessions, restarting the sim stack before each one
# (a long-running PX4 can drift into an estimator fault: phantom speed, "High Accelerometer Bias").
# usage: ./batch.sh "4 4 4 3" [first_seed]     # batteries per session; 10 trials each (A/B + 1 E)
set -uo pipefail
cd "$(dirname "$0")"
plan=(${1:?usage: $0 "4 4 4 3" [first_seed]})
seed=${2:-101}
for n in "${plan[@]}"; do
  echo "[batch] === session: $n batteries, seed $seed ==="
  ./stop_all.sh >/dev/null 2>&1; sleep 5
  ./launch_sim.sh --headless >/dev/null 2>&1 || { echo "[batch] ABORT: sim launch failed"; exit 1; }
  ./experiment.sh protocol --variant small_lift --batteries "$n" --seed "$seed" || echo "[batch] session exited with an error"
  seed=$((seed+1))
done
./stop_all.sh >/dev/null 2>&1
echo "[batch] done"
