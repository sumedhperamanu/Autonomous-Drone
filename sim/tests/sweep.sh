#!/usr/bin/env bash
# Rate-gain sweep: run the same step tests at several MC_ROLLRATE_K / MC_PITCHRATE_K values.
# Usage: bash sim/tests/sweep.sh <airframe-id> <model> k1 k2 ...    (results appended to /tmp/sweep_<model>.txt)
HERE="$(cd "$(dirname "$0")" && pwd)"; AF=$1; MODEL=$2; shift 2
for k in "$@"; do
  echo "=== $MODEL  K=$k" | tee -a /tmp/sweep_$MODEL.txt
  PLOT=/tmp/plot_${MODEL}_k$k.png bash "$HERE/run_case.sh" $AF $MODEL MC_ROLLRATE_K=$k MC_PITCHRATE_K=$k 2>&1 | grep -E "ALT|ROLL|hover|set " | tee -a /tmp/sweep_$MODEL.txt
done
