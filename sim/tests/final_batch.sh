#!/usr/bin/env bash
# Day 4 final check: every shipped airframe at its baseline gains (with a pitch step), plus edge checks.
HERE="$(cd "$(dirname "$0")" && pwd)"; OUT=${OUT:-/tmp/final_batch.txt}
run() { label=$1; shift; echo "=== $label" | tee -a $OUT; bash "$HERE/run_case.sh" "$@" 2>&1 | grep -E "ALT|ROLL|PITCH|hover|rate tracking|set " | tee -a $OUT; }
PITCH=1 PLOT=/tmp/plot_final_5in.png run "5in baseline (K 0.4, computed inertia)" 22050 gz_fiveinch
run "5in edge check K 0.7 (computed inertia)" 22050 gz_fiveinch MC_ROLLRATE_K=0.7 MC_PITCHRATE_K=0.7
run "7in 6S edge check K 0.85" 22052 gz_seveninch MC_ROLLRATE_K=0.85 MC_PITCHRATE_K=0.85
PITCH=1 PLOT=/tmp/plot_final_7in6s.png run "7in 6S baseline (K 0.5)" 22052 gz_seveninch
run "7in 4S K 0.7" 22054 gz_seveninch_4s MC_ROLLRATE_K=0.7 MC_PITCHRATE_K=0.7
run "7in 4S K 1.0" 22054 gz_seveninch_4s MC_ROLLRATE_K=1.0 MC_PITCHRATE_K=1.0
PITCH=1 PLOT=/tmp/plot_final_7in4s.png run "7in 4S baseline (K 0.5)" 22054 gz_seveninch_4s
echo BATCH DONE | tee -a $OUT
