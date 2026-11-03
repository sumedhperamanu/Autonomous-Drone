#!/usr/bin/env bash
# One full scripted test: start sim, wait until ready, fly step_tests.py, stop sim, analyse the log.
# Usage: bash sim/tests/run_case.sh <airframe-id> <model> [NAME=VALUE ...]
#   e.g. bash sim/tests/run_case.sh 22054 gz_seveninch_4s
# Options as env vars: PITCH=1 adds a pitch step, PLOT=file.png saves a plot.
# Stops any sim that is already running.
set -uo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
# PX4 checkout: $PX4_DIR if set, else ~/PX4-Autopilot, else /opt/PX4-Autopilot (the repo's dev container)
if [ -z "${PX4_DIR:-}" ]; then
  for d in "$HOME/PX4-Autopilot" /opt/PX4-Autopilot; do [ -d "$d/build/px4_sitl_default" ] && PX4_DIR=$d && break; done
fi
[ -n "${PX4_DIR:-}" ] || { echo "PX4 build not found. Build it once (make px4_sitl gz_fiveinch) or set PX4_DIR=..."; exit 1; }
export PX4_DIR
PY=${PY:-python3}
AF=$1; MODEL=$2; shift 2
pkill -x px4; pkill -x ruby; sleep 2
bash "$HERE/sim_headless.sh" "$AF" "$MODEL" >/dev/null
for i in $(seq 1 90); do grep -q "Ready for takeoff" "/tmp/px4_$MODEL.log" && break; sleep 1; done
grep -q "Ready for takeoff" "/tmp/px4_$MODEL.log" || { echo "sim did not become ready"; tail -20 "/tmp/px4_$MODEL.log"; exit 1; }
ARGS=(); for kv in "$@"; do ARGS+=(--param "$kv"); done
timeout 170 $PY "$HERE/step_tests.py" ${PITCH:+--pitch} "${ARGS[@]}" | grep -E "set |hover thrust|landed" 
LOG=$(ls -t "$PX4_DIR"/build/px4_sitl_default/rootfs/log/*/*.ulg | head -1)
pkill -x px4; pkill -x ruby
$PY "$HERE/analyze_steps.py" "$LOG" ${PLOT:+--plot "$PLOT"}
