#!/usr/bin/env bash
# Fly the 7" (4S) step test three times: no wind, typical Waterloo wind and windy, then draw each wind run
# over the no-wind run. Graphs: altitude, roll step, pitch step, and how far the drone gets pushed off its spot.
# Usage: bash sim/tests/wind_plots.sh [output folder]      (default ~/plots-wind)
#        CASES=windy bash sim/tests/wind_plots.sh           only redo some of them (nowind, typical, windy)
# Takes about 5 minutes. Results: the PNGs, the three logs, and summary.txt with the numbers.
# A flight that fails (for example the drone never takes off) is tried once more.
set -uo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
OUT=${1:-$HOME/plots-wind}; mkdir -p "$OUT"
[ -n "${CASES:-}" ] || : > "$OUT/summary.txt"
if [ -z "${PX4_DIR:-}" ]; then
  for d in "$HOME/PX4-Autopilot" /opt/PX4-Autopilot; do [ -d "$d/build/px4_sitl_default" ] && PX4_DIR=$d && break; done
fi
export PX4_DIR
PY=${PY:-python3}
AF=${AF:-22054}; MODEL=${MODEL:-gz_seveninch_4s}
for case in ${CASES:-nowind typical windy}; do
  W=$case; [ $case = nowind ] && W=
  ok=
  for attempt in 1 2; do
    echo "== $case (try $attempt)" | tee -a "$OUT/summary.txt"
    WIND="$W" PITCH=1 bash "$HERE/run_case.sh" "$AF" "$MODEL" 2>&1 | tee -a "$OUT/summary.txt"
    [ "${PIPESTATUS[0]}" = 0 ] && { ok=1; break; }
  done
  [ -n "$ok" ] || { echo "the $case flight failed twice, stopping"; exit 1; }
  cp "$(ls -t "$PX4_DIR"/build/px4_sitl_default/rootfs/log/*/*.ulg | head -1)" "$OUT/7in_4s_$case.ulg"
done
if ! $PY -c "import matplotlib.pyplot" 2>/dev/null; then
  echo "Graphs skipped: matplotlib doesn't work here. Copy the .ulg files out and run analyze_steps.py where it does."
elif [ -f "$OUT/7in_4s_nowind.ulg" ]; then
  $PY "$HERE/analyze_steps.py" "$OUT/7in_4s_nowind.ulg" --label 'no wind' --plot "$OUT/7in_4s_nowind.png" \
    --title '7" on 4S, no wind' > /dev/null
  [ -f "$OUT/7in_4s_typical.ulg" ] && $PY "$HERE/analyze_steps.py" "$OUT/7in_4s_typical.ulg" --calm "$OUT/7in_4s_nowind.ulg" \
    --label 'typical wind' --plot "$OUT/7in_4s_typical.png" \
    --title '7" on 4S: typical Waterloo wind (5 m/s, gusts to about 8) vs no wind' > /dev/null
  [ -f "$OUT/7in_4s_windy.ulg" ] && $PY "$HERE/analyze_steps.py" "$OUT/7in_4s_windy.ulg" --calm "$OUT/7in_4s_nowind.ulg" \
    --label 'windy' --plot "$OUT/7in_4s_windy.png" \
    --title '7" on 4S: windy day (9 m/s, gusts to about 15) vs no wind' > /dev/null
fi
ls "$OUT"
echo "done: results in $OUT"
