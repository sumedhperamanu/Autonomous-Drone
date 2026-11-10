#!/usr/bin/env bash
# Copies both drones (Gazebo models + PX4 airframes) into a PX4 v1.17 checkout.
#   bash sim/install.sh                      looks in $PX4_DIR, ~/PX4-Autopilot, then /opt/PX4-Autopilot
#   bash sim/install.sh /path/to/PX4-Autopilot
# Run it again whenever you pull changes to sim/. It only overwrites our own files.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"

PX4=""
for d in "${1:-}" "${PX4_DIR:-}" "$HOME/PX4-Autopilot" /opt/PX4-Autopilot; do
  if [ -n "$d" ] && [ -f "$d/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt" ]; then PX4=$d; break; fi
done
[ -n "$PX4" ] || { echo "Couldn't find PX4. Give the path: bash sim/install.sh /path/to/PX4-Autopilot"; exit 1; }

MODELS=$PX4/Tools/simulation/gz/models
AIRFRAMES=$PX4/ROMFS/px4fmu_common/init.d-posix/airframes

ver=$(git -C "$PX4" describe --tags 2>/dev/null || echo "unknown")
case "$ver" in v1.17*) ;; *) echo "Heads up: this PX4 is $ver. These files were made for v1.17.0." ;; esac

for drone in fiveinch seveninch; do
  cp -r "$HERE/$drone/models/"* "$MODELS/"
  for f in "$HERE/$drone/airframes/"*; do
    name=$(basename "$f")
    cp "$f" "$AIRFRAMES/$name"
    sed -i 's/\r$//' "$AIRFRAMES/$name"     # in case a Windows checkout added CRLF line endings
    grep -q "$name" "$AIRFRAMES/CMakeLists.txt" || sed -i "/Reserve for custom models/a\\	$name" "$AIRFRAMES/CMakeLists.txt"
  done
done

# Handy launcher: bash ~/run_sim.sh [target] [world] starts a sim and links it to QGroundControl
cp "$HERE/run.sh" "$HOME/run_sim.sh"
# Wind: python3 ~/wind.py typical (in a second terminal while a sim is running)
cp "$HERE/wind.py" "$HOME/wind.py"

echo "Installed into $PX4"
echo "Now run one of these (first build takes 5 to 10 min), or use: bash ~/run_sim.sh gz_seveninch_4s"
echo "  cd $PX4 && make px4_sitl gz_seveninch_4s"
echo "  cd $PX4 && make px4_sitl gz_fiveinch"
