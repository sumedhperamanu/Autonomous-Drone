#!/usr/bin/env bash
# Copies P1's drones, then the perception model, airframe and test world, into a PX4 v1.17 checkout.
#   bash sim/perception/install.sh                      looks in $PX4_DIR, ~/PX4-Autopilot, then /opt/PX4-Autopilot
#   bash sim/perception/install.sh /path/to/PX4-Autopilot
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"

PX4=""
for d in "${1:-}" "${PX4_DIR:-}" "$HOME/PX4-Autopilot" /opt/PX4-Autopilot; do
  if [ -n "$d" ] && [ -f "$d/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt" ]; then PX4=$d; break; fi
done
[ -n "$PX4" ] || { echo "Couldn't find PX4. Give the path: bash sim/perception/install.sh /path/to/PX4-Autopilot"; exit 1; }

# The perception model is built on P1's seveninch model
bash "$HERE/../install.sh" "$PX4" > /dev/null

AIRFRAMES=$PX4/ROMFS/px4fmu_common/init.d-posix/airframes
name=22060_gz_seveninch_perception

cp -r "$HERE/models/"* "$PX4/Tools/simulation/gz/models/"
cp "$HERE/worlds/"*.sdf "$PX4/Tools/simulation/gz/worlds/"
cp "$HERE/airframes/$name" "$AIRFRAMES/$name"
sed -i 's/\r$//' "$AIRFRAMES/$name"
grep -q "$name" "$AIRFRAMES/CMakeLists.txt" || sed -i "/4001_gz_x500/i\\	$name" "$AIRFRAMES/CMakeLists.txt"

echo "Installed into $PX4"
echo "Now run: cd $PX4 && make px4_sitl gz_seveninch_perception"
echo "Moving-obstacle world: cd $PX4 && PX4_GZ_WORLD=dodge make px4_sitl gz_seveninch_perception"
