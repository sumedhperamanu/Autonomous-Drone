#!/usr/bin/env bash
# Copy the 5-inch quad models and airframes into the kit's PX4 checkout.
# Usage (inside the dev container, from the kit folder): bash sim/fiveinch/install.sh [path/to/PX4-Autopilot]
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
PX4_DIR="${1:-$HERE/../../third_party/PX4-Autopilot}"
MODELS="$PX4_DIR/Tools/simulation/gz/models"
AIRFRAMES="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"

[ -d "$MODELS" ] && [ -d "$AIRFRAMES" ] || { echo "PX4 not found at $PX4_DIR (run scripts/bootstrap.sh first)"; exit 1; }

cp -r "$HERE"/models/* "$MODELS/"
cp "$HERE"/airframes/* "$AIRFRAMES/"
for af in 22050_gz_fiveinch 22051_gz_fiveinch_lidar_flow; do
  grep -q "$af" "$AIRFRAMES/CMakeLists.txt" || sed -i "/Reserve for custom models/a\\	$af" "$AIRFRAMES/CMakeLists.txt"
done

echo "Installed. Start the sim with one of:"
echo "  scripts/sim_up.sh gz_fiveinch                    # bare 5-inch quad"
echo "  scripts/sim_up.sh gz_fiveinch_lidar_flow walls   # phase 1 sensors, room with walls"
