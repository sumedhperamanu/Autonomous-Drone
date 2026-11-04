#!/usr/bin/env bash
# Start one of our sims and send its telemetry to QGroundControl.
#   bash run.sh                              7" on 4S
#   bash run.sh gz_fiveinch                  5"
#   bash run.sh gz_seveninch_lidar_flow walls
# install.sh copies this file to ~/run_sim.sh so it's easy to find.
#
# QGC: telemetry goes to host.docker.internal, which is your Windows PC when you use Docker Desktop.
# Set QGC_IP=1.2.3.4 to send it somewhere else, or QGC_IP=none to skip it.
set -euo pipefail
TARGET=${1:-gz_seveninch_4s}
WORLD=${2:-default}

PX4=""
for d in "${PX4_DIR:-}" "$HOME/PX4-Autopilot" /opt/PX4-Autopilot; do
  if [ -n "$d" ] && [ -f "$d/Makefile" ]; then PX4=$d; break; fi
done
[ -n "$PX4" ] || { echo "Couldn't find PX4. Set PX4_DIR=/path/to/PX4-Autopilot"; exit 1; }

QGC_IP=${QGC_IP:-$(getent ahostsv4 host.docker.internal 2>/dev/null | awk 'NR==1 {print $1}')}
ROOTFS=$PX4/build/px4_sitl_default/rootfs
if [ -n "$QGC_IP" ] && [ "$QGC_IP" != "none" ]; then
  # Wait in the background until PX4 is up, then add a MAVLink link to QGC (UDP 14550).
  # On the very first run PX4 hasn't been built yet, so this also waits for the build (up to 1 hour).
  (
    for _ in $(seq 1 1800); do
      if [ -x "$ROOTFS/../bin/px4-commander" ] && (cd "$ROOTFS" && ../bin/px4-commander status >/dev/null 2>&1); then
        cd "$ROOTFS"
        sleep 3
        ../bin/px4-mavlink start -u 14561 -o 14550 -t "$QGC_IP" -r 4000000 >/dev/null 2>&1 \
          && echo "QGroundControl link: sending to $QGC_IP port 14550"
        exit 0
      fi
      sleep 2
    done
  ) &
fi

cd "$PX4"
PX4_GZ_WORLD=$WORLD make px4_sitl "$TARGET"
