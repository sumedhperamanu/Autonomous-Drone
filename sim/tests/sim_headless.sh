#!/usr/bin/env bash
# Run one headless PX4 + Gazebo sim in the background (no GUI), for scripted tests.
# Usage: bash sim/tests/sim_headless.sh <airframe-id> <model> [world]   e.g. ... 22050 gz_fiveinch
# Stop:  pkill -x px4; pkill -x ruby      (gz sim runs as ruby)
set -euo pipefail
# PX4 checkout: $PX4_DIR if set, else ~/PX4-Autopilot, else /opt/PX4-Autopilot (the repo's dev container)
if [ -z "${PX4_DIR:-}" ]; then
  for d in "$HOME/PX4-Autopilot" /opt/PX4-Autopilot; do [ -d "$d/build/px4_sitl_default" ] && PX4_DIR=$d && break; done
fi
[ -n "${PX4_DIR:-}" ] || { echo "PX4 build not found. Build it once (make px4_sitl gz_fiveinch) or set PX4_DIR=..."; exit 1; }
export PX4_DIR
B=$PX4_DIR/build/px4_sitl_default
cd "$B/rootfs"
rm -f parameters*.bson dataman            # fresh params each run, so the airframe defaults apply
export PX4_SYS_AUTOSTART=$1 PX4_SIM_MODEL=$2 PX4_GZ_WORLD=${3:-default} HEADLESS=1
nohup "$B/bin/px4" -d "$B/etc" -w "$B/rootfs" > "/tmp/px4_$2.log" 2>&1 &
echo "px4 pid $!  log /tmp/px4_$2.log"
