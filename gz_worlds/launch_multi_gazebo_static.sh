#!/usr/bin/env bash
# Launches N PX4 instances attached to a STATICALLY-authored multi-model Gazebo
# world (multi_x500_static.sdf), instead of PX4's dynamic -i N model-spawn
# mechanism. See gz_worlds/README.md for why: dynamic spawning left 2 of 3
# instances with no real sensor data (a Gazebo sensor-system attachment race),
# confirmed via `gz topic -i` showing "No publishers" despite a live PX4
# subscriber. The static approach -- every model already declared at
# world-load time, each PX4 instance attaching via PX4_GZ_MODEL_NAME instead of
# spawning -- fixed this, but needs the LAST instance to attach only after the
# earlier ones have fully settled; a short (~4s) gap reproduced the same bug on
# the last-attached instance specifically, a long (~15-20s) gap did not.
set -eu
N="${1:-3}"
# multi_x500_static.sdf only declares x500_0..x500_2 -- a higher N would silently
# target models that don't exist and fail to attach (found via Sourcery review).
if [ "$N" -gt 3 ]; then
  echo "error: this world only declares 3 models (x500_0..x500_2); got N=$N." >&2
  echo "add more <include> blocks to multi_x500_static.sdf before raising N." >&2
  exit 1
fi
WORLD="multi_x500_static"
PX4_BIN="$HOME/PX4-Autopilot/build/px4_sitl_default/bin/px4"
LOG_DIR="/tmp/sitl_run/logs"
mkdir -p "$LOG_DIR"

export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}:$HOME/PX4-Autopilot/Tools/simulation/gz/models:$HOME/PX4-Autopilot/Tools/simulation/gz/worlds"
export DISPLAY="${DISPLAY:-:1}"

for i in $(seq 0 $((N - 1))); do
  mkdir -p "/tmp/sitl_run/gz_static_$i"
  (
    cd "/tmp/sitl_run/gz_static_$i"
    env HEADLESS=1 GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH" \
      PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_WORLD="$WORLD" PX4_GZ_MODEL_NAME="x500_$i" \
      "$PX4_BIN" -i "$i" > "$LOG_DIR/static_$i.log" 2>&1 &
  )
  echo "launched instance $i (attached to x500_$i)"
  if [ "$i" -eq 0 ]; then
    sleep 8     # let the first instance finish launching gz sim + the static world
  else
    sleep 25    # the critical gap -- 4s reproduced the sensor-attachment bug on
                # the last instance; 18s was reliable for global_pos_ok/
                # home_pos_ok but NOT sufficient to prevent occasional
                # persistent (60s+) arm() COMMAND_DENIED on a RANDOM instance
                # (not always the same slot across runs -- evidence this is
                # CPU-contention jitter during startup, not a per-slot bug).
                # Bumped empirically; still not root-caused further than
                # "needs more settle time," treat as empirical, not derived.
  fi
done
echo "all $N instances launched -- settling further before any arm attempt..."
sleep 15
echo "all $N instances launched, attached to the static world's pre-declared models"
