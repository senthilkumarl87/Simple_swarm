#!/bin/bash

# ============================================================
#  PX4 SITL - Multi Drone Launcher (Single Script)
#  Usage: ./start_multi_px4.sh <num_drones>
#  Example: ./start_multi_px4.sh 2
#
#  Each drone launches in a new terminal window with 5s delay.
#  Connect QGroundControl after — it auto-detects all drones.
#
#  Port mapping:
#    Drone 0 → QGC port 14550
#    Drone 1 → QGC port 14551
#    Drone N → QGC port 1455N
# ============================================================

# --- Config ---
PX4_ROOT="$HOME/PX4-Autopilot"
PX4_BIN="$PX4_ROOT/build/px4_sitl_default/bin/px4"
PX4_ETC="$PX4_ROOT/build/px4_sitl_default/etc"

BASE_LAT="28.4523"
BASE_LON="77.0695"
BASE_ALT="200"

DELAY=5   # seconds between each drone launch

# -----------------------------------------------

NUM_DRONES=${1:-2}  # Default: 2 drones

# Validate input
if ! [[ "$NUM_DRONES" =~ ^[0-9]+$ ]] || [ "$NUM_DRONES" -lt 1 ]; then
    echo "[ERROR] Please provide a valid number of drones (e.g. ./start_multi_px4.sh 2)"
    exit 1
fi

# Validate PX4 binary
if [ ! -f "$PX4_BIN" ]; then
    echo "[ERROR] PX4 binary not found at: $PX4_BIN"
    echo "        Run: cd ~/PX4-Autopilot && make px4_sitl_default none_iris"
    exit 1
fi

# Validate bc is available (used for float arithmetic)
if ! command -v bc &>/dev/null; then
    echo "[ERROR] 'bc' is not installed. Run: sudo apt install bc"
    exit 1
fi

echo "=============================================="
echo "  PX4 SITL Multi-Drone Launcher"
echo "  Drones     : $NUM_DRONES"
echo "  Delay      : ${DELAY}s between each"
echo "  Mode       : Headless (no Gazebo)"
echo "=============================================="
echo ""

for i in $(seq 0 $(( NUM_DRONES - 1 )) ); do

    GCS_PORT=$(( 14550 + i ))
    API_PORT=$(( 14540 + i ))
    # All drones share the SAME spawn GPS on purpose. Each PX4 SIH instance
    # anchors its local NED origin at its own spawn lat/lon, and the swarm
    # controller assumes all drones share one NED frame (it sends follower
    # targets computed from leader NED). Spawning them apart caused the
    # formation to appear as a straight east-west line in QGC — the spawn
    # offset dominated the ±5 m formation deltas. In SIH each instance is
    # its own isolated physics sim, so co-spawning has no collision cost.
    LON_OFFSET=$BASE_LON

    echo "[$(date +%H:%M:%S)] Launching Drone $((i+1)) (instance $i)"
    echo "             QGC port : $GCS_PORT"
    echo "             API port : $API_PORT"
    echo "             Position : $BASE_LAT, $LON_OFFSET"

    # Build the command to run inside a new terminal tab.
    #
    # Airframe: sihsim_quadx (PX4_SYS_AUTOSTART=10040) — the SIH (Simulator-In-Hardware)
    # quadrotor. PX4 SIH runs the physics internally and emits simulated GPS/baro/mag,
    # so there's no dependency on an external simulator (Gazebo/jMAVSim) and GPS
    # comes healthy within a few seconds. Using the default `none_iris` here
    # would hang waiting for an external simulator on TCP 4560 and trip the
    # controller's "GPS not ok within 30s" timeout.
    CMD="export HEADLESS=1; \
export PX4_SYS_AUTOSTART=10040; \
export PX4_SIM_MODEL=quadx; \
export PX4_SIMULATOR=sihsim; \
export PX4_HOME_LAT=$BASE_LAT; \
export PX4_HOME_LON=$LON_OFFSET; \
export PX4_HOME_ALT=$BASE_ALT; \
echo '--- Drone $((i+1)) | Instance $i | QGC port $GCS_PORT | airframe sihsim_quadx ---'; \
$PX4_BIN -i $i -d $PX4_ETC; \
exec bash"

    # Launch in a new terminal window (supports gnome-terminal, xterm, xfce4-terminal)
    if command -v gnome-terminal &>/dev/null; then
        gnome-terminal --title="PX4 Drone $((i+1))" -- bash -c "$CMD" &
    elif command -v xfce4-terminal &>/dev/null; then
        xfce4-terminal --title="PX4 Drone $((i+1))" -e "bash -c '$CMD'" &
    elif command -v xterm &>/dev/null; then
        xterm -title "PX4 Drone $((i+1))" -e "bash -c '$CMD'" &
    else
        echo "[WARN] No GUI terminal found. Launching in background (no console output)."
        eval "$CMD" &
    fi

    # Wait before next drone (skip wait after last one)
    if [ $i -lt $(( NUM_DRONES - 1 )) ]; then
        echo "         Waiting ${DELAY}s before next drone..."
        echo ""
        sleep $DELAY
    fi

done

echo ""
echo "=============================================="
echo "  All $NUM_DRONES drone(s) launched!"
echo "  Open QGroundControl now."
echo "  It will auto-detect all vehicles."
echo ""
echo "  If a drone is missing in QGC, add manually:"
for i in $(seq 0 $(( NUM_DRONES - 1 )) ); do
    echo "    Drone $((i+1)) → UDP port $(( 14550 + i ))"
done
echo "=============================================="
