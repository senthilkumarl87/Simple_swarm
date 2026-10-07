# Gazebo test worlds

## `obstacle_test.sdf`

Validates F_obstacle (`swarm_election.obstacle_force()`) against a real physical
collision object, not just unit-test numbers. Copy into
`$PX4_ROOT/Tools/simulation/gz/worlds/obstacle_test.sdf`, then launch with
`PX4_GZ_WORLD=obstacle_test` (see `px4-gazebo-ros2-sim` skill for the general
launch pattern).

Contains one static obstacle: a 2m-radius, 20m-tall cylinder at local NED
(N=12, E=0) -- directly in the path of `realtime_swarm_dynamic.py`'s default
`--goal-n 20 --goal-e 0` single-vehicle flight.

### Reproducing the before/after comparison

```bash
# Without F_obstacle -- flies straight at the obstacle
python3 realtime_swarm_dynamic.py --config <1-drone config> --port-base <base> \
  --goal-n 20 --goal-e 0 --isolation-rth-timeout 60 --duration 20 \
  --control-rate 10 --takeoff-alt 10 --output-dir out_without

# With F_obstacle -- same obstacle position given to the controller
python3 realtime_swarm_dynamic.py --config <1-drone config> --port-base <base> \
  --goal-n 20 --goal-e 0 --obstacle 12:0:2.0 --isolation-rth-timeout 60 \
  --duration 20 --control-rate 10 --takeoff-alt 10 --output-dir out_with
```

`--isolation-rth-timeout 60` is needed for a single-drone test: `tier_of()`
correctly classifies a lone drone (no swarm neighbors by construction) as
"isolated," which would otherwise trigger a real RTH partway through a short
test and cut off the flight before it reaches the obstacle.

### Result (2026-10-07, this repo's SITL)

Without `--obstacle`: drone flew straight north (E stayed ~0), reached the
obstacle surface (N=10, since center=12/radius=2) around t=9.9s, then altitude
went erratic and negative — a real physical collision with the cylinder.

With `--obstacle 12:0:2.0`: drone approached only to N=6.1 before deflecting
east (E grew from ~0.3 to ~7.9m over the same window), min distance to the
obstacle center stayed at 5.97m (clear of the 2m radius + margin), altitude
stayed stable (~10.0-10.4m) throughout, continued safely past the obstacle.

Known limitation, same shape as the eligibility gate's ranging-sensor gap:
`--obstacle` is a KNOWN position fed to the controller, not live onboard
sensing (lidar/depth camera). This closes the missing CONTROL LAW only.

## `multi_x500_static.sdf`

Fixes the multi-vehicle Gazebo sensor-attachment bug found earlier (3 PX4
instances dynamically spawned via `-i N` into one shared world -- only 1 of 3
ever got real sensor data; `gz topic -i` showed "No publishers" on the other
two despite a live PX4 subscriber). Confirmed this was specific to *dynamic*
spawning, not Gazebo multi-vehicle in general: this world declares all N x500
models statically at world-load time instead, and each PX4 instance attaches
via `PX4_GZ_MODEL_NAME=x500_N` (skips the spawn path entirely -- see
`px4-rc.gzsim`'s "Connect to existing model" branch) rather than spawning its
own.

**Launch**: `launch_multi_gazebo_static.sh N` (copy `multi_x500_static.sdf`
into `$PX4_ROOT/Tools/simulation/gz/worlds/` first, matching `obstacle_test.sdf`'s
setup).

**Result**: all 3 instances reached `global_pos_ok=True, home_pos_ok=True` --
genuinely fixed, confirmed by checking actual MAVSDK health status for each
instance, not just that the process launched without error. One critical
detail found by testing, not assumed: the LAST instance to attach needs a
real settle gap after the earlier ones -- a ~4s gap between launches
reproduced the identical bug (that specific instance stuck with no sensor
data) even with the static world; an ~18s gap did not. Not root-caused further
than "needs more time than a few seconds"; treat the exact number as empirical
headroom, not a derived constant. Residual `armable=False` flakiness after
that (different instances affected at different times) matches the
already-identified generic load-jitter pre-arm issue (`px4-gazebo-ros2-sim`
skill, "High Accelerometer Bias"), unrelated to the multi-vehicle bug this
world was built to fix.

Not yet done: a full multi-drone flight test (election/flocking) against this
world -- health-status validation only so far.
