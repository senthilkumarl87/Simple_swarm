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

**Full multi-drone flight test now done**, not just health-status checks. Getting there surfaced three more real
bugs in `realtime_swarm_mavsdk.py` (all fixed, see its own commit): `arm()` transiently `COMMAND_DENIED` under
real CPU contention even after health checks report armable (fixed with a bounded retry); that retry then let
PX4's offboard setpoint go stale, so `offboard.start()` failed with `NO_SETPOINT_SET` (fixed by refreshing the
setpoint during the wait); and the "skip if already in air" check trusted a fresh connection's first `in_air()`
read alone, which can be stale, leaving one drone motionless on the ground for a whole run while the others flew
(fixed by also requiring `armed`). Separately, one PX4 instance's EKF2 got persistently stuck despite Gazebo
publishing real sensor data and a sane physics pose -- not root-caused, worked around by restarting just that one
PX4 process (re-attach via `PX4_GZ_MODEL_NAME`, no Gazebo restart needed).

Example command (3 drones, matching `multi_x500_static.sdf`'s declared positions):

```bash
python3 realtime_swarm_dynamic.py --config <3-drone config> --port-base 50040 \
  --comm-range-lora 30 --goal-n 20 --goal-e 0 --duration 25 --control-rate 10 \
  --takeoff-alt 10 --land-on-exit --output-dir out_multi_gz
```

Result (2026-10-07): all 3 drones took off, elected hw_id 1, then organically (no injected fault)
periodic-re-elected to hw_id 3 at t=14.1s as scores shifted, briefly partitioned (hw_id 3 isolated at t=19.9s),
and merged back to hw_id 1 at t=21.5s -- real election/partition/merge dynamics in live Gazebo physics, not
fault-injected. One real safety finding, not glossed over: minimum pairwise separation reached 0.66m mid-flight
(excluding the known co-spawn artifact at t=0), tighter than the obstacle test's 5-6m margins -- plausibly tied to
the partition event reducing effective repulsion responsiveness. Open item: flocking-parameter tuning under real
election dynamics specifically, not just static scenarios.
