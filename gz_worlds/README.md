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
