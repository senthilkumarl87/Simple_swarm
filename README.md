# Simple_swarm — Algorithm & Pipeline Reference

Leader/follower swarm toolkit covering three deployment tiers from the same
core math:

1. **Offline planner** — generate per-drone trajectories from a pattern file +
   leader path, simulate them through a kinematic PD tracker to check
   feasibility, and visualize commanded + achieved motion.
2. **Option C — real-time, loopback only** — independent leader + follower
   processes talk over localhost UDP. Each follower runs its own kinematic PD
   tracker. Validates the protocol, rates, and follower math before touching
   an autopilot.
3. **Option B — real-time, MAVSDK-in-the-loop** — a single process owns
   MAVSDK connections to every drone (or SITL instance), subscribes to the
   leader's telemetry, computes setpoints, and pushes them via offboard mode.
4. **Option D — dynamic master election + three-force flocking** (§16) —
   same one-process/MAVSDK-in-the-loop shape as Option B, but with no fixed
   leader: every drone runs the same decentralized election + flocking logic
   every control tick, closing Option B's own §10.4 limitation ("single
   process = single point of failure... every follower falls back on
   whatever PX4 does when offboard stream stops").

The offline tier is self-contained (no MAVSDK, no sockets). Option C uses
loopback UDP only. Option B and Option D both talk to `mavsdk_server`
instances connected to PX4. Every tier shares the same rotation math (§5.2)
and the same output CSV schema (§5.5), so the visualizer / animator work on
any stage's logs — Option D's own CSV output is covered in §16.8, not §1.4:
same per-drone position/velocity/yaw columns, but master/tier/election
events are separate files (there is no "leader" to express an offset from).

---

## 1. Pipeline at a glance

### 1.1 Offline tier (planning + kinematic validation)

```
                            test_inputs/<name>/
                          ┌────────────────────┐
                          │ drones_config.json │      (leader + fleet)
                          │ formation.csv      │      (body-frame offsets)
                          │ leader_path.csv    │      (t, n, e, alt [, hdg])
                          └─────────┬──────────┘
                                    │
                       generate_trajectories.py
                                    │
                                    ▼
                    output/drone_<id>[_leader].csv
                    (NED pos/vel/acc + yaw + mode + LED)
                                    │
         ┌──────────────────────────┼──────────────────────────┐
         ▼                          ▼                          ▼
 visualize_trajectories.py   animate_trajectories.py   simulate_swarm.py
  4-panel static PNG         3D GIF of planned paths    PD-tracked playback
                                                       + safety thresholds
                                                                │
                                                                ▼
                                               output/simulated/drone_*.csv
                                               (achieved, same schema)
                                               + commanded_vs_simulated.gif
                                               (if --animate)
```

### 1.2 Real-time tier — Option C (loopback UDP, kinematic followers)

```
      realtime_leader.py              realtime_follower.py  (one per drone)
  ┌───────────────────────┐        ┌──────────────────────────────────────┐
  │ reads leader_path.csv │  UDP   │ binds 37540+hw_id, non-blocking       │
  │ wall-clock replay     │ ─────▶ │ extrapolates leader to "now"          │
  │ unicasts @ --rate Hz  │ JSON   │ computes target = leader + R·offset   │
  │ logs its own CSV      │        │ PD integrator @ --sim-rate (50 Hz)    │
  └───────────────────────┘        │ logs drone_N.csv                      │
              ▲                    └──────────────────────────────────────┘
              │                                     │
         run_realtime_demo.py  ───── spawns ────────┘
         (orchestrator + optional --visualize)
```

### 1.3 Real-time tier — Option B (centralized, MAVSDK)

```
                   realtime_swarm_mavsdk.py (ONE process)
                ┌──────────────────────────────────────────┐
                │ async main loop, no sockets internally   │
                │                                          │
                │  leader state source:                    │
                │    --leader-source mavsdk  → subscribe   │
                │        telemetry of drone hw_id = leader │
                │    --leader-source csv     → replay file │
                │                                          │
                │  control loop @ --control-rate Hz:       │
                │    for each follower:                    │
                │      compute target (rotation + ω×r)     │
                │      drone.offboard.set_position_...     │
                │                                          │
                │  graceful shutdown: stop offboard, land  │
                └──────────────────────────────────────────┘
                   │       │       │           │
                   ▼       ▼       ▼           ▼
                 mavsdk  mavsdk  mavsdk      mavsdk
                 _srv    _srv    _srv        _srv
                   │       │       │           │
                 PX4(1)  PX4(2)  PX4(3)  ...  PX4(N)
                (SITL or real vehicles)
```

### 1.4 Common CSV schema

Every tier writes per-drone CSVs in the same schema, so
`visualize_trajectories.py --trajectories-dir <anything>` and
`animate_trajectories.py --trajectories-dir <anything>` work on offline,
Option-C, and Option-B output alike.

---

## 2. Scripts

### Offline tier

| File | Role |
|---|---|
| `generate_trajectories.py` | Plans each drone's trajectory from the pattern + leader path. |
| `simulate_swarm.py`        | Kinematic playback with PD tracking + saturation + safety checks. |
| `visualize_trajectories.py`| Static 4-panel diagnostic plot for any trajectory directory. |
| `animate_trajectories.py`  | 3D GIF animation of any trajectory directory. |
| `_gen_test_inputs.py`      | Regenerates `test_inputs/` with five canonical scenarios. |
| `run_tests.py`             | Offline pipeline regression runner; exit 1 on any scenario fail. |

### Real-time tier — Option C (loopback UDP)

| File | Role |
|---|---|
| `realtime_protocol.py`   | Shared UDP/JSON schema + port convention (hw_id N → `127.0.0.1:37540+N`). |
| `realtime_filters.py`    | Optional α-β smoother for the follower (off by default; see §7.6). |
| `realtime_leader.py`     | Wall-clock CSV replayer; unicasts state @ `--rate` Hz to every follower. |
| `realtime_follower.py`   | Listens, rotates offset, PD-tracks its own state @ `--sim-rate` Hz. |
| `run_realtime_demo.py`   | Orchestrator — spawns leader + all followers as subprocesses. |
| `run_realtime_tests.py`  | End-to-end real-time test runner; wall-clock scenario matrix. |

### Real-time tier — Option B (centralized, MAVSDK)

| File | Role |
|---|---|
| `realtime_swarm_mavsdk.py` | Centralized controller — connects to every drone via MAVSDK, picks a leader state source (MAVSDK telemetry or CSV replay), computes targets, pushes offboard setpoints. |
| `start_multi_px4.sh`       | Spawns N headless PX4 SIH (`sihsim_quadx`) instances, one per gnome-terminal. All drones co-spawn at one GPS origin; ports `14540+N` / `14550+N`. See §14.4. |
| `swarm_gui.py`             | Tkinter + matplotlib GUI — pick a scenario, preview the formation in 3D, one-click Simulate to launch PX4 + mavsdk_server + controller. See §15. |

---

## 3. Coordinate conventions

Because the output plugs into the PX4 offboard playback in the repo, the
conventions match:

- **NED** — `north` = +n, `east` = +e, `down` = +d. Altitude is positive up but
  stored as `pz = -altitude` in the output CSVs.
- **Heading**: degrees in `leader_path.csv` (0° = facing north, +90° = east,
  clockwise from north, standard compass).
- **Yaw**: radians in every output CSV's `yaw` column. Converted once in
  `build_trajectory_df` via `radians(heading_deg)`.
- **Velocity / acceleration**: world-frame NED. `vz > 0` = descending, `vz < 0`
  = climbing. Consistent with PX4 offboard setpoints.

---

## 4. Input formats

### 4.1 `drones_config.json`

```json
{
  "drones": [
    {"drone_id": 1, "hw_id": 1, "ip": "udp://:14540", "port": 14540, "is_leader": true},
    {"drone_id": 2, "hw_id": 2, "ip": "udp://:14541", "port": 14541, "is_leader": false}
  ]
}
```

Only `hw_id` and `is_leader` are used by the planner. The script requires
**exactly one** leader.

### 4.2 `formation.csv`

```
hw_id,follow,offset_n,offset_e,offset_alt
1,0,0,0,0
2,1,-2,-2,0.5
3,1, 2,-2,0.5
```

- `follow = 0` marks the leader row.
- `follow = <leader hw_id>` marks a follower; only rows that reference the
  leader's hw_id are loaded (multi-leader formations are not supported here).
- Offsets are in the **leader's body frame** — `offset_n` = along the leader's
  facing direction (forward positive), `offset_e` = perpendicular to the right,
  `offset_alt` = up relative to the leader.

### 4.3 `leader_path.csv`

Required columns: `t, north, east, altitude`. Optional: `heading` (degrees).

```
t,north,east,altitude,heading
0.0, 0.0, 0.0, 10.0,   0.0
1.0, 2.0, 1.0, 10.5,  15.0
...
```

- `t` is sorted ascending but does **not** need to be uniform — velocity and
  acceleration are derived via `np.gradient(value, t)` which handles
  non-uniform spacing.
- If `heading` is missing, it is derived from the path tangent (see §5.1).

---

## 5. Planner algorithm (`generate_trajectories.py`)

### 5.1 Auto-derived heading (when the column is absent)

```
dN = ∇ₜ(north)
dE = ∇ₜ(east)
heading_deg = atan2(dE, dN) · (180/π)
```

This makes the leader point where it's going; matches a typical fixed-wing or
auto-yaw copter. If you want a different leader yaw (e.g. always facing the
sun), provide the column explicitly.

### 5.2 Follower position — rigid-body rotation by leader heading

At every sample the follower's body-frame offset `(oₙ, o_e)` is rotated into
world NED by the leader's heading θ, then added to the leader's world
position:

```
[rotₙ]   [cos θ   -sin θ] [oₙ]
[rot_e] = [sin θ    cos θ] [o_e]

followerₙ(t) = leaderₙ(t) + rotₙ(t)
follower_e(t) = leader_e(t) + rot_e(t)
follower_alt(t) = leader_alt(t) + o_alt
```

This is the only coupling between leader and follower positions. It is applied
per-sample, so when the leader turns, the whole formation rotates around the
leader in real time. Altitude offset is a plain scalar add (no rotation).

Verification from the sample data — follower 2 with offset `(2, -2)` tracked
against leader heading 0° → 360°:

| leader heading | F2 relative to leader (n, e) |
|---|---|
| 0°   | (+2, −2) |
| 90°  | (+2, +2) |
| 180° | (−2, +2) |
| 270° | (−2, −2) |

Classic rigid rotation, formation shape preserved.

### 5.3 Velocity and acceleration via `np.gradient`

Using actual time stamps handles irregular sampling:

```
vx = ∇ₜ(north)      ax = ∇ₜ(vx)
vy = ∇ₜ(east)       ay = ∇ₜ(vy)
vz = −∇ₜ(altitude)  az = ∇ₜ(vz)      (NED sign flip for vertical)
```

`np.gradient` uses second-order centered differences internally (forward
/ backward at the endpoints), which means endpoint derivatives are O(dt)
instead of O(dt²).

### 5.4 Yaw mode

Each output row carries a `yaw` column in radians. Two modes, selected via
`--follower-yaw`:

**`leader` (default)** — every follower's yaw = leader's heading at that
timestep. Formation flies rigid; all drones face the same direction.

```
yaw_follower(t) = radians(heading_leader(t))
```

**`motion`** — each follower's yaw tracks its own direction of travel
(tangent to its own path):

```
yaw_raw(t) = atan2(vy(t), vx(t))
yaw(t)     = yaw_raw(t)          if ||v_xy(t)|| ≥ 0.1 m/s
           = heading_leader(t)   otherwise      (low-speed fallback)
yaw_final  = unwrap(yaw)                        (remove 2π jumps)
```

The leader's yaw always uses its commanded heading from `leader_path.csv` —
the path was chosen deliberately and we shouldn't override it with a
motion-tangent estimate.

### 5.5 Output CSV schema (what every downstream script consumes)

```
idx, t, px, py, pz, vx, vy, vz, ax, ay, az, yaw, mode, ledr, ledg, ledb
```

- `px, py, pz` = world NED (pz = −altitude).
- `yaw` in radians.
- `mode = 70` (maneuvering), matches the existing `offboard_from_csv.py`
  state-machine code.
- LED = white for followers (255,255,255) and red for the leader
  (255,64,64) — purely for the visualizer / playback; not consumed by the
  algorithm.

---

## 6. Kinematic simulator (`simulate_swarm.py`)

### 6.1 Purpose

The planner output is **commanded** motion — a geometric plan. It may
include instantaneous velocity jumps (at heading reversals) or climb rates
that a real copter can't match. The simulator replays the CSVs as
setpoints through a simple dynamic model to expose infeasibility before
you touch an autopilot.

### 6.2 Model

Per drone, independently:

```
state:   x = [position (n,e,d),  velocity (vn,ve,vd)]     ∈ ℝ⁶
         ψ = yaw                                          ∈ ℝ
input:   x_set, v_set (from planner),  ψ_set (from planner)
```

Double integrator on (pos, vel) + first-order yaw tracker. No attitude
dynamics, no aerodynamics, no coupling between drones (they only interact
via the safety metrics).

### 6.3 Control law

```
a_cmd     = k_p (x_set − x)  +  k_v (v_set − v)      PD in each axis
ψ̇_cmd     = k_yaw (ψ_set − ψ)
```

Defaults: `k_p = 4`, `k_v = 3`, `k_yaw = 4`. These make the horizontal
and vertical loops roughly critically damped at the default saturation
limits. Tune via the JSON limits file if you want more/less lag.

### 6.4 Saturation (independent horizontal / vertical)

```
‖a_xy_cmd‖ > max_accel_xy   →   a_xy = a_xy · max_accel_xy / ‖a_xy_cmd‖
|a_z_cmd|  > max_accel_z    →   a_z  = clip(a_z_cmd, ±max_accel_z)

after v += a·Δt:
‖v_xy‖     > max_speed_xy   →   v_xy scaled to max_speed_xy
v_z  < −max_speed_up        →   v_z = −max_speed_up
v_z  >  max_speed_down      →   v_z =  max_speed_down

|ψ̇_cmd|    > max_yaw_rate   →   ψ̇ = clip(ψ̇_cmd, ±max_yaw_rate)
```

Horizontal velocity / acceleration are clamped by *norm* (direction
preserved). Vertical is clamped by *sign* with asymmetric climb/descent
limits — matches quadcopter physics (climb is thrust-limited, descent
is drag-limited and usually slower).

### 6.5 Integration

Forward Euler at fixed `sim_dt` (default 0.02 s = 50 Hz):

```
v_{k+1} = v_k + a_k · Δt
x_{k+1} = x_k + v_{k+1} · Δt
ψ_{k+1} = ψ_k + ψ̇_k · Δt
```

The commanded trajectory is linearly interpolated onto the 50 Hz grid
before integration. Yaw is `np.unwrap`'d before interpolation so that a
commanded sweep 345° → 0° interpolates forward as +15° rather than
backward by −345°.

### 6.6 Safety metrics

At the end of the run:

- **Per-drone**:
  - peak tracking error = `max ‖x_cmd − x_actual‖`
  - RMS tracking error
  - saturation % on accel, velocity, yaw-rate (fraction of steps where
    the clamp activated)
- **Global**:
  - minimum pairwise separation across all drone pairs over all samples
  - minimum altitude (ground-collision check)

Thresholds (configurable via constants in the script):

```
max_tracking_error = 2.0 m
min_separation     = 0.5 m
min_altitude       = 0.0 m
```

Exit code is `1` if any threshold is crossed — the runner uses this.

---

## 7. Real-time tiers (Option C + Option B)

The offline planner produces CSVs that every timestep is pre-computed. The
real-time tiers do the same math **per tick in wall-clock time**: the leader
state becomes a live signal (either replayed from CSV or read off MAVSDK
telemetry), and each follower consumes it through a fixed-rate control
loop.

### 7.1 Wire protocol (Option C only)

One UDP JSON datagram per leader tick to each follower, defined in
`realtime_protocol.py`:

```json
{
  "seq": 42, "hw_id": 1, "t_leader": 4.20,
  "n":  8.4, "e":  1.8, "alt": 10.84,
  "heading_deg": 21.0,
  "vn": 2.0, "ve": 0.42, "vd": -0.2,
  "heading_rate_dps": 15.0
}
```

- `n, e, alt` — leader world-NED position (altitude positive up).
- `vn, ve, vd` — leader world-NED velocity (`vd > 0` = descending).
- `heading_deg` — continuous (unwrapped) leader heading in degrees.
- `heading_rate_dps` — dθ/dt in deg/s. **Critical for rotating formations**
  (see §7.4). Followers that see a 0 here behave correctly for straight-line
  translation; they lag badly when the leader turns in place.

Port convention: follower hw_id `N` binds to `127.0.0.1:37540 + N`. The
base is centralized in `REALTIME_PORT_BASE` and chosen to stay clear of the
repo's MAVSDK (`14540+`) and GCS (`34550`) ranges.

Option B does not use this protocol — everything lives in one process, so
leader state is just shared Python state (`LeaderState` in
`realtime_swarm_mavsdk.py`).

### 7.2 Option C architecture

Three process types:

| Process | Responsibilities |
|---|---|
| `realtime_leader.py` | Reads `leader_path.csv`, pre-computes tangent heading and velocities, interpolates at wall-clock time, unicasts a packet to every follower each tick, logs its own trajectory. |
| `realtime_follower.py` (one per drone) | Binds its UDP port non-blocking, waits for the first leader packet, initializes state at `leader_pos + R(heading)·offset_body`, then runs a fixed-rate loop. |
| `run_realtime_demo.py` | Spawns followers first (so they bind before the leader sends), then the leader, waits for all to finish, optionally runs the visualizer. |

**Rate separation**: the leader broadcasts at `--rate` Hz (default 10) and
each follower runs its inner control loop at `--sim-rate` Hz (default 50).
Between leader packets the follower **linearly extrapolates** using the last
received velocity and heading rate:

```
Δt = now - last_packet_rx_time
leader_n(now)       = last.n        + last.vn * Δt
leader_e(now)       = last.e        + last.ve * Δt
leader_alt(now)     = last.alt      + (-last.vd) * Δt          # NED sign
leader_heading(now) = last.heading  + last.heading_rate_dps * Δt
```

This works because the inner loop runs at 50 Hz while packets arrive at 10
Hz — followers see a prediction-between-updates that is always within one
leader period of ground truth. At 10 Hz on loopback, typical packet jitter
was <1 ms in our test runs.

**Inner control loop** (per sim tick, see §6.2–6.4 — it's the offline
simulator's PD tracker with saturation, reused verbatim):

```python
rot_n, rot_e = R(leader_heading) · (offset_n, offset_e)
ω = radians(leader_heading_rate_dps)

target_pos = (leader_n + rot_n,
              leader_e + rot_e,
              -(leader_alt + offset_alt))            # NED down positive
target_vel = (leader_vn - ω·rot_e,                   # §7.4 feed-forward
              leader_ve + ω·rot_n,
              leader_vd)
target_yaw = radians(leader_heading)

# PD + saturation (same as simulate_swarm.py DEFAULT_LIMITS)
acc = kp*(target_pos - pos) + kv*(target_vel - vel)
# clamp acc / vel / yaw_rate, Euler integrate
```

**Shutdown**: each follower exits after `--silence-timeout` seconds with no
leader packet (default 2 s). In real flight you'd replace this with a
"hover in place" safety; for this demo, exit is fine.

### 7.3 Option B architecture

One process, no sockets on the control path:

```
realtime_swarm_mavsdk.py (async main)
├── connect_drone() for every hw_id in config   → System objects on 127.0.0.1:50040+hw_id
├── wait_gps_ok()                                 (skipped in --dry-run)
├── arm_takeoff() on every drone                  (skipped in --dry-run)
├── start_offboard_at_current() on each follower  (skipped in --dry-run)
├── leader_{mavsdk,csv}_*_task() → LeaderState    (pick via --leader-source)
├── control_loop()  @ --control-rate Hz:
│     target = rotation + feed-forward (§7.4)
│     drone.offboard.set_position_velocity_ned(...)  per follower
├── telemetry_logger_task() per drone → achieved CSV
└── graceful_shutdown(): stop offboard, optional land
```

**Leader state source** is pluggable:

- `--leader-source mavsdk` — subscribes to the leader drone's
  `telemetry.position_velocity_ned()` and `telemetry.attitude_euler()`
  streams. Heading rate is computed in-process by shortest-angle finite
  difference on consecutive yaw samples
  (`LeaderState.update_heading` — handles 359°→1° correctly as +2°, not
  −358°).
- `--leader-source csv` — replays `leader_path.csv` in wall-clock time
  into `LeaderState` at 50 Hz (same math as `realtime_leader.py`, but
  in-process).

**Setpoint** is pushed via
`drone.offboard.set_position_velocity_ned(PositionNedYaw(...),
VelocityNedYaw(...))`. Position *and* velocity — PX4's onboard position
controller blends both, so the rotational feed-forward (§7.4) is essential
for rotating formations.

**No PD / saturation in Option B** — PX4 has its own position controller
with its own limits. We send references; the flight controller closes the
loop. That means no `kp/kv/kyaw`, no accel/velocity clamps in this script.

**`--dry-run`**: still connects, still reads telemetry, still computes
setpoints, still logs — but never calls `arm()`, `takeoff()`, or
`offboard.start()`. Use it to validate wiring without putting vehicles in
motion.

**Frame assumption**: everyone shares one NED origin. In SITL this holds
when you spawn all drones at the same XY. For real flight across distances
you'll want to switch to global (lat/lon) broadcasts and do
`LLA → local NED` per follower, like `src/drone_config.py::convert_LLA_to_NED`
does. Not wired up yet.

### 7.4 Rotational velocity feed-forward

When the leader rotates, each follower's world-NED position changes even
if the leader is stationary (e.g. a hover-and-rotate). The offline planner
captures this automatically because `np.gradient` on the commanded
trajectory bakes the tangential velocity into the output CSV. The
real-time tiers must reconstruct it.

Derivation: with body offset `(oₙ, o_e)` rotated by leader heading θ(t):

```
rot_n(t) = oₙ·cos θ(t) − o_e·sin θ(t)
rot_e(t) = oₙ·sin θ(t) + o_e·cos θ(t)

d(rot_n)/dt = θ̇·(−oₙ·sin θ − o_e·cos θ) = −θ̇·rot_e
d(rot_e)/dt = θ̇·( oₙ·cos θ − o_e·sin θ) = +θ̇·rot_n
```

So the follower's total target velocity is:

```
target_vn = leader_vn − ω·rot_e
target_ve = leader_ve + ω·rot_n
target_vd = leader_vd
```

where `ω = radians(heading_rate_dps)`. This was the bug the real-time test
suite caught in `05_hover_rotate`: without it, peak tracking error was
**1.64 m** because the PD controller had only position feedback for a
follower orbiting a stationary leader at 2.1 m/s. With the feed-forward it
drops to **0.30 m** — a 5× improvement. Applied in both Option C
(`realtime_follower.py`) and Option B (`control_loop` in
`realtime_swarm_mavsdk.py`).

### 7.6 Optional α-β smoother on the leader state (Option C only, off by default)

`realtime_filters.AlphaBetaFilter` is a minimal constant-velocity α-β
tracker (state = position + velocity; gains α, β). When enabled via
`realtime_follower.py --smooth`, three independent instances run on the
leader's `(n, e, alt)` samples. Both the filtered position and the
filtered velocity are then used for the follower's target computation
(replacing the raw packet values), so filtering is internally consistent
across both channels. Heading is **not** filtered — it's already
shortest-angle robust on the leader side, and filtering it would need
unwrap bookkeeping for little gain.

The follower also rejects packets whose `seq` is lower than the last seen
one, so UDP reordering across a real network won't drive the filter
backwards.

Why it's off by default — direct measurement on the loopback test suite,
α=0.5, β=0.1 vs no filter:

| scenario | α-β OFF | α-β ON | Δ |
|---|---|---|---|
| 01_basic_spear         | 0.06 m | 0.13 m | +0.07 m |
| 02_circle_auto_heading | 0.23 m | 0.36 m | +0.13 m |
| 03_vertical_spiral     | 0.47 m | 0.77 m | +0.30 m |
| 05_hover_rotate        | 0.30 m | 0.30 m | +0.00 m |

On loopback the leader signal is synthesized from a clean CSV and UDP
jitter is sub-millisecond. With nothing to filter, smoothing is pure
phase lag and tracking error grows roughly proportionally to the
leader's velocity (cost = v · effective_lag). The hover-rotate case is
unchanged because the leader's position is stationary, so there's
neither noise to smooth nor velocity to lag behind — the rotational
feed-forward (§7.4) handles that case upstream of the filter.

When the filter actually earns its keep:

- Option C running across a real network (WiFi / LTE / mesh radio) with
  visible UDP jitter or occasional packet loss.
- Signal sources with observable noise (e.g. if we later add
  GPS-broadcast leader mode and per-follower LLA→NED conversion that
  inherits GPS jitter).

For anything on loopback, leave it off. See §13.4 for the flags.

### 7.7 What Option C uses from the offline simulator

The per-follower inner loop in Option C is literally a copy of the offline
PD + saturation kinematic model from §6. So:

- Option C output CSVs have the same schema as simulated ones (§5.5), and
  `visualize_trajectories.py` / `animate_trajectories.py` work unchanged.
- The tracking bounds in `run_realtime_tests.py` (§9.2) compare Option C
  output against the **offline** planner's output for the same scenario,
  since the offline plan is the ground-truth target the real-time
  integration should reproduce.

Option B output CSVs use the same schema but the `px, py, pz, vx, vy, vz`
columns come from MAVSDK telemetry (what PX4 actually flew), not from an
integrator — so they can include sensor noise, vehicle dynamics, and wind.

---

## 8. Visualization

### 8.1 `visualize_trajectories.py` — four static panels

| Panel | What it shows |
|---|---|
| 3D | all drone paths in world NED, start (●) and end (▲) markers |
| Top-down | east/north projection + heading arrows at 8 sampled times + formation polygons at 6 sampled times |
| Altitude vs time | per-drone altitude trace |
| Formation (leader-translated frame) | leader pinned at origin; formation polygon at 8 sampled times, colored by time (viridis); leader-heading arrows — this is where formation rotation is visually unambiguous |

The formation-polygon and formation-rotation panels sample by **time**
(not index), so they work correctly when different drones have different
sample counts — e.g. Option C logs where the leader is at 10 Hz and
followers are at 50 Hz.

### 8.2 `animate_trajectories.py` — 3D GIF of planned motion

Per frame: current marker + fading trail (`--trail-length` samples) +
formation polygon connecting all drones + per-drone yaw arrow. Title
shows `t` and the leader's current heading.

### 8.3 `simulate_swarm.py --animate` — commanded vs achieved overlay

Same animation style but each drone is drawn twice:

- **filled marker** = achieved (simulator output)
- **hollow marker** = commanded (planner setpoint at this t)
- **dotted line** between them = instantaneous tracking error vector,
  stretches on sharp commanded turns and snaps back once the controller
  catches up

Title shows `t` plus the worst tracking error across the swarm at the
current frame.

---

## 9. Test procedure

### 9.1 Scenarios

| # | Folder | Fleet | Leader motion | What it tests |
|---|---|---|---|---|
| 1 | `01_basic_spear` | 5-drone spear | gentle curve + slow climb | happy-path regression baseline |
| 2 | `02_circle_auto_heading` | 2-follower V | 5 m circle, heading column omitted | auto-tangent heading derivation |
| 3 | `03_vertical_spiral` | 3-follower quad | helix, 0.3 m/s climb | vertical tracking, altitude derivatives |
| 4 | `04_infeasible_tight_limits` | 2-follower V | 20 m/s leader, 45°/s yaw | simulator's safety-threshold exit code (expected fail) |
| 5 | `05_hover_rotate` | 7 followers on a 4 m ring | hover + 30°/s yaw | rigid-body rotation around stationary leader |

### 9.2 Two independent runners

**Offline runner** — `run_tests.py`. Runs all 5 scenarios through
`generate_trajectories.py → simulate_swarm.py`. Parses the simulator's
stdout report and compares against `meta.json` bounds. Synchronous, ~2 s
total wall-clock. Used as the regression gate for planner and simulator
logic.

**Real-time runner** — `run_realtime_tests.py`. Runs all scenarios *except*
`04_infeasible_tight_limits` (not meaningful at the Option C level — the
simulator's safety test is about clamp behavior, not real-time protocol)
through `run_realtime_demo.py`. For every scenario:

1. Ensures the offline ground truth exists (generates it if missing).
2. Spawns leader + all followers, waits for natural exit.
3. Verifies orchestrator + subprocess exit codes are 0.
4. Checks leader packet count is within ±10% of `rate × duration`.
5. Checks leader and worst-follower tick p95 jitter < 30 ms.
6. Compares every follower's achieved CSV against the offline ground
   truth, skipping the first 1 s to let the PD transient settle. Peak
   3D tracking error must stay under a per-scenario tolerance
   (`TRACKING_TOL_M` map in the runner).

Real-time runner takes ~85 s (runs in wall-clock) and is CI-ready — exit
code 1 on any failure. When we introduced the heading-rate feed-forward
(§7.4), it was the hover-rotate scenario in *this* runner that failed
first and drove the fix. Numbers after the fix:

| scenario | before | after | tolerance |
|---|---|---|---|
| 01_basic_spear         | 0.31 m | 0.06 m | 0.60 m |
| 02_circle_auto_heading | 0.47 m | 0.22 m | 0.60 m |
| 03_vertical_spiral     | 0.89 m | 0.46 m | 1.00 m |
| 05_hover_rotate        | 1.64 m ✗ | 0.30 m | 0.80 m |

Option B (MAVSDK) doesn't have a built-in test runner — it needs a live
`mavsdk_server` connected to PX4 (SITL or real), and the code-level checks
it would run are already covered by the Option C runner plus non-MAVSDK
unit checks inside `realtime_swarm_mavsdk.py` (config loading, rotation
math, CSV leader replay, shortest-angle heading rate). End-to-end Option B
validation is done against the SITL setup documented in §12.

### 9.3 Running

```bash
# Offline regression suite (fast, ~2 s)
python3 _gen_test_inputs.py            # one-time, regenerates test_inputs/
python3 run_tests.py                   # all five scenarios
python3 run_tests.py --filter spiral
python3 run_tests.py -v

# Real-time regression suite (wall-clock, ~85 s for 4 scenarios)
python3 run_realtime_tests.py
python3 run_realtime_tests.py --filter hover
```

Both runners exit non-zero on any scenario failure.

### 9.4 `meta.json` expectation schema

```json
{
  "name": "01_basic_spear",
  "description": "...",
  "expect_simulation": "ok",          // or "fail" for intentionally infeasible
  "expect_peak_err_max": 1.5,         // optional upper bound on max tracking error (m)
  "expect_min_sep_min": 1.0,          // optional lower bound on min pairwise separation (m)
  "follower_yaw": "leader"            // optional, forwarded to generator
}
```

The offline runner parses `simulate_swarm.py`'s stdout (regex over the
report block) and compares against these bounds. The real-time runner uses
its own `TRACKING_TOL_M` table keyed by scenario name. Tighten either bound
to convert a scenario into a sharper regression check.

### 9.5 Adding a scenario

1. Add a `scenario_NN_<name>(root)` function in `_gen_test_inputs.py` that
   writes `drones_config.json`, `formation.csv`, `leader_path.csv`, and
   `meta.json` (plus `limits.json` if custom).
2. Call it from `main()`.
3. Run `python3 _gen_test_inputs.py && python3 run_tests.py &&
   python3 run_realtime_tests.py`.
4. If the scenario tolerance needs a non-default for real-time, add an
   entry to `TRACKING_TOL_M` in `run_realtime_tests.py`.

---

## 10. Limitations

### 10.1 What every tier shares

- **Geometric, rigid formation.** Offsets are fixed per scenario; the
  formation CSV is static. No morphing shapes or dynamic role changes.
- **Single leader.** `drones_config.json` must have exactly one
  `is_leader: true`.
- **Shared NED origin.** All drones must fly in the same local NED frame.
  For real flight across distances you'd extend Option B to broadcast
  LLA and convert per follower, like `src/drone_config.py`.
- **No obstacle awareness.** Drones avoid each other only to the extent
  that their formation offsets keep them apart. There is no live sensing
  or path-planning around external obstacles.

### 10.2 Offline planner + simulator

- Geometric and kinematic; no attitude dynamics, rotor limits, wind,
  propwash, sensor noise, or estimator lag. The simulator captures
  infeasible commanded speeds / accelerations / yaw rates, but it does
  not replace a real SITL or hardware run for final validation.

### 10.3 Option C — loopback real-time

- **Loopback UDP only.** Leader and followers run on one machine; we don't
  open any port other than `127.0.0.1`. Crossing machines would need
  unicast/multicast bind addresses + packet-loss / reordering handling
  (current extrapolator tolerates drop-1-packet, not drop-10).
- **JSON wire format.** Easy to eyeball, higher overhead than a binary
  struct. Fine for 10 Hz × a few drones; swap for struct-packed bytes if
  you push rate or fleet size up.
- **Followers still use the kinematic PD integrator**, so Option C results
  reflect our model of a drone, not an actual drone.
- **Exit-on-silence.** Followers simply quit after 2 s of no leader
  packets. In real flight you want hover-in-place + alarm, not process
  exit.

### 10.4 Option B — centralized MAVSDK

- **Single process = single point of failure.** If the controller process
  dies, every follower falls back on whatever PX4 does when offboard
  stream stops (typically hover, then RTL if `COM_OBL_ACT` is set). Option D
  (§16) was built specifically to address the *logical* single-leader half
  of this (no fixed master to lose) by electing a new master among
  survivors; it does not remove the *process* single point of failure, since
  it is still one process computing every drone's election + flocking (§16.14).
- **No onboard saturation or PD here.** We rely on PX4's position
  controller and whatever parameter limits (`MPC_XY_VEL_MAX`, etc.) you
  have set. Make sure those match your airframe before flying.
- **Shared NED origin assumption** still applies. Multi-home real-world
  deployments need LLA broadcasts — not yet implemented.
- **Not validated end-to-end in this environment.** The non-MAVSDK code
  paths (config, rotation, feed-forward, CSV leader replay, shortest-angle
  heading rate) are unit-tested; the MAVSDK path needs a running
  `mavsdk_server` + PX4 SITL or vehicle to exercise. See §12 for the SITL
  setup.

---

## 11. Files on disk

```
Simple_swarm/
├── README.md                    # this file
│
├── generate_trajectories.py     # offline: planner
├── simulate_swarm.py            # offline: kinematic simulator
├── visualize_trajectories.py    # static 4-panel plot of any trajectory dir
├── animate_trajectories.py      # 3D GIF animator of any trajectory dir
│
├── realtime_protocol.py         # option C: UDP/JSON schema + port convention
├── realtime_filters.py          # option C: optional α-β smoother (opt-in)
├── realtime_leader.py           # option C: CSV replayer → UDP broadcaster
├── realtime_follower.py         # option C: UDP listener + PD tracker
├── run_realtime_demo.py         # option C: orchestrator
│
├── realtime_swarm_mavsdk.py     # option B: centralized MAVSDK controller
│                                 #   (also option D's connect/arm/takeoff/
│                                 #    shutdown/logging plumbing, reused via import)
│
├── swarm_election.py            # option D: election/flocking engine, no MAVSDK dependency (§16.3)
├── realtime_swarm_dynamic.py    # option D: MAVSDK entrypoint, dynamic election + flocking control loop
├── phase1_gazebo_single_vehicle.py       # option D: single-vehicle Gazebo lifecycle validation
├── phase1_offboard_loss_followup.py      # option D: real mavsdk_server-kill link-loss validation
├── gz_worlds/                   # option D: Gazebo worlds + launch scripts (§16.12)
│   ├── multi_x500_static.sdf
│   ├── obstacle_test.sdf
│   ├── launch_multi_gazebo_static.sh
│   └── README.md
├── tests/                       # option D: unit tests for swarm_election.py (§16.13)
│   ├── run_all.sh
│   └── test_*.py
│
├── _gen_test_inputs.py          # regenerates test_inputs/
├── run_tests.py                 # offline regression runner
├── run_realtime_tests.py        # option-C regression runner (wall-clock)
│
├── test_inputs/
│   ├── 01_basic_spear/          (drones_config.json, formation.csv, leader_path.csv, meta.json)
│   ├── 02_circle_auto_heading/
│   ├── 03_vertical_spiral/
│   ├── 04_infeasible_tight_limits/   (+ limits.json)
│   └── 05_hover_rotate/
│
├── output/
│   ├── drone_*.csv                   # default planner output
│   ├── simulated/                    # default simulator output
│   ├── realtime/                     # default option-C demo output
│   └── mavsdk_realtime/              # default option-B controller output
│
├── test_outputs/                     # offline runner artefacts, per scenario
│   └── <scenario>/{generated, simulated}/
│
└── test_outputs_realtime/            # option-C runner artefacts, per scenario
    └── <scenario>/drone_*.csv
```

---

## 12. CLI cheat sheet

### 12.1 Offline tier

```bash
# plan
python3 generate_trajectories.py \
  --config      test_inputs/01_basic_spear/drones_config.json \
  --formation   test_inputs/01_basic_spear/formation.csv \
  --leader-path test_inputs/01_basic_spear/leader_path.csv \
  --output-dir  output \
  [--follower-yaw leader|motion] \
  [--plot]

# visualize (static) — works on any trajectory dir
python3 visualize_trajectories.py --trajectories-dir output --save-dir output --no-show

# animate (planned motion only)
python3 animate_trajectories.py --trajectories-dir output \
  --output output/swarm_animation.gif --fps 15 --trail-length 25

# simulate (kinematic playback + safety checks)
python3 simulate_swarm.py \
  --trajectories-dir output \
  --output-dir output/simulated \
  [--limits my_drone.json] \
  [--plot]            # commanded_vs_simulated.png (3D overlay + error-vs-time)
  [--animate]         # commanded_vs_simulated.gif (hollow=cmd, filled=achieved)

# offline regression tests
python3 _gen_test_inputs.py
python3 run_tests.py
```

### 12.2 Option C — loopback UDP demo

```bash
# one-shot (orchestrator spawns leader + all followers)
python3 run_realtime_demo.py \
  --config      test_inputs/01_basic_spear/drones_config.json \
  --formation   test_inputs/01_basic_spear/formation.csv \
  --leader-path test_inputs/01_basic_spear/leader_path.csv \
  [--leader-rate 10] [--sim-rate 50] [--visualize] \
  [--smooth] [--smooth-alpha 0.5] [--smooth-beta 0.1]     # optional α-β (§7.6)

# process-by-process (for debugging — separate terminals)
python3 realtime_follower.py --config ... --formation ... --hw-id 2 --output /tmp/f2.csv &
python3 realtime_follower.py --config ... --formation ... --hw-id 3 --output /tmp/f3.csv &
python3 realtime_leader.py   --config ... --formation ... --leader-path ... \
                             --rate 10 --output /tmp/leader.csv

# real-time regression tests (wall-clock ~85 s)
python3 run_realtime_tests.py
python3 run_realtime_tests.py --filter hover
```

### 12.3 Option B — centralized MAVSDK controller

> **For a step-by-step SITL setup from scratch, see §14 — the end-to-end
> walkthrough.** For a one-click Tkinter-based launcher wrapping the
> same stack, see §15 (`swarm_gui.py`). This subsection is the terse
> CLI summary.

Prerequisites per drone: `mavsdk_server` running on 127.0.0.1:(50040+hw_id)
and connected to its PX4 (SITL or real vehicle) via MAVLink UDP. Use
`sihsim_quadx` for headless SITL (`none_iris` won't arm; Gazebo Classic
multi-SITL is broken — see §14.2 troubleshooting).

```bash
# SITL setup (one-time; from the existing repo tooling)
cd ../multiple_sitl/PX4-Autopilot && make px4_sitl_default
cd ../../Simple_swarm
../multiple_sitl/multiple_sitl.sh -n 5 -m iris      # 5 PX4 instances — BROKEN on Gazebo Classic 11; use §14.4 SIH instead

# start mavsdk_server per drone (separate terminals)
../mavsdk_server -p 50041 udp://:14541 &
../mavsdk_server -p 50042 udp://:14542 &
../mavsdk_server -p 50043 udp://:14543 &
../mavsdk_server -p 50044 udp://:14544 &
../mavsdk_server -p 50045 udp://:14545 &

# dry-run first — connects + reads telemetry, NO arm/takeoff/offboard
python3 realtime_swarm_mavsdk.py \
  --config        test_inputs/01_basic_spear/drones_config.json \
  --formation     test_inputs/01_basic_spear/formation.csv \
  --leader-source csv \
  --leader-path   test_inputs/01_basic_spear/leader_path.csv \
  --duration 15 --dry-run

# full run — arm, takeoff, fly formation, land
python3 realtime_swarm_mavsdk.py \
  --config        test_inputs/01_basic_spear/drones_config.json \
  --formation     test_inputs/01_basic_spear/formation.csv \
  --leader-source csv \
  --leader-path   test_inputs/01_basic_spear/leader_path.csv \
  --takeoff-alt 10 --control-rate 20 --duration 15 --land-on-exit

# if leader is a real drone, not a CSV replay
python3 realtime_swarm_mavsdk.py \
  --config ... --formation ... \
  --leader-source mavsdk \
  --takeoff-alt 10 --control-rate 20 --duration 30 --land-on-exit

# visualize what actually flew (MAVSDK telemetry logs)
python3 visualize_trajectories.py \
  --trajectories-dir output/mavsdk_realtime --save-dir output/mavsdk_realtime --no-show
python3 animate_trajectories.py \
  --trajectories-dir output/mavsdk_realtime \
  --output           output/mavsdk_realtime/achieved.gif
```

---

## 13. Tunable parameters summary

### 13.1 Planner (`generate_trajectories.py`)

- `YAW_MOTION_SPEED_FLOOR = 0.1 m/s` — below this the motion-tangent yaw
  falls back to leader heading.
- `DEFAULT_LED = (255, 255, 255)`, `LEADER_LED = (255, 64, 64)` — LED
  colors in the output CSV.
- `TRAJ_MODE_MANEUVER = 70` — mode code written into every output row.

### 13.2 Offline simulator (`simulate_swarm.py` — override via `--limits foo.json`)

Also reused verbatim by `realtime_follower.py` (Option C).

| Parameter | Default | Meaning |
|---|---|---|
| `max_speed_xy` | 10.0 m/s | horizontal cruise clamp |
| `max_speed_up` | 5.0 m/s  | climb clamp |
| `max_speed_down` | 3.0 m/s | descent clamp |
| `max_accel_xy` | 5.0 m/s² | horizontal acceleration clamp |
| `max_accel_z`  | 3.0 m/s² | vertical acceleration clamp |
| `max_yaw_rate` | 1.5708 rad/s (~90°/s) | yaw slew clamp |
| `kp_pos` | 4.0 | position gain |
| `kv_vel` | 3.0 | velocity gain |
| `kyaw`   | 4.0 | yaw gain |
| `sim_dt` | 0.02 s | integrator step |

### 13.3 Safety thresholds (hardcoded in simulator's `DEFAULT_THRESHOLDS`)

| Threshold | Default | Failure condition |
|---|---|---|
| `max_tracking_error` | 2.0 m | any drone's peak position error exceeds this |
| `min_separation` | 0.5 m | any pair of drones gets closer than this at any time |
| `min_altitude` | 0.0 m | any drone's altitude drops below this |

### 13.4 Option C — real-time demo

CLI flags (all have sensible defaults):

| Script | Flag | Default | Meaning |
|---|---|---|---|
| `realtime_leader.py`     | `--rate`                   | 10.0 Hz | broadcast rate |
| `realtime_follower.py`   | `--sim-rate`               | 50.0 Hz | inner PD loop rate |
| `realtime_follower.py`   | `--silence-timeout`        | 2.0 s   | exit on leader silence |
| `realtime_follower.py`   | `--first-packet-timeout`   | 15.0 s  | abort startup if leader never speaks |
| `realtime_follower.py`   | `--smooth`                 | off     | enable the α-β filter (see §7.6) |
| `realtime_follower.py`   | `--smooth-alpha`           | 0.5     | α-β position gain |
| `realtime_follower.py`   | `--smooth-beta`            | 0.1     | α-β velocity gain |
| `realtime_protocol.py`   | `REALTIME_PORT_BASE`       | 37540   | follower N → port (base + N) |

Real-time-runner tolerances (`run_realtime_tests.py::TRACKING_TOL_M`):

| Scenario | Tolerance on peak 3D tracking error (vs offline ground truth) |
|---|---|
| 01_basic_spear         | 0.60 m |
| 02_circle_auto_heading | 0.60 m |
| 03_vertical_spiral     | 1.00 m |
| 05_hover_rotate        | 0.80 m |

### 13.5 Option B — centralized MAVSDK controller

All passed via CLI flags on `realtime_swarm_mavsdk.py`:

| Flag | Default | Meaning |
|---|---|---|
| `--leader-source` | `mavsdk` | `mavsdk` (subscribe to leader drone telemetry) or `csv` (replay file) |
| `--leader-path`   | —        | required when `--leader-source=csv` |
| `--takeoff-alt`   | 10.0 m   | arm + takeoff altitude; 0 = skip |
| `--control-rate`  | 20.0 Hz  | setpoint push rate |
| `--duration`      | ∞        | how long to run the control loop; default = until Ctrl+C |
| `--port-base`     | 50040    | gRPC port base; follower N uses port (base + N). Matches `src/drone.py`. |
| `--first-state-timeout` | 30.0 s | abort if no leader state arrives within this window |
| `--dry-run`       | off      | connect + read + compute + log, but no arm/takeoff/offboard |
| `--land-on-exit`  | off      | issue `land()` on every drone on shutdown |

---

## 14. End-to-end walkthrough — PX4 SITL + MAVSDK + QGroundControl

The full path from "I have PX4 and QGC installed" to "four drones flying
a spear formation visible in QGC". Each step lists the command, what to
check, and what goes wrong.

### 14.1 Prerequisites (one-time)

You need:

- **PX4 built** at a known path. In this walkthrough we use
  `$HOME/PX4-Autopilot` — adjust to wherever yours is.
  ```bash
  cd $HOME/PX4-Autopilot
  make px4_sitl_default
  ```
  Takes ~15–20 minutes first time. Rebuild needed only after pulling new
  PX4 sources.

- **mavsdk_server binary** somewhere on disk. Download Linux x64 from
  https://github.com/mavlink/MAVSDK/releases (pick a release matching
  the pip `mavsdk` package version). In this repo it's already at
  `/home/senthilkumarl/mavsdk_drone_show/mavsdk_server`.

- **MAVSDK Python package**:
  ```bash
  pip install mavsdk
  ```

- **QGroundControl** installed (AppImage or packaged). Already set up if
  you've ever flown PX4 SITL on this machine.

- **Python deps for Simple_swarm**: `numpy pandas matplotlib`.

### 14.2 Step 1 — Verify the PX4 build

```bash
# PX4 binary
ls $HOME/PX4-Autopilot/build/px4_sitl_default/bin/px4

# SIH airframe — this is the one we want
ls $HOME/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes \
   | grep sihsim
```
Expect to see `10040_sihsim_quadx`. If missing, rebuild.

> **Why SIH (Simulator-In-Hardware) and not `none_iris` or Gazebo?**
> - `none_iris` launches PX4 but requires an external simulator on TCP
>   4560+N. Without one, `simulator_mavlink` blocks, GPS never populates,
>   arm always fails with "Preflight Fail: GPS not ok".
> - `Tools/simulation/gazebo-classic/sitl_multiple_run.sh` doesn't work
>   on Gazebo Classic 11 — the script calls `gz model --spawn-file=…`
>   which is Ignition syntax; models never spawn, every PX4 waits
>   forever.
> - `sihsim_quadx` bundles PX4's internal SIH physics loop.
>   `SENS_EN_GPSSIM / BAROSIM / MAGSIM` are all set in the airframe, so
>   sensors populate, EKF converges, and drones are actually arm-able.
>   Headless, no external dependencies.

### 14.3 Step 2 — Prepare scenario files

Under `Simple_swarm/test_inputs/<your_scenario>/` create three files:

**`drones_config.json`** — hw_id → MAVSDK UDP port mapping (`14540+hw_id-1`):
```json
{
  "drones": [
    {"drone_id": 1, "hw_id": 1, "ip": "udp://:14540", "port": 14540, "is_leader": true},
    {"drone_id": 2, "hw_id": 2, "ip": "udp://:14541", "port": 14541, "is_leader": false},
    {"drone_id": 3, "hw_id": 3, "ip": "udp://:14542", "port": 14542, "is_leader": false},
    {"drone_id": 4, "hw_id": 4, "ip": "udp://:14543", "port": 14543, "is_leader": false}
  ]
}
```

**`formation.csv`** — rigid body-frame offsets relative to the leader
(see §4.2 for conventions):
```
hw_id,follow,offset_n,offset_e,offset_alt
1,0,0,0,0
2,1,-3,-3,0
3,1,-5,0,0
4,1,-3,3,0
```

**`leader_path.csv`** — NED coords + heading over wall-clock time. See
§4.3 for column conventions; optional `heading` is degrees with 0°=N.
For a 40 m-radius circle over 45 s:
```python
import csv, math
rows = [["t","north","east","altitude","heading"]]
R, PERIOD, HOVER, ALT = 20.0, 40.0, 5.0, 15.0
t = 0.0
while t <= HOVER + PERIOD + 1e-6:
    alpha = 0.0 if t <= HOVER else 2*math.pi*(t-HOVER)/PERIOD
    n = R*math.cos(alpha); e = R*math.sin(alpha)
    hdg = 90.0 if t <= HOVER else 90.0 + 360.0*(t-HOVER)/PERIOD
    rows.append([round(t,3), round(n,3), round(e,3), ALT, round(hdg,3)])
    t += 0.5
with open("leader_path.csv","w",newline="") as f: csv.writer(f).writerows(rows)
```

### 14.4 Step 3 — Launch N PX4 SITL instances (headless, SIH)

Clean any residue, then launch one PX4 process per drone. Each instance
needs its own working directory (for `dataman`, params, logs) and a
unique `-i N`:

```bash
pkill -9 -f 'bin/px4 -i' 2>/dev/null; sleep 2

PX4_BIN=$HOME/PX4-Autopilot/build/px4_sitl_default/bin/px4
PX4_ETC=$HOME/PX4-Autopilot/build/px4_sitl_default/etc

for i in 0 1 2 3; do
  rm -rf /tmp/sitl_run/rootfs_$i
  mkdir -p /tmp/sitl_run/rootfs_$i
  nohup bash -c "cd /tmp/sitl_run/rootfs_$i && exec env \
    PX4_SYS_AUTOSTART=10040 \
    PX4_SIM_MODEL=quadx \
    PX4_SIMULATOR=sihsim \
    PX4_HOME_LAT=28.4523 PX4_HOME_LON=77.0695 PX4_HOME_ALT=200 \
    HEADLESS=1 \
    $PX4_BIN -i $i -d $PX4_ETC" \
    > /tmp/sitl_run/px4_sih_$i.log 2>&1 &
  sleep 3   # stagger startup so mavlink instances don't collide
done
```

Important:
- `-i N` = instance number. Drone hw_id = N+1. SysID = N+1 (auto-assigned).
- `-d <path>` = absolute path to the built `etc/` dir.
- `cd /tmp/sitl_run/rootfs_$i` BEFORE launch — PX4 writes `dataman`,
  `parameters.bson`, `log/` in CWD. Don't share rootfs between instances.
- **All drones spawn at the SAME GPS lat/lon** on purpose. Each PX4 SIH
  instance anchors its local NED origin at its own spawn GPS, and the
  swarm controller assumes a single shared NED frame (it sends every
  follower targets computed from the leader's NED). Staggering longitude
  by `+i*0.0005°` makes the drones start ~50 m apart in absolute space,
  which exceeds the ±5 m formation deltas — the formation then looks like
  a straight east-west line in QGC instead of a spear (see §14.13). In
  SIH each instance is its own isolated physics sim, so co-spawning has
  no collision cost; QGC simply shows them overlapped until they fly.
- `PX4_SYS_AUTOSTART=10040` + `PX4_SIM_MODEL=quadx` selects the
  `sihsim_quadx` airframe. Using the default `none_iris` instead would
  hang waiting for an external simulator on TCP 4560+N and trip the
  controller's "GPS not ok within 30s" timeout.
- Use `nohup bash -c "... exec env …"` — without `exec env`, the
  `env VAR=val` becomes a subshell command that may not propagate.
  Without `nohup`, the processes die when your terminal closes.

Verify:
```bash
pgrep -af 'bin/px4 -i'
# expect 4 processes (one per instance)

ss -ulnp 2>/dev/null | grep -E '1857[0-3]|1458[0-3]|1454[0-3]'
# expect PX4 listening on 18570+N (GCS local), 14580+N (MAVSDK local),
# and mavsdk_server on 14540+N once it's running (next step)
```

### 14.5 Step 4 — Launch N mavsdk_server processes

One gRPC bridge per drone, pairing a gRPC server port with the drone's
MAVLink UDP port:

```bash
MAVSDK=/path/to/mavsdk_server    # adjust
for i in 0 1 2 3; do
  $MAVSDK -p $((50041+i)) udp://:$((14540+i)) \
    > /tmp/sitl_run/mavsdk_$i.log 2>&1 &
done
```

Verify:
```bash
ss -lnp 2>/dev/null | grep mavsdk_server
# expect 4 UDP listeners on 14540..14543 and 4 TCP listeners on 50041..50044
```

Port mapping, authoritative source `px4-rc.mavlink` (§7 in this doc):
- drone N (instance i=N-1): gRPC `50040+N`, MAVLink UDP `14540+i` =
  `14540+N-1`.
- `drones_config.json` assumes this mapping — don't change port
  conventions without updating the config.

### 14.6 Step 5 — Wait for EKF heading convergence

PX4's EKF takes **5–15 seconds** after startup to stabilize heading
(during which `is_global_position_ok` returns false and arm will
reject). Wait until every instance has emitted "Ready for takeoff":

```bash
# poll every 2s until all 4 say Ready
until all_ready=true; for i in 0 1 2 3; do
  grep -q 'Ready for takeoff' /tmp/sitl_run/px4_sih_$i.log || all_ready=false
done; $all_ready; do sleep 2; done
```

Or just wait 15 seconds after the last PX4 launched — usually enough.

### 14.7 Step 6 — Set up QGroundControl

Open QGC. By default it has a UDP Auto-Connect link listening on
`:14550`. PX4 sends GCS heartbeats to that port; QGC should auto-discover
all drones within a few seconds.

**In QGC verify:**

1. Click the vehicle selector at the top-left. Dropdown should show 4
   entries (SysID 1, 2, 3, 4). If only some appear:
   - `Application Settings → General → "UDP Auto-Connect"` = ON.
   - If running PX4 without broadcast (default), restart QGC *after*
     PX4 — QGC's heartbeats then reach PX4's GCS ports and each drone
     replies.
2. Map view is centered on the drones. They spawn at `lat 28.4523°N`
   (or whatever `PX4_HOME_LAT` you set). Click any vehicle in the
   selector — QGC should snap to it.
3. Add a Values Panel at the bottom. Add Altitude, Horizontal Velocity,
   Yaw. These tick live during flight independent of map view.

**If still invisible:**

- Add explicit UDP comm links to each PX4 GCS local port
  (`Application Settings → Comm Links → Add → UDP → Port 18570/18571/…`).
  Less elegant than auto-discover but bulletproof.
- Check that QGC is actually listening: `ss -ulnp | grep QGround` should
  show `:14550`.

### 14.8 Step 7 — Dry-run the controller

Before arming anything, confirm plumbing end-to-end with `--dry-run`.
It connects, reads telemetry, computes setpoints, writes logs — but
does NOT arm/takeoff/offboard.

```bash
cd Simple_swarm
python3 realtime_swarm_mavsdk.py \
  --config        test_inputs/<your_scenario>/drones_config.json \
  --formation     test_inputs/<your_scenario>/formation.csv \
  --leader-source csv \
  --leader-path   test_inputs/<your_scenario>/leader_path.csv \
  --duration 10 --dry-run
```

Expected output:
```
[main] leader=1  followers=[2, 3, 4]
[hw_id 1] connecting on mavsdk_server :50041 ...
[hw_id 1] mavsdk connected
[hw_id 2] connecting on mavsdk_server :50042 ...
[hw_id 2] mavsdk connected
... (same for 3, 4)
[ctrl] will also push CSV setpoints to the leader drone (hw_id 1)
[ctrl] running at 20.0 Hz for 10.0s
[ctrl] control loop done — 200 ticks
[main] logs in .../output/mavsdk_realtime/
```

Check `output/mavsdk_realtime/drone_*_commanded.csv` — these contain
what the controller *would* have pushed. If the numbers look sensible
(leader position tracing the CSV, followers offset by the rotation),
the math is right.

### 14.9 Step 8 — Fly the swarm

Remove `--dry-run`, add `--takeoff-alt` and `--land-on-exit`:

```bash
python3 realtime_swarm_mavsdk.py \
  --config        test_inputs/<your_scenario>/drones_config.json \
  --formation     test_inputs/<your_scenario>/formation.csv \
  --leader-source csv \
  --leader-path   test_inputs/<your_scenario>/leader_path.csv \
  --takeoff-alt 15 --control-rate 20 --duration 132 --land-on-exit
```

You should see:
```
[hw_id 1] GPS ok
[hw_id 2] GPS ok
...
[hw_id 1] offboard takeoff → 15.0 m
[hw_id 2] offboard takeoff → 15.0 m
...
[ctrl] will also push CSV setpoints to the leader drone (hw_id 1)
[ctrl] running at 20.0 Hz for 132.0s
[ctrl] control loop done — 2640 ticks
[main] logs in .../output/mavsdk_realtime/
```

In QGC: **all 4 vehicles** should move — leader tracing the planned
path, followers maintaining spear formation around it. Watch the
altitude / velocity values in the Values Panel to confirm motion.

> **Why all 4 move (and not just the followers):**
> With `--leader-source csv`, the controller does two things
> simultaneously:
> 1. Subscribes to the real leader drone's MAVSDK telemetry →
>    populates `LeaderState` → drives **follower** setpoints so they
>    formation-lock onto where the leader actually is.
> 2. Interpolates the CSV path at current wall-clock time → drives the
>    **leader** drone along the planned path via offboard setpoints.
>
> This means followers track the real leader (not the idealized CSV),
> which eliminates the "formation offset from real leader" issue that
> occurs if you compute follower targets off the CSV directly.

### 14.10 Step 9 — Validate the formation

```bash
python3 validate_formation.py \
  --trajectories-dir output/mavsdk_realtime \
  --config      test_inputs/<your_scenario>/drones_config.json \
  --formation   test_inputs/<your_scenario>/formation.csv \
  --start-after 20.0 --tol-pos 2.5 --tol-heading 10.0
```

Checks two invariants:

1. **Formation position**: at every sample (after takeoff transient)
   each follower sat within `tol-pos` m of
   `leader_position + R(leader_heading) · offset_body`.
2. **Tip points to heading**: the vector from the tail (deepest-aft
   follower) to the leader aligns with the leader's heading within
   `tol-heading` degrees.

Expected numbers for real SITL with PX4 SIH at 20 Hz control:
- Peak position error during steady-state: 1–2 m
- RMS position: 0.5–1 m
- Peak heading alignment during continuous rotation: 5–15°
- Peak errors spike 2–4 m during the first few seconds after takeoff
  (PD lag while quads accelerate to the rotating formation)

### 14.11 Step 10 — Visualize

```bash
python3 visualize_trajectories.py \
  --trajectories-dir output/mavsdk_realtime \
  --save-dir output/mavsdk_realtime --no-show

python3 animate_trajectories.py \
  --trajectories-dir output/mavsdk_realtime \
  --output output/mavsdk_realtime/achieved.gif
```

Outputs:
- `swarm_trajectories.png` — 4-panel static plot: 3D paths, top-down,
  altitude vs time, formation-in-leader-translated-frame (best shows
  rotation).
- `achieved.gif` — 3D animation with fading trail + heading arrows.

### 14.12 Step 11 — Cleanup

```bash
for p in $(pgrep -f 'bin/px4 -i'); do kill $p; done
pkill -x mavsdk_server
# Close QGC from the UI, or pkill QGroundControl
```

### 14.13 Troubleshooting (real failures seen in the wild)

| Symptom | Cause | Fix |
|---|---|---|
| `TimeoutError: hw_id N: no mavsdk_server on 127.0.0.1:5004N` | `mavsdk_server` not running on that port | Check `ss -lnp \| grep mavsdk_server`. Restart missing one. |
| `TimeoutError: hw_id N GPS not ok within 30.0s` | EKF not converged yet, or PX4 using `none_iris` (no sim sensors) | Wait longer, or switch to `sihsim_quadx`. |
| `OffboardError: NO_SETPOINT_SET` | `offboard.start()` called without a prior `set_position_ned()` | Confirmed `arm_and_offboard_takeoff` seeds a position setpoint before `start()`. |
| `Preflight Fail: heading estimate not stable` (in PX4 log) | EKF hasn't finished initial convergence | Wait ~15 s after PX4 starts. Normal during boot. |
| QGC shows 0 vehicles | QGC restarted after PX4; drones latched onto old QGC partner IP; or `MAV_0_BROADCAST=0` so PX4 only replies to whoever pinged it first | Simplest fix: `realtime_swarm_mavsdk.py` now pushes `MAV_0_BROADCAST=1` via MAVSDK after each connect — heartbeats broadcast to the whole subnet. Alternatively restart PX4 after QGC is running, or add explicit UDP comm links in QGC for each PX4 GCS port (`18570+N`). |
| **Formation looks like a straight east-west line in QGC (instead of spear/triangle)** | Drones spawned at different GPS longitudes (`+i*0.0005°` was the old pattern → ~50 m apart). The controller sends follower targets in LEADER's NED frame, but each follower anchors its own NED at ITS OWN spawn. So follower-2's "go to (−3, −3)" is relative to follower-2's spawn, which is 50 m east of leader's spawn. Net effect: drones are ~50 m apart along E axis with ±5 m wiggles — a line | Spawn every drone at the **same** GPS lat/lon. `start_multi_px4.sh` now does this by default. The validator (`validate_formation.py`) will still PASS in the broken case because it compares actual-NED vs expected-NED within each drone's own frame; only QGC (absolute world view) reveals the line. For real hardware, where drones naturally have different homes, the controller needs a home-offset transform — open question, out of scope for SIH. |
| QGC shows vehicle 1 but it never moves | `--leader-source csv` without the leader-drives path (old versions); or arming silently failed on drone 1 | Verify `realtime_swarm_mavsdk.py` line that says `will also push CSV setpoints to the leader drone (hw_id 1)`. That's the drive-leader log. If missing, update the controller. |
| "Waypoints out of sequence" warning in QGC | `drone.action.takeoff()` creates a stranded takeoff mission item; QGC's mission sync complains with 4 drones racing | Use `arm_and_offboard_takeoff()` (does NOT call `action.takeoff()`); it seeds offboard and arms directly. |
| PX4 exits immediately with "Exiting NOW" | Stale process holding port, or rootfs locked by another instance | `pkill -9 -f 'bin/px4'` + `rm -rf /tmp/sitl_run/rootfs_*` + relaunch. |
| `sitl_multiple_run.sh` spawns nothing | Gazebo Classic 11 doesn't support `gz model --spawn-file=…` | Don't use that script. Use the `sihsim_quadx` loop in Step 14.4. |
| `cannot join current thread` / `_MultiThreadedRendezvous CANCELLED` at Python exit | mavsdk/aiogrpc destructor cleanup during interpreter shutdown | Harmless. The flight already completed by the time you see it. |
| Tracking error much worse than expected | `--leader-source csv` without the leader being flown; followers chase idealized leader while real drone 1 sits on ground | Confirm the controller version includes `drive_leader = True` path, AND the leader drone appears as armed+in-air in QGC. |

### 14.14 Expected end-state — what "working" looks like

After running the full walkthrough you should have:

- **4 PX4 instances** running headless with SIH physics (`pgrep px4` → 4).
- **4 mavsdk_server instances** bridging gRPC ↔ MAVLink (`pgrep mavsdk_server` → 4).
- **QGC** showing 4 vehicles in the selector, all reporting armed → in-air → landed states in sync.
- **`output/mavsdk_realtime/`** containing per-drone CSVs (`drone_<N>.csv` achieved, `drone_<N>_commanded.csv` what was sent) plus `swarm_trajectories.png`.
- **`validate_formation.py`** reporting PASS on both position (≤2.5 m) and heading alignment (≤10°) checks.
- A **4-panel plot** where the bottom-right "formation in leader-translated frame" shows all 8 snapshot polygons nearly overlapping — the invariant proof that the spear tip always points along leader heading.

## 15. GUI shortcut (`swarm_gui.py`)

§14 is the manual recipe. For day-to-day use, a Tkinter + matplotlib GUI
at `Simple_swarm/swarm_gui.py` wraps the whole thing into one click.

### 15.1 What it does

Three panes in one window:

- **Scenario browser (left)** — lists every subdirectory of `test_inputs/`
  that has the three required files (`drones_config.json`, `formation.csv`,
  `leader_path.csv`). Clicking a scenario loads a summary: drone count,
  leader ID, leader path duration, altitude range, and the description
  from `meta.json` if present. Below are input fields for takeoff
  altitude, control rate, duration (blank = match leader path), and
  checkboxes for dry-run / land-on-exit.
- **3D preview (right)** — the selected scenario's leader path in yellow
  plus the follower polygon rendered at three samples along the path
  (start / mid / end). Each polygon's offsets are rotated by the leader
  heading at that sample, so the "formation rotates with heading in the
  N-E plane" invariant is visually verifiable **before flying**. Mouse
  rotate / zoom via the matplotlib toolbar.
- **Orchestration log (bottom)** — streaming stdout from every stage of
  the launch. Every line is also teed to `/tmp/swarm_gui.log`
  (line-buffered, truncated on each session) so you can share a failing
  run by pasting `tail -200 /tmp/swarm_gui.log`. "Copy log path" button
  puts that path on the clipboard.

### 15.2 Simulate / Stop buttons

**Simulate ▶** runs, in a worker thread so the GUI stays responsive:

1. `pkill -9 -f 'bin/px4 -i'` + `pkill -9 -f 'mavsdk_server'`
   and `rm -rf /tmp/px4-* /tmp/sitl_run/rootfs_*` — scrubs stale state
   so the next boot doesn't inherit a parameter store from a
   different-airframe run.
2. `bash start_multi_px4.sh N` — N read from the scenario config.
3. Waits 30 s + 5 s per extra drone (SIH boot + EKF headroom).
4. Spawns N `mavsdk_server` processes (gRPC `50040+hw_id`, MAVLink
   `udp://:14540+hw_id−1`); stdout/stderr go to
   `/tmp/mavsdk_server_<hw_id>.log`.
5. Launches `realtime_swarm_mavsdk.py --leader-source csv` with the
   scenario files + GUI knobs, streaming its output.

**Stop ■** kills the controller, every `mavsdk_server`, and every PX4
launched by the current GUI session, in that order.

### 15.3 Observability added for GUI use

So that log-pane output actually tells you what flew, the controller
emits:

- `[hw_id N] MAV_0_BROADCAST=1 set (QGC will auto-discover)` — confirms
  PX4's heartbeats are going out to the subnet. If this is missing,
  QGC won't auto-discover the vehicle.
- `[hw_id N] offboard takeoff done: in_air=True armed=True mode=FlightMode.OFFBOARD alt=14.92m`
  — the four booleans/values prove the drone is actually flying.
  If you see `in_air=False` or `alt=0.00m`, the takeoff didn't
  engage even though the controller claimed success.
- `[status] t=  4.0s  L(hw1): N=+20.1 E=-0.3 alt=+15.0 yaw=+90°  F2: alt=+15.0  F3: alt=+15.0  F4: alt=+15.0`
  every 2 s — a live position snapshot. If leader N/E never change
  or follower alts stay 0, the sim isn't actually moving regardless
  of what QGC shows.

### 15.4 Prerequisites beyond §14.1

- `python3-tk` system package (`sudo apt install python3-tk`).
- `gnome-terminal` (or `xfce4-terminal` / `xterm`) to host the PX4
  consoles; `start_multi_px4.sh` falls back to headless-background if
  none is present but you lose visibility into per-instance PX4 output.

### 15.5 What the GUI does NOT do

- Does not start QGroundControl — launch it separately as in §14.7.
- Does not run `validate_formation.py` after the flight — that's still
  a manual step (see §14.10).
- Does not handle real hardware — it assumes the SITL launcher and the
  SIH airframe. For real drones you'd drive `realtime_swarm_mavsdk.py`
  directly (no launcher, connect `mavsdk_server` to the vehicle's
  serial/UDP, and skip steps 1–4).

### 15.6 Scenario preview math (informational)

The right-pane 3D plot is built from exactly the same rotation math as
the controller, so "what the plot shows" = "what the flight will execute":

```python
# for each sample time t along the leader path:
for each follower in formation[follow=leader_id]:
    on, oe, oalt = follower.body_offset
    th = radians(leader_path.heading[t])
    rot_n = on*cos(th) - oe*sin(th)
    rot_e = on*sin(th) + oe*cos(th)
    world = (leader_path.north[t] + rot_n,
             leader_path.east[t]  + rot_e,
             leader_path.altitude[t] + oalt)
```

If the 3D preview shows the triangle swinging with the leader's heading
(visible because three samples are drawn at different headings), that's
the invariant. If the actual flight in QGC looks different, the
comparison hierarchy is:

1. **Is the preview triangle itself wrong?** — bug in formation.csv
   (body-frame offsets don't form the intended shape).
2. **Does the log `[status]` line show the right NED positions but QGC
   shows a straight line?** — drones spawned at different GPS origins
   (see §14.13).
3. **Log shows alt=0 / in_air=False?** — arming or offboard didn't
   engage; inspect `/tmp/mavsdk_server_<N>.log` and the PX4 console
   window for that drone.

---

## 16. Option D — Dynamic master election + three-force flocking

Option B (§7.3/§10.4) has exactly one leader, fixed in `drones_config.json`
for the life of the process; if that drone (or the controller process
computing its setpoints) goes away, nothing re-elects a replacement.
Option D removes the *fixed-leader* half of that: there is no `is_leader`
field at all, no designated drone — every drone runs the same decentralized
suitability-score election with hysteresis every control tick, and whichever
currently-reachable drone has the best score is master. The architecture
brief this was built from explicitly frames it as updating a "single process
= single point of failure" fixed-master design — Option B's own §10.4 wording
is almost verbatim what motivated this.

What Option D does **not** change from Option B: still one Python process
per swarm, still `mavsdk_server` per drone, still `offboard.set_position_velocity_ned()`
as the actuation path, still no onboard PD/saturation (PX4's position
controller does that). It reuses Option B's connect/arm/takeoff/shutdown/
logging plumbing directly via `import realtime_swarm_mavsdk` rather than
duplicating it.

### 16.1 Architecture

```
                 realtime_swarm_dynamic.py (ONE process)
              ┌────────────────────────────────────────────────┐
              │ connect_drone() / arm_and_offboard_takeoff()    │  ← from realtime_swarm_mavsdk.py
              │ per hw_id in config (no is_leader field used)   │
              │                                                  │
              │ per-drone telemetry pollers → DroneState         │
              │   (pos, vel, energy, compute_capacity, ...)      │
              │                                                  │
              │ control loop @ --control-rate Hz, per tick:      │
              │   comm_graph()         LoRa-range adjacency      │
              │   update_neighbor_link_state()   decay/throttle  │
              │   update_isolation_rth()          degraded/RTH   │
              │   clusters()           connected components      │
              │   ElectionState.run_election()    per cluster    │
              │     └─ eligibility_vote() → suitability_score()  │
              │        → tie_break_key()                         │
              │     └─ merge detection → MergeState (§16.9)      │
              │   flock_force()        per drone (§16.7)         │
              │   offboard.set_position_velocity_ned(...)        │
              │                                                  │
              │ graceful_shutdown(): stop offboard, optional land │
              └────────────────────────────────────────────────┘
                 │        │        │              │
                 ▼        ▼        ▼              ▼
              mavsdk_srv mavsdk_srv mavsdk_srv  mavsdk_srv
                 │        │        │              │
               PX4(1)   PX4(2)   PX4(3)   ...   PX4(N)
              (SITL or real vehicles)
```

### 16.2 Why "dynamic" — periodic *and* event-driven re-election

Two separate triggers call `ElectionState.run_election()` every tick's
cluster loop:

- **Periodic**: every drone's suitability score is recomputed and the best
  candidate re-checked on a fixed cadence (independent of faults — this is
  what keeps a drone from permanently monopolizing the master role as its
  own energy/position/connectivity factors change over a long mission).
- **Event-driven**: a drone going unreachable, isolated, or RTH-triggered
  removes it from `eligible_ids` immediately rather than waiting for the
  next periodic check.

Hysteresis (`MASTER_SWITCH_MARGIN=0.05`, `MASTER_MIN_HOLD_TIME=5.0s`) keeps
a narrowly-better candidate from flapping the master role back and forth —
a switch only happens if the challenger's score beats the incumbent's by
more than the margin, and (for event-driven triggers specifically) the
incumbent has held the role for at least the minimum hold time.

### 16.3 Files

| File | Role |
|---|---|
| `swarm_election.py` | Pure logic, **no MAVSDK import at all** — election, eligibility, flocking, comm graphs, merge reconciliation, degradation tiers, RTH. Importable and unit-testable with no SITL running. |
| `realtime_swarm_dynamic.py` | The MAVSDK entrypoint: CLI, per-drone telemetry pollers, the control loop, fault-injection flags, CSV/event logging. |
| `realtime_swarm_mavsdk.py` | Shared with Option B: `connect_drone()`, `arm_and_offboard_takeoff()`, `wait_gps_ok()`, `force_qgc_broadcast()`, `telemetry_logger_task()`, `graceful_shutdown()`. |
| `gz_worlds/` | Gazebo worlds + launch tooling for validating Option D specifically (multi-vehicle election/flocking, obstacle avoidance) — see §16.12. |
| `tests/` | 28 unit-test assertions against `swarm_election.py` directly, no SITL needed — see §16.13. |
| `phase1_gazebo_single_vehicle.py`, `phase1_offboard_loss_followup.py` | Standalone single-vehicle Gazebo validation scripts (lifecycle, setpoint-lapse, and real `mavsdk_server`-kill link-loss behavior) that informed Option D's design but don't depend on `swarm_election.py` themselves. |

### 16.4 Election: suitability score, eligibility, tie-break

`suitability_score()` (one call per cluster per election) computes, per
candidate `i`:

```
S_i = w_E·E_i + w_P·P_i + w_C·C_i + w_L·L_i

E_i = clip(energy_i, 0, 1)                                    # battery, 0..1
P_i = clip(1 - |pos_i - cluster_centroid| / (comm_range_lora * max_relay_hops), 0, 1)
C_i = clip(verified_neighbors_i / (|cluster| - 1), 0, 1)       # fraction of cluster mates
                                                                # it can reach AND that passed
                                                                # the eligibility vote
L_i = clip(compute_capacity_i, 0, 1)                           # companion-PC headroom
```

Default weights: `w_E=0.40, w_P=0.25, w_C=0.25, w_L=0.10`. Ties (and
near-ties within the hysteresis margin) are broken deterministically by
`tie_break_key()`: `(score, verified_neighbor_count, energy, -hw_id)` —
highest score wins, then most-verified-neighbors, then most energy, then
lowest hw_id, so every drone computing this independently always agrees.

**Hard eligibility floor**: `rth=True` is excluded from `eligible_ids`
entirely — a drone mid-return-to-launch cannot become (or remain) master,
found as a live bug (it could otherwise reconnect mid-flight-home and win a
merge's tie-break). `degraded_mode` alone does **not** exclude a drone: an
isolated drone is its own singleton cluster by definition and needs to stay
its own master to keep operating autonomously.

**Still soft, not hard** (a known, named gap against the original
architecture brief, which specified these as hard pre-score cutoffs): a
minimum energy reserve and a minimum connectivity floor. Both are *inputs*
to the score (`w_E·E_i`, `w_C·C_i`) but neither has a hard cutoff — this
needs a policy threshold decision that hasn't been made, not an
implementation gap.

### 16.5 Eligibility vote — pairwise distance-consistency (Byzantine/spoofing detection)

`eligibility_vote()` checks, for every directly-connected pair in a
cluster: does the pair's *reported* position difference match the
*measured* distance between them (within `eligibility_tau=2.0m`)? If not,
**both** endpoints of that one inconsistent pair lose a vote (not just the
"guilty" one — there is no independent arbiter to decide which one is
lying from the pair alone). A drone that fails too many such checks against
its neighbors is excluded from `eligible_ids`.

`measured_distance()` is the one function standing in for a real ranging
sensor: `ranging_available=False` by default, and setting it `True` without
a real sensor behind it (Tomoto/LoRa has none, confirmed) raises loudly at
call time rather than silently lying to the gate.

**Known, named limitation**: because both ends of a noisy pair lose a vote,
a single honest drone paired against one genuinely spoofing neighbor can
become collaterally ineligible alongside the spoofer, in the degenerate
2-drone-cluster case especially. Attributing fault to just the liar would
need either a third independent peer's corroborating measurement or an
external arbiter — neither exists yet.

### 16.6 Communication model — LoRa-primary, WiFi-optional

Built around the project's actual chosen radio hardware: an in-house LoRa
module ("Tomoto") for drone-to-drone comms, with an optional WiFi module on
the companion PC as a pure accelerant.

- `comm_range_lora` (default 30m in `realtime_swarm_dynamic.py`, 150m in
  `ElectionParams`'s own default) is the **primary, required** link —
  election, eligibility voting, and flocking's comm graph are all built on
  it alone. Nothing may assume WiFi is present.
- `comm_range_wifi` (default 0 = absent) is an **optional accelerant only**:
  currently used solely to speed up a merge's bulk `D_merged` payload
  transfer (§16.9) when the winner and loser also happen to be in WiFi
  range of each other. Everything else ignores it entirely.
- `max_relay_hops=1`: Tomoto is confirmed broadcast-only with no addressed
  routing layer, so a drone can only directly hear others within
  `comm_range_lora` — there is no multi-hop relay to model, and 1 is the
  physically correct value here, not a placeholder.
- `lora_broadcast_interval` (default 0 = continuous/live, a WiFi-like
  idealization) models Tomoto's real, much lower broadcast rate when set
  `>0`: a drone only actually *receives* a given neighbor's latest state
  every `lora_broadcast_interval` seconds, not every tick — `flock_force()`
  then reads throttled, possibly-stale state for still-in-range neighbors,
  not just post-disconnect ones. `update_neighbor_link_state()` must be
  called once per tick for the throttle to take effect.

**Known, named limitation**: `lora_bandwidth_bps` (1000, a conservative
placeholder) and Tomoto's real duty-cycle/airtime budget are both
uncharacterized against actual hardware — these parameters exist so the
timing *mechanism* (merge payload chunking, broadcast throttling) can be
built and tested now, not because the numbers are validated.

### 16.7 Three-force local flocking + obstacle avoidance

`flock_force()` — independent of election state, runs for every healthy
drone every tick regardless of who's master — sums:

```
total = f_goal + f_rep + alignment_gain·f_align + f_coh + f_obs      (normal)
total = f_goal + f_rep + f_obs                                       (degraded_mode: no cohesion/alignment)
```

- **f_goal**: unit vector toward the drone's own goal.
- **f_rep** (repulsion, safety-critical): active within `D_rep=8.0m`,
  magnitude `repulsion_gain·(D_rep - d)/max(d, 0.5)`. Weighted by each
  neighbor's decayed `neighbor_influence` (§16.8). Exact-zero-distance
  neighbors get a deterministic (hash-based, not random, so runs stay
  reproducible) escape direction rather than a zero force — two coincident
  vehicles need *some* direction to separate along.
- **f_coh** (cohesion): active beyond `cohesion_start=10.0m`, capped at
  `cohesion_max=40.0m`, averaged over contributing neighbors weighted by
  influence (a fully-decayed stale neighbor contributes 0 to both the
  numerator *and* the averaging denominator, not just the numerator).
- **f_align** (alignment): active within `alignment_max_dist=25.0m`, pulls
  velocity toward neighbors', weighted by both distance and influence.
- **f_obs** (obstacle avoidance, `obstacle_force()`): same force-law shape
  as `f_rep`, against `--obstacle N:E:RADIUS` positions passed on the CLI.
  Obstacle positions are **known/configured, not sensed live** — no
  lidar/depth-camera integration exists yet; swapping in real onboard
  sensing is a separate, future step. Same deterministic-escape-direction
  treatment at the obstacle's exact center.

Total force is clamped to `max_accel=3.0 m/s²`, and the resulting velocity
to `max_speed=4.0 m/s`, before being sent as the offboard setpoint.

**Known, named limitation**: no damping/braking term near the goal —
`f_goal` stays full-magnitude at every nonzero distance and only reaches
zero exactly *at* the goal, so a drone arriving with nonzero velocity
overshoots and oscillates rather than settling. Observed directly in a real
multi-drone Gazebo flight (§16.12).

### 16.8 Link-failure handling: neighbor decay, degradation tiers, isolation → RTH

Ported from the companion research project's already-validated abstract
simulation (`swarm_sim_core.py`, a separate 2D reference implementation —
not part of this repo), function-for-function:

- **Neighbor-influence decay** (`update_neighbor_link_state()`): a neighbor
  that drops out of `comm_range_lora` holds its last-known (frozen) state
  and ramps its flocking-force weight linearly to 0 over
  `NEIGHBOR_INFLUENCE_DECAY_WINDOW=1.0s`, rather than vanishing the instant
  it's unreachable — avoiding the force discontinuity a hard cutoff would
  introduce. Past `NEIGHBOR_STATE_TIMEOUT=2.0s` it's fully excluded.
  `--no-neighbor-decay` reproduces the old hard-cutoff behavior as an
  ablation baseline.
- **Degradation tiers** (`tier_of()`): classifies every drone each tick as
  `full` (direct link to its cluster's master), `relay` (in the main
  cluster, reaches the master only indirectly), `partition` (a smaller,
  valid sub-cluster, not the main one — e.g. both sides of a network split),
  or `isolated` (zero reachable neighbors). Surfaced in the live `[status]`
  log line.
- **Isolation → degraded_mode → RTH** (`update_isolation_rth()`): zero
  reachable neighbors sets `degraded_mode=True` immediately (flocking drops
  cohesion/alignment, §16.7); past `ISOLATION_RTH_TIMEOUT=6.0s` isolated,
  `rth=True` fires. `realtime_swarm_dynamic.py` watches this flag's
  False→True transition and calls the **real** MAVSDK
  `action.return_to_launch()` — not a simulated flag — with a bounded retry
  on failure (a failed call used to permanently strand the drone with no
  setpoints and no retry; fixed). Reconnecting clears `degraded_mode`,
  `isolated_since`, *and* `rth` — all three, matching the function's own
  docstring (a real bug where `rth` alone didn't clear was found and fixed
  via a test-quality review, §16.13).

### 16.9 Partition and merge reconciliation

When a comm-graph split later reconnects into one cluster containing more
than one distinct former master, `ElectionState.run_election()` detects the
merge (checked *before* any early-return branch — an earlier version
returned early when the merge winner happened to equal an arbitrary
"representative" former master, which re-detected the same merge every
tick forever; fixed) and creates a `MergeState` for the losing side: a
chunked bulk transfer of `merge_payload_bytes` (a placeholder — the real
task-allocation feature that would define this payload doesn't exist yet),
paced by `lora_bandwidth_bps` by default and sped up opportunistically over
`wifi_bandwidth_bps` only when `wifi_graph()` shows the winner and loser
are also in WiFi range of each other. `advance_merge_sync()` is called once
per tick in the control loop; `completed_merges`/`election.pending_merges`
are reported in the live `[status]` line and the final summary.

**Known, named gap**: two masters can be in Tomoto/LoRa range (enough to
agree on a merge winner via the election itself) while still outside WiFi
range (can't yet exchange the bulk `D_merged` payload) — there is no
explicit "winner decided, database sync pending" intermediate state beyond
what `MergeState`/`pending_merges` already represent; and the reachability
check for an in-progress merge checks WiFi presence but not whether the
LoRa link between winner and loser specifically (vs. just being in the same
reconnected cluster) is actually still up — a real but unresolved finding
from the most recent review pass.

### 16.10 Fault injection (CLI)

| Flag | Effect |
|---|---|
| `--crash HW_ID` | Excludes that hw_id from the run entirely (never connects, never flies) — models a drone that was never airborne, not a mid-flight failure. |
| `--byzantine HW_ID:ON:OE` | That drone self-reports its position offset by `(ON, OE)` meters — exercises the eligibility vote's spoofing detection (§16.5). |
| `--isolate HW_ID:T0:T1` | Severs every comm edge to/from that drone for sim-time window `[T0, T1)` — repeatable per hw_id (a dict-comprehension bug silently dropped all but the last window per hw_id for a repeated flag; fixed to store a list of windows). The isolated drone stays a genuine singleton participant (own election, goal-seeking-only flocking) rather than being deleted from the graph, which is what lets it diverge to its own master and produce a real merge event on reconnection. |
| `--compute-capacity HW_ID:VALUE` | Overrides a specific drone's `L_i` input for testing, since the live default (`compute_capacity_from_load()`, real `os.getloadavg()`-based) can't yet differentiate between drones running in one shared process (§16.14). |
| `--obstacle N:E:RADIUS` | Adds a known static obstacle at local NED `(N, E)` with the given radius (§16.7); repeatable. |

### 16.11 CLI reference

```
python3 realtime_swarm_dynamic.py \
  --config drones_config.json \
  --port-base 50040 \
  --comm-range-lora 30 [--comm-range-wifi 0] \
  [--lora-broadcast-interval 0] [--no-neighbor-decay] \
  [--isolation-rth-timeout 6.0] \
  --goal-n 20 --goal-e 0 \
  --takeoff-alt 10 --control-rate 20 --duration 25 \
  [--land-on-exit] [--dry-run] \
  [--crash HW_ID] [--byzantine HW_ID:ON:OE] [--isolate HW_ID:T0:T1] \
  [--compute-capacity HW_ID:VALUE] [--obstacle N:E:RADIUS] \
  --output-dir out_dir
```

`--config` reuses the same `drones_config.json` as Option B/C (§4.1) — the
`is_leader` field is simply ignored, since Option D has no fixed leader.
Omitting `--duration` runs until Ctrl-C. `--dry-run` connects and computes
everything but never arms/takes off/sends offboard setpoints, same
semantics as Option B's `--dry-run` (§7.3).

### 16.12 Validated results (real PX4 SITL + Gazebo, not just unit tests)

Every mechanism above has been exercised against live PX4, not only the
unit tests in §16.13:

- **Single-vehicle Gazebo** (`phase1_gazebo_single_vehicle.py`): full
  arm→takeoff→offboard→land lifecycle, a 6s setpoint-stream lapse (PX4
  stayed in OFFBOARD the whole time — a paused application loop with the
  connection otherwise alive behaves differently from a genuinely dead
  link), and (`phase1_offboard_loss_followup.py`) a real `mavsdk_server`
  process kill mid-flight — confirmed via the PX4 `.ulg` flight log, not
  just a live telemetry read (which gave a contradictory, misleading
  snapshot right after reconnecting) — that PX4's own offboard-loss
  failsafe lands the vehicle smoothly and disarms, with zero visibility to
  the companion side.
- **Obstacle avoidance** (`gz_worlds/obstacle_test.sdf`): a real 2m-radius,
  20m-tall cylinder in the flight path. Without `--obstacle`: straight-line
  collision, altitude went erratic/negative. With the same position given
  via `--obstacle`: clean deflection, closest approach 5.97m to the
  obstacle center, stable altitude throughout.
- **Multi-vehicle Gazebo** (`gz_worlds/multi_x500_static.sdf`, 3 drones):
  getting real multi-vehicle Gazebo working at all required a
  statically-authored world (PX4's dynamic `-i N` model-spawn mechanism has
  a real Gazebo sensor-attachment race — only 1 of 3 dynamically-spawned
  instances ever got real sensor data). Once fixed, a full 3-drone flight
  showed organic (no injected fault) periodic re-election as scores
  drifted, a brief real partition, and an automatic merge back to one
  master — live election/partition/merge dynamics in real physics, not
  simulated in the abstract 2D model.
- **Headless multi-drone SITL fault injection**: `--isolate` + merge +
  `--lora-broadcast-interval` exercised together in one real armed flight;
  a separate `--crash` test confirming re-election among survivors
  (hysteresis correctly gating the switch, not instant).

### 16.13 Tests

`tests/` — 28 assertions across 6 files, plain `python3 tests/test_X.py`
scripts (not pytest-style), runnable with no SITL/Gazebo at all since
`swarm_election.py` has no MAVSDK dependency:

| File | Covers |
|---|---|
| `test_swarm_election.py` | Core election/hysteresis, LoRa-primary/WiFi-optional comm graphs |
| `test_phase2_phase3_gaps.py` | Merge reconciliation, the ranging-sensor hook (fails loudly without real hardware) |
| `test_phase5_decay.py` | Neighbor-influence decay (ramp vs. hard-cutoff, reconnection recovery) |
| `test_phase5_tiers_rth.py` | Degradation tiers, isolation → degraded_mode → RTH |
| `test_rth_eligibility.py` | RTH'd drones excluded from election candidacy |
| `test_compute_capacity.py` | `L_i`/`compute_capacity_from_load()` actually affects the score and election outcome |

Run all of them: `tests/run_all.sh`. A later automated review (Sourcery)
found 4 of these assertions were weaker than their own stated intent (e.g.
a "force with a stale neighbor" computed but never actually checked against
anything) — all 4 fixed, and one of them (`test_phase5_tiers_rth.py`)
surfaced a genuine engine bug in the process (§16.8's RTH-clearing fix),
not just a test gap.

### 16.14 Known limitations / open items

- **Still one process** computing every drone's election and flocking —
  Option D removes the *logical* fixed-leader single point of failure, not
  the *process* one (§10.4 cross-reference above). A real deployment needs
  each drone running its own separate companion-PC process.
- **`compute_capacity_from_load()` is real but shared, not per-drone.** It
  reads this one process's actual `os.getloadavg()`, inverted/normalized —
  genuine CPU-load data, but the same number for every drone, since there's
  only one process. `--compute-capacity HW_ID:VALUE` overrides it for
  testing until each drone has its own companion PC.
- **No real ranging sensor.** `measured_distance()`'s `ranging_available`
  stays `False` on the actual target hardware (Tomoto has none, confirmed)
  — fails loudly rather than lying to the eligibility gate if flipped
  without one.
- **Energy-reserve and connectivity floors are soft, not hard** (§16.4) —
  needs a policy threshold decision, not more code.
- **Task reassignment** is a named placeholder (`merge_payload_bytes` has
  no real payload behind it yet) — blocked on a separate, not-yet-built
  task-allocation feature.
- **Tomoto's real broadcast rate and duty-cycle budget are
  uncharacterized** against actual hardware (§16.6) — the companion
  research project's abstract simulation found a broadcast-interval safety
  margin (safe through several seconds, failing sharply beyond that) that
  gives a concrete target to validate real hardware against, but that
  sweep was run in the separate `swarm_sim_core.py` project, not
  reproduced against real Tomoto radios here.
- **No live obstacle sensing** (§16.7) — `--obstacle` positions are known/
  configured, not derived from lidar/depth-camera data.
- **Goal-seeking oscillates near a static goal** (§16.7) — no
  damping/braking term, observed directly in live Gazebo flight.
- **Merge reachability isn't fully re-checked mid-sync** (§16.9) — a
  pending merge can advance assuming LoRa connectivity without confirming
  the specific winner-loser link (not just general cluster membership) is
  still up.
   window for that drone.
