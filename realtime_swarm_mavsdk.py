#!/usr/bin/env python3
"""
Realtime swarm — centralized MAVSDK controller (Option B).

One process owns a MAVSDK connection to every drone in drones_config.json.
It picks a leader state source (either MAVSDK telemetry of the leader drone,
or a CSV replay for benching without a leader vehicle), then at --control-rate
Hz computes each follower's target NED position+velocity+yaw using the same
rotation math as Option C (including heading-rate feed-forward) and pushes
offboard setpoints to each follower.

PREREQUISITES
  For every drone listed in drones_config.json, a mavsdk_server instance must
  be running and bound to a unique gRPC port. Default port = 50040 + hw_id
  (matches the convention in src/drone.py).

  Example SITL setup (from the existing repo tooling):
      cd multiple_sitl && ./multiple_sitl.sh -n 5 -m iris     # spawns 5 PX4s
      # then per drone:
      ./mavsdk_server -p 50041 udp://:14541
      ./mavsdk_server -p 50042 udp://:14542
      ...

  For a single-drone real test, run mavsdk_server against your vehicle's
  MAVLink connection (serial or UDP) on port 50041 (hw_id=1).

LIFECYCLE
  1. Connect to every drone's mavsdk_server.
  2. (unless --dry-run) Wait for GPS lock, arm + takeoff drones that are on
     the ground, then start offboard on each follower.
  3. Main loop at --control-rate Hz:
       - if --leader-source mavsdk: read latest telemetry of the leader drone
       - if --leader-source csv:    interpolate leader_path.csv at wall-time
       - for each follower: compute target, push set_position_velocity_ned
       - log achieved telemetry per drone
  4. On Ctrl+C / --duration expiry: stop offboard, optionally land, disarm.

DRY-RUN
  --dry-run connects to mavsdk_servers (so they must be running), reads
  telemetry, and computes setpoints, but does NOT arm/takeoff/offboard.
  It still writes logs, so you can validate the math and wiring without
  putting vehicles in motion.

OUTPUT
  --output-dir contains per-drone CSVs in the same schema as Option C,
  so visualize_trajectories.py / animate_trajectories.py work on them.
"""

import argparse
import asyncio
import json
import math
import signal
import sys
import time
from pathlib import Path

import numpy as np
import pandas as pd

try:
    from mavsdk import System
    from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw
except ImportError:
    print("Error: mavsdk not installed. Install with: pip install mavsdk", file=sys.stderr)
    sys.exit(1)

sys.path.insert(0, str(Path(__file__).resolve().parent))

MAVSDK_GRPC_BASE = 50040   # follower N → port 50040+N, matches src/drone.py


# ---------------------------------------------------------------- helpers ----

def load_config(config_path: Path):
    with config_path.open() as f:
        cfg = json.load(f)
    leaders = [d["hw_id"] for d in cfg["drones"] if d.get("is_leader")]
    if len(leaders) != 1:
        raise ValueError(f"drones_config.json must have exactly one leader; found {len(leaders)}")
    leader_id = int(leaders[0])
    all_ids = [int(d["hw_id"]) for d in cfg["drones"]]
    return leader_id, all_ids


def load_formation(formation_path: Path, leader_id: int):
    df = pd.read_csv(formation_path)
    followers = df[df["follow"] == leader_id]
    out = {}
    for _, row in followers.iterrows():
        out[int(row["hw_id"])] = (
            float(row["offset_n"]), float(row["offset_e"]), float(row["offset_alt"])
        )
    return out


def load_leader_path(path: Path) -> pd.DataFrame:
    """Same preprocessing as realtime_leader.py."""
    df = pd.read_csv(path).sort_values("t").reset_index(drop=True)
    t = df["t"].to_numpy()
    if "heading" not in df.columns:
        dn = np.gradient(df["north"].to_numpy(), t)
        de = np.gradient(df["east"].to_numpy(),  t)
        df["heading"] = np.degrees(np.arctan2(de, dn))
    df["heading_unwrapped"] = np.degrees(np.unwrap(np.radians(df["heading"].to_numpy())))
    df["vn"] =  np.gradient(df["north"].to_numpy(),     t)
    df["ve"] =  np.gradient(df["east"].to_numpy(),      t)
    df["vd"] = -np.gradient(df["altitude"].to_numpy(),  t)
    df["heading_rate_dps"] = np.gradient(df["heading_unwrapped"].to_numpy(), t)
    return df


def rotate_offset(on, oe, heading_deg):
    th = math.radians(heading_deg)
    c, s = math.cos(th), math.sin(th)
    return on * c - oe * s, on * s + oe * c


# ------------------------------------------------------------ shared state ----

class LeaderState:
    """Latest-known leader state, updated by either a MAVSDK subscription
    or a CSV replay task. Written by one producer, read by the control loop."""
    def __init__(self):
        self.n = self.e = self.d = None     # NED position (m)
        self.vn = self.ve = self.vd = 0.0   # NED velocity (m/s)
        self.heading_deg = 0.0
        self.heading_rate_dps = 0.0
        self._last_heading_sample = None    # for finite-diff rate when subscribing
        self._last_heading_time = None
        self.updated_at = None

    def update_position(self, n, e, d, vn, ve, vd):
        self.n, self.e, self.d = n, e, d
        self.vn, self.ve, self.vd = vn, ve, vd
        self.updated_at = time.monotonic()

    def update_heading(self, heading_deg):
        now = time.monotonic()
        if self._last_heading_sample is not None:
            # shortest-angle delta so a 359→1 sample gives +2° rate, not -358°
            prev = self._last_heading_sample
            delta = ((heading_deg - prev + 180.0) % 360.0) - 180.0
            dt = now - self._last_heading_time
            if dt > 1e-3:
                self.heading_rate_dps = delta / dt
        self.heading_deg = heading_deg
        self._last_heading_sample = heading_deg
        self._last_heading_time = now


# --------------------------------------------------- leader source: MAVSDK ----

async def leader_mavsdk_position_task(sys_drone: System, state: LeaderState):
    async for pvn in sys_drone.telemetry.position_velocity_ned():
        state.update_position(
            pvn.position.north_m, pvn.position.east_m, pvn.position.down_m,
            pvn.velocity.north_m_s, pvn.velocity.east_m_s, pvn.velocity.down_m_s,
        )


async def leader_mavsdk_attitude_task(sys_drone: System, state: LeaderState):
    async for att in sys_drone.telemetry.attitude_euler():
        state.update_heading(att.yaw_deg)


# ---------------------------------------------------- leader source: CSV -----

async def leader_csv_replay_task(path_df: pd.DataFrame, state: LeaderState):
    """Replay leader_path.csv in wall-clock time into `state`."""
    t_arr = path_df["t"].to_numpy()
    t_end = float(t_arr[-1])
    t_start = time.monotonic()

    def interp(col): return lambda now: float(np.interp(min(now, t_end), t_arr, path_df[col]))
    getters = {
        "n":                interp("north"),
        "e":                interp("east"),
        "alt":              interp("altitude"),
        "vn":               interp("vn"),
        "ve":               interp("ve"),
        "vd":               interp("vd"),
        "heading_deg":      interp("heading_unwrapped"),
        "heading_rate_dps": interp("heading_rate_dps"),
    }
    while True:
        now = time.monotonic() - t_start
        state.update_position(
            getters["n"](now), getters["e"](now), -getters["alt"](now),
            getters["vn"](now), getters["ve"](now), getters["vd"](now),
        )
        # CSV source sets heading + rate directly rather than finite-differencing
        state.heading_deg      = getters["heading_deg"](now)
        state.heading_rate_dps = getters["heading_rate_dps"](now)
        if now >= t_end:
            return
        await asyncio.sleep(0.02)    # 50 Hz update of shared state


# --------------------------------------------------------- per-drone setup ---

async def connect_drone(hw_id: int, port: int, timeout: float = 10.0) -> System:
    print(f"[hw_id {hw_id}] connecting on mavsdk_server :{port} ...", flush=True)
    sys_drone = System(mavsdk_server_address="127.0.0.1", port=port)

    async def _connect_and_wait():
        await sys_drone.connect()
        async for state in sys_drone.core.connection_state():
            if state.is_connected:
                return

    try:
        await asyncio.wait_for(_connect_and_wait(), timeout=timeout)
    except asyncio.TimeoutError:
        raise TimeoutError(
            f"hw_id {hw_id}: no response from mavsdk_server on 127.0.0.1:{port} "
            f"within {timeout}s. Is the server running and connected to a vehicle?"
        )
    print(f"[hw_id {hw_id}] mavsdk connected")
    return sys_drone


async def wait_gps_ok(hw_id: int, drone: System, timeout: float = 30.0):
    t0 = time.monotonic()
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print(f"[hw_id {hw_id}] GPS ok")
            return
        if time.monotonic() - t0 > timeout:
            raise TimeoutError(f"hw_id {hw_id} GPS not ok within {timeout}s")


async def force_qgc_broadcast(hw_id: int, drone: System):
    """Ensure PX4 broadcasts MAVLink heartbeats so QGC can auto-discover this
    vehicle even after a QGC restart. Without this PX4 only replies to whoever
    pinged it first and silently stays dark to the rest of the network."""
    try:
        await drone.param.set_param_int("MAV_0_BROADCAST", 1)
        print(f"[hw_id {hw_id}] MAV_0_BROADCAST=1 set (QGC will auto-discover)")
    except Exception as e:
        # not fatal — QGC may still see the vehicle if it was already connected
        print(f"[hw_id {hw_id}] could not set MAV_0_BROADCAST: {e}")


async def arm_and_offboard_takeoff(hw_id: int, drone: System, alt: float):
    """Take off into offboard mode directly — no PX4 auto-takeoff mission item.

    Rationale: PX4's `action.takeoff()` creates an internal takeoff waypoint,
    which lingers after we switch to offboard and triggers a "waypoints out
    of sequence" warning in QGroundControl (especially with multi-vehicle
    sync). Seeding offboard at (current_n, current_e, -alt) lets PX4's
    position controller climb to altitude without creating any mission
    items. Matches how mavlink_swarm_controller.py in this repo does it.
    """
    # Clear any residual mission left over from previous runs / auto-takeoff.
    try:
        await drone.mission.clear_mission()
    except Exception:
        pass    # not supported on all vehicles; harmless to skip

    # Skip if already airborne (e.g. re-running the controller mid-flight).
    async for v in drone.telemetry.in_air():
        if v:
            print(f"[hw_id {hw_id}] already in air")
            return
        break

    # Snapshot current ground position
    init = None
    async for pvn in drone.telemetry.position_velocity_ned():
        init = pvn; break
    n0, e0 = init.position.north_m, init.position.east_m

    # Seed the offboard setpoint at takeoff altitude (NED: down is negative-up)
    await drone.offboard.set_position_ned(
        PositionNedYaw(n0, e0, -alt, 0.0))

    # Arm + engage offboard
    await drone.action.arm()
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"[hw_id {hw_id}] offboard start FAILED: {e}")
        raise

    # Wait until airborne (up to 10s — if never true, log it and continue)
    in_air = False
    t_air = time.monotonic()
    async for v in drone.telemetry.in_air():
        if v:
            in_air = True; break
        if time.monotonic() - t_air > 10.0:
            break

    # Settle loop: up to 15s, polling current altitude
    last_alt = None
    for _ in range(75):   # up to ~15 s
        pvn = None
        async for p in drone.telemetry.position_velocity_ned():
            pvn = p; break
        if pvn is None:
            break
        last_alt = -pvn.position.down_m
        if abs(last_alt - alt) < 1.0:
            break
        await asyncio.sleep(0.2)

    # Confirm we're actually armed + in offboard — if either is false,
    # the drone will just sit on the ground while the control loop pushes
    # setpoints into the void. Report explicitly.
    armed = False
    async for a in drone.telemetry.armed():
        armed = a; break
    flight_mode = "?"
    async for fm in drone.telemetry.flight_mode():
        flight_mode = str(fm); break
    alt_str = f"{last_alt:.2f}m" if last_alt is not None else "?"
    print(f"[hw_id {hw_id}] offboard takeoff done: "
          f"in_air={in_air}  armed={armed}  mode={flight_mode}  alt={alt_str}")


# ---------------------------------------------------- logging scaffolding ----

async def telemetry_logger_task(hw_id: int, drone: System, log: list, stop: asyncio.Event):
    """Keep pulling position_velocity_ned + yaw and append rows until stop."""
    # attitude subscription in a separate task so we can zip with position
    latest_yaw = [0.0]

    async def yaw_sub():
        async for att in drone.telemetry.attitude_euler():
            latest_yaw[0] = att.yaw_deg
            if stop.is_set():
                return

    yaw_task = asyncio.create_task(yaw_sub())
    try:
        async for pvn in drone.telemetry.position_velocity_ned():
            if stop.is_set():
                break
            log.append({
                "t": time.monotonic(),
                "px": pvn.position.north_m,
                "py": pvn.position.east_m,
                "pz": pvn.position.down_m,
                "vx": pvn.velocity.north_m_s,
                "vy": pvn.velocity.east_m_s,
                "vz": pvn.velocity.down_m_s,
                "yaw_deg": latest_yaw[0],
            })
    finally:
        yaw_task.cancel()


# ------------------------------------------------------ main control loop ----

async def control_loop(args, drones: dict, formation: dict, leader_state: LeaderState,
                       logs: dict, stop: asyncio.Event, leader_id: int = None,
                       leader_path_df: pd.DataFrame = None):
    # Wait until leader state is populated
    t0 = time.monotonic()
    while leader_state.n is None:
        if time.monotonic() - t0 > args.first_state_timeout:
            print(f"[ctrl] no leader state within {args.first_state_timeout}s; aborting")
            stop.set(); return
        await asyncio.sleep(0.05)

    follower_ids = sorted(formation.keys())
    period = 1.0 / args.control_rate
    start_mono = time.monotonic()
    tick = 0

    # Pre-commanded-setpoint log (what we sent)
    cmd_logs = {fid: [] for fid in follower_ids}
    if leader_id is not None and leader_id in drones:
        cmd_logs[leader_id] = []    # also log leader commanded setpoints

    # When a CSV leader path is provided AND the leader drone is connected,
    # drive the leader along the CSV path too so it physically flies (else
    # it sits idle on the ground — the reported "vehicle 1 didn't move" bug).
    drive_leader = (
        leader_path_df is not None
        and leader_id is not None
        and leader_id in drones
    )
    if drive_leader:
        print(f"[ctrl] will also push CSV setpoints to the leader drone (hw_id {leader_id})")

    print(f"[ctrl] running at {args.control_rate} Hz for {args.duration or '∞'}s")
    last_status = 0.0

    while not stop.is_set():
        now = time.monotonic()
        if args.duration and (now - start_mono) >= args.duration:
            break

        # periodic live telemetry — every 2s so the user can see real motion
        # in the log even when QGC isn't visible. Prints leader + each follower
        # measured NED alt + reads from the telemetry logs (the most recent row).
        if now - last_status >= 2.0 and logs:
            last_status = now
            t_rel = now - start_mono
            parts = [f"t={t_rel:5.1f}s"]
            if leader_id in logs and logs[leader_id]:
                r = logs[leader_id][-1]
                parts.append(f"L(hw{leader_id}): N={r['px']:+.1f} E={r['py']:+.1f} "
                             f"alt={-r['pz']:+.1f} yaw={r['yaw_deg']:+.0f}°")
            for fid in follower_ids:
                if fid in logs and logs[fid]:
                    r = logs[fid][-1]
                    parts.append(f"F{fid}: alt={-r['pz']:+.1f}")
            print("[status] " + "  ".join(parts))

        # Snapshot leader state
        ln, le, ld = leader_state.n, leader_state.e, leader_state.d
        lvn, lve, lvd = leader_state.vn, leader_state.ve, leader_state.vd
        lhdg = leader_state.heading_deg
        lhdg_rate = leader_state.heading_rate_dps
        omega = math.radians(lhdg_rate)

        # Compute + push per follower
        for fid in follower_ids:
            on, oe, oalt = formation[fid]
            rot_n, rot_e = rotate_offset(on, oe, lhdg)
            target_n = ln + rot_n
            target_e = le + rot_e
            target_d = ld - oalt                          # NED: altitude offset subtracted
            target_vn = lvn - omega * rot_e               # rotational feed-forward
            target_ve = lve + omega * rot_n
            target_vd = lvd
            target_yaw_deg = lhdg

            cmd_logs[fid].append({
                "t": now - start_mono,
                "px": target_n, "py": target_e, "pz": target_d,
                "vx": target_vn, "vy": target_ve, "vz": target_vd,
                "yaw_deg": target_yaw_deg,
            })

            if not args.dry_run:
                try:
                    await drones[fid].offboard.set_position_velocity_ned(
                        PositionNedYaw(target_n, target_e, target_d, target_yaw_deg),
                        VelocityNedYaw(target_vn, target_ve, target_vd, 0.0),
                    )
                except OffboardError as e:
                    print(f"[ctrl] hw_id {fid} offboard error: {e}")

        # Drive the leader drone with the CSV path (interpolated at wall-clock
        # time). The leader_state used above for follower targets comes from
        # the leader's REAL MAVSDK telemetry; here we use the planned CSV.
        if drive_leader:
            tnow = now - start_mono
            t_arr = leader_path_df["t"].to_numpy()
            clamp = float(np.clip(tnow, t_arr[0], t_arr[-1]))
            cmd_ln   = float(np.interp(clamp, t_arr, leader_path_df["north"]))
            cmd_le   = float(np.interp(clamp, t_arr, leader_path_df["east"]))
            cmd_ld   = -float(np.interp(clamp, t_arr, leader_path_df["altitude"]))
            cmd_lvn  = float(np.interp(clamp, t_arr, leader_path_df["vn"]))
            cmd_lve  = float(np.interp(clamp, t_arr, leader_path_df["ve"]))
            cmd_lvd  = float(np.interp(clamp, t_arr, leader_path_df["vd"]))
            cmd_lhdg = float(np.interp(clamp, t_arr, leader_path_df["heading_unwrapped"]))
            cmd_logs[leader_id].append({
                "t": tnow,
                "px": cmd_ln, "py": cmd_le, "pz": cmd_ld,
                "vx": cmd_lvn, "vy": cmd_lve, "vz": cmd_lvd,
                "yaw_deg": cmd_lhdg,
            })
            if not args.dry_run:
                try:
                    await drones[leader_id].offboard.set_position_velocity_ned(
                        PositionNedYaw(cmd_ln, cmd_le, cmd_ld, cmd_lhdg),
                        VelocityNedYaw(cmd_lvn, cmd_lve, cmd_lvd, 0.0),
                    )
                except OffboardError as e:
                    print(f"[ctrl] leader hw_id {leader_id} offboard error: {e}")

        tick += 1
        sleep_for = (start_mono + tick * period) - time.monotonic()
        if sleep_for > 0:
            await asyncio.sleep(sleep_for)
        else:
            # loop fell behind; yield at least once so other tasks can run
            await asyncio.sleep(0)

    # dump commanded setpoints
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for fid, rows in cmd_logs.items():
        pd.DataFrame(rows).to_csv(args.output_dir / f"drone_{fid}_commanded.csv", index=False)
    print(f"[ctrl] control loop done — {tick} ticks")


async def graceful_shutdown(args, drones: dict, leader_id: int):
    if args.dry_run:
        return
    # Stop offboard on every follower
    for hw_id, drone in drones.items():
        if hw_id == leader_id:
            continue
        try:
            await drone.offboard.stop()
        except OffboardError as e:
            print(f"[shutdown] hw_id {hw_id} stop offboard: {e}")

    if args.land_on_exit:
        for hw_id, drone in drones.items():
            try:
                await drone.action.land()
            except Exception as e:
                print(f"[shutdown] hw_id {hw_id} land: {e}")


# -------------------------------------------------------------- entrypoint ---

async def run(args):
    leader_id, all_ids = load_config(args.config)
    formation = load_formation(args.formation, leader_id)
    print(f"[main] leader={leader_id}  followers={sorted(formation.keys())}")

    # Connect every drone. The leader drone is always included:
    #   - leader-source=mavsdk: we SUBSCRIBE to its telemetry
    #   - leader-source=csv:    we DRIVE it with the CSV path so it flies too
    #     (otherwise QGC just shows vehicle 1 sitting idle on the ground —
    #     confusing, and you lose visual confirmation the leader path is real)
    targets = set(formation.keys())
    targets.add(leader_id)
    drones = {}
    for hw_id in sorted(targets):
        port = args.port_base + hw_id
        drones[hw_id] = await connect_drone(hw_id, port)

    # Push MAV_0_BROADCAST=1 to every drone whether or not we're going to arm —
    # it's the right thing for QGC discovery either way (and cheap to do).
    await asyncio.gather(*(force_qgc_broadcast(hw_id, d) for hw_id, d in drones.items()))

    if not args.dry_run:
        await asyncio.gather(*(wait_gps_ok(hw_id, d) for hw_id, d in drones.items()))
        # Arm + takeoff into offboard in one step, concurrently on every drone.
        # No PX4 auto-takeoff waypoint is created, so QGC doesn't see a
        # dangling takeoff mission item and doesn't complain about sequence.
        await asyncio.gather(*(arm_and_offboard_takeoff(hw_id, d, args.takeoff_alt)
                               for hw_id, d in drones.items()))

    # Leader state source for FOLLOWER TARGET COMPUTATION:
    # Always subscribe to the real leader drone's MAVSDK telemetry, so
    # followers formation-lock onto where the leader actually is (not
    # where the CSV path says it should be). This way the spear tracks
    # the leader's real flight, including any tracking lag.
    leader_state = LeaderState()
    leader_tasks = []
    leader_tasks.append(asyncio.create_task(
        leader_mavsdk_position_task(drones[leader_id], leader_state)))
    leader_tasks.append(asyncio.create_task(
        leader_mavsdk_attitude_task(drones[leader_id], leader_state)))

    # Separately, if --leader-source csv, also drive the leader drone with
    # CSV setpoints so it physically flies the planned path. control_loop
    # reads the CSV path at current wall-clock time on each tick.
    leader_path_df = None
    if args.leader_source == "csv" and args.leader_path is not None:
        leader_path_df = load_leader_path(args.leader_path)

    # Telemetry loggers per drone (so we capture achieved)
    stop = asyncio.Event()
    tel_logs = {hw_id: [] for hw_id in drones}
    tel_tasks = [
        asyncio.create_task(telemetry_logger_task(hw_id, d, tel_logs[hw_id], stop))
        for hw_id, d in drones.items()
    ]

    # Signal handling for Ctrl+C
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, lambda: stop.set())

    try:
        await control_loop(args, drones, formation, leader_state, tel_logs, stop,
                           leader_id=leader_id, leader_path_df=leader_path_df)
    finally:
        stop.set()
        for t in leader_tasks + tel_tasks:
            t.cancel()
        await graceful_shutdown(args, drones, leader_id)

    # Also dump the leader's commanded trajectory (from the CSV replay) as
    # drone_<leader>_leader.csv so the visualizer gets a full formation view.
    if args.leader_source == "csv" and args.leader_path is not None:
        try:
            lp = load_leader_path(args.leader_path)
            lp_df = pd.DataFrame({
                "idx":  np.arange(len(lp), dtype=int),
                "t":    lp["t"].to_numpy(),
                "px":   lp["north"].to_numpy(),
                "py":   lp["east"].to_numpy(),
                "pz":  -lp["altitude"].to_numpy(),
                "vx":   lp["vn"].to_numpy(),
                "vy":   lp["ve"].to_numpy(),
                "vz":   lp["vd"].to_numpy(),
                "ax":   np.zeros(len(lp)),
                "ay":   np.zeros(len(lp)),
                "az":   np.zeros(len(lp)),
                "yaw":  np.radians(lp["heading_unwrapped"].to_numpy()),
                "mode": np.full(len(lp), 70, dtype=int),
                "ledr": np.full(len(lp), 255, dtype=int),
                "ledg": np.full(len(lp), 64,  dtype=int),
                "ledb": np.full(len(lp), 64,  dtype=int),
            })
            args.output_dir.mkdir(parents=True, exist_ok=True)
            lp_df.to_csv(args.output_dir / f"drone_{leader_id}_leader.csv", index=False)
        except Exception as e:
            print(f"[main] could not export leader path: {e}")

    # Dump achieved telemetry logs in the common schema so visualize_trajectories.py
    # and animate_trajectories.py work on Option B output unchanged.
    args.output_dir.mkdir(parents=True, exist_ok=True)
    t_zero = min((rows[0]["t"] for rows in tel_logs.values() if rows), default=time.monotonic())
    for hw_id, rows in tel_logs.items():
        if not rows:
            continue
        df = pd.DataFrame(rows)
        df["t"] = df["t"] - t_zero
        df["yaw"] = np.radians(df["yaw_deg"])                # common schema uses radians
        df.drop(columns=["yaw_deg"], inplace=True)
        # fields we don't measure via MAVSDK — fill zero-valued columns to match schema
        df.insert(0, "idx", np.arange(len(df), dtype=int))
        for col in ("ax", "ay", "az"):
            df[col] = 0.0
        df["mode"] = 70
        is_leader = (hw_id == leader_id)
        df["ledr"] = 255 if is_leader else 255
        df["ledg"] = 64  if is_leader else 255
        df["ledb"] = 64  if is_leader else 255
        df = df[["idx", "t", "px", "py", "pz", "vx", "vy", "vz",
                 "ax", "ay", "az", "yaw", "mode", "ledr", "ledg", "ledb"]]
        name = f"drone_{hw_id}_leader.csv" if is_leader else f"drone_{hw_id}.csv"
        df.to_csv(args.output_dir / name, index=False)
    print(f"[main] logs in {args.output_dir}/")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config",      type=Path, required=True)
    p.add_argument("--formation",   type=Path, required=True)
    p.add_argument("--leader-source", choices=("mavsdk", "csv"), default="mavsdk",
                   help="Where the leader state comes from: real MAVSDK telemetry of the leader "
                        "drone (default), or a CSV replay for bench testing without a leader vehicle.")
    p.add_argument("--leader-path", type=Path, default=None,
                   help="Required when --leader-source=csv.")
    p.add_argument("--takeoff-alt", type=float, default=10.0,
                   help="Arm + takeoff altitude (m) for drones on the ground. 0 = skip.")
    p.add_argument("--control-rate", type=float, default=20.0, help="Setpoint rate (Hz)")
    p.add_argument("--duration",    type=float, default=None,
                   help="How long to run the control loop (s). Default: until Ctrl+C.")
    p.add_argument("--output-dir",  type=Path,
                   default=Path(__file__).resolve().parent / "output" / "mavsdk_realtime")
    p.add_argument("--port-base",   type=int, default=MAVSDK_GRPC_BASE,
                   help=f"gRPC port base; follower N uses port (base + N). Default {MAVSDK_GRPC_BASE}.")
    p.add_argument("--dry-run", action="store_true",
                   help="Connect + read telemetry + compute setpoints, but do NOT arm/takeoff/offboard.")
    p.add_argument("--land-on-exit", action="store_true",
                   help="After the control loop ends, issue land to every drone.")
    p.add_argument("--first-state-timeout", type=float, default=30.0,
                   help="Abort if no leader state arrives within this window.")
    args = p.parse_args()

    if args.leader_source == "csv" and args.leader_path is None:
        p.error("--leader-source=csv requires --leader-path")

    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        print("\n[main] interrupted")


if __name__ == "__main__":
    main()
