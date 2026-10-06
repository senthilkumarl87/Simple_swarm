#!/usr/bin/env python3
"""
Realtime swarm -- dynamic election + three-force flocking (replaces Option B's
fixed leader / rigid-formation model with the fault-tolerant design from the
mbc3 research workspace's uav_swarm_fault_tolerance_proposal.tex).

Reuses realtime_swarm_mavsdk.py's proven connection/arm/takeoff/shutdown/logging
plumbing unchanged; only the control loop differs: instead of one fixed leader
driving a rigid rotating formation, every drone's master eligibility, suitability
score, and local flocking force are computed centrally each tick from REAL MAVSDK
telemetry -- but using only the information a drone would actually have in a
decentralized deployment (neighbors within --comm-range-lora). This is a centralized
simulator of a decentralized algorithm, same pattern as swarm_sim_core.py itself,
just driven by real vehicle telemetry/dynamics instead of simulated physics. See
UAV_Swarm_Update_Claude_Code_Spec.md, Appendix C, Phase 3/4 for where this sits on
the hardware-implementation roadmap.

SCOPE OF THIS FIRST PASS (not yet done -- see Appendix C Phase 5):
  - No real inter-UAV comm layer; comm_graph is computed centrally from real GPS
    positions against --comm-range-lora, not from an actual radio/mesh link budget.
  - No independent ranging sensor behind the eligibility gate's d_AB -- see the
    docstring in swarm_election.py for exactly what this does and doesn't defend
    against as a result.
  - No neighbor-influence decay / link-failure-timeout state machine (Sec. 6);
    a neighbor silently drops out of the flocking force the instant it leaves
    --comm-range-lora, rather than decaying over NEIGHBOR_INFLUENCE_DECAY_WINDOW.
  - No automatic RTH on isolation timeout.
  - Merge reconciliation on partition rejoin is implicit (the tie-break chain
    naturally promotes the single highest-scoring eligible candidate across the
    merged cluster) rather than the explicit step-by-step procedure in the
    proposal's Section 7.1 algobox -- see the comment in
    swarm_election.ElectionState.run_election.

FAULT INJECTION (for exercising the scenarios in the proposal's Section 9.3):
  --crash HW_ID              never connect/arm this drone -- simulates scenario 2
  --byzantine HW_ID:ON:OE    apply a constant self-report offset (m) to this
                              drone's position for the eligibility gate only --
                              simulates scenario 3 (spoofed position)
  --isolate HW_ID:T0:T1      exclude this drone from the comm graph during
                              [T0, T1) seconds of wall-clock run time -- simulates
                              scenario 8 (temporary isolation) / contributes to
                              scenario 6/7 (forced partition) when applied to a
                              whole side of the swarm

Usage mirrors realtime_swarm_mavsdk.py; see its --help and the README for the
general SITL setup (mavsdk_server per drone, start_multi_px4.sh).
"""

import argparse
import asyncio
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

from realtime_swarm_mavsdk import (
    MAVSDK_GRPC_BASE,
    connect_drone,
    wait_gps_ok,
    force_qgc_broadcast,
    arm_and_offboard_takeoff,
    telemetry_logger_task,
    graceful_shutdown,
)
from swarm_election import DroneState, ElectionParams, ElectionState, comm_graph, clusters, flock_force


# ------------------------------------------------------------- fault injection --

def parse_byzantine(spec: str):
    parts = spec.split(":")
    hw_id = int(parts[0])
    on = float(parts[1]) if len(parts) > 1 else 10.0
    oe = float(parts[2]) if len(parts) > 2 else 10.0
    return hw_id, np.array([on, oe])


def parse_isolate(spec: str):
    hw_id_s, t0_s, t1_s = spec.split(":")
    return int(hw_id_s), float(t0_s), float(t1_s)


# --------------------------------------------------------- telemetry polling ----

async def drone_state_tracker(hw_id: int, drone: System, state: DroneState, updated: dict):
    """Keep `state` (shared, mutated in place) current from MAVSDK streams."""

    async def pos_vel():
        async for pvn in drone.telemetry.position_velocity_ned():
            state.pos[:] = (pvn.position.north_m, pvn.position.east_m)
            state.vel[:] = (pvn.velocity.north_m_s, pvn.velocity.east_m_s)
            updated[hw_id] = time.monotonic()

    async def battery():
        try:
            async for b in drone.telemetry.battery():
                state.energy = float(np.clip(b.remaining_percent, 0.0, 1.0))
        except Exception:
            pass    # SITL battery plugin not always present -- keep default 1.0

    await asyncio.gather(pos_vel(), battery())


# ------------------------------------------------------------ main control loop -

async def control_loop(args, drones: dict, states: dict, logs: dict, stop: asyncio.Event):
    follower_ids = sorted(drones.keys())
    period = 1.0 / args.control_rate
    start_mono = time.monotonic()
    tick = 0

    election = ElectionState()
    params = ElectionParams(comm_range_lora=args.comm_range_lora, comm_range_wifi=args.comm_range_wifi)

    isolate_windows = {hw_id: (t0, t1) for hw_id, t0, t1 in args.isolate}

    cmd_logs = {fid: [] for fid in follower_ids}
    last_status = 0.0
    last_master_print: dict[int, int] = {}

    print(f"[ctrl] dynamic election + flocking running at {args.control_rate} Hz "
          f"for {args.duration or '∞'}s, comm_range_lora={args.comm_range_lora}m "
          f"comm_range_wifi={args.comm_range_wifi or 'absent'}m")

    while not stop.is_set():
        now = time.monotonic()
        t_rel = now - start_mono
        if args.duration and t_rel >= args.duration:
            break

        # Comm graph: exclude drones currently inside an --isolate window.
        isolated_now = {hw_id for hw_id, (t0, t1) in isolate_windows.items() if t0 <= t_rel < t1}
        live_states = {i: s for i, s in states.items() if i not in isolated_now}
        adj = comm_graph(live_states, params.comm_range_lora)    # PRIMARY link only -- see swarm_election.py docstring

        for cluster_ids in clusters(adj):
            master = election.run_election(cluster_ids, live_states, adj, params, t_rel, "periodic")
            for i in cluster_ids:
                live_states[i].is_master = (i == master)
                if master is not None and last_master_print.get(i) != master:
                    print(f"[election] t={t_rel:6.1f}s  hw_id {i}: master -> {master}")
                    last_master_print[i] = master

        for i in follower_ids:
            if i in isolated_now:
                continue    # isolated drones hold position rather than compute flocking blind
            force = flock_force(i, live_states, adj, params)
            s = states[i]
            new_vel = s.vel + force * period
            speed = float(np.linalg.norm(new_vel))
            if speed > params.max_speed:
                new_vel = new_vel * (params.max_speed / speed)
            target_n = s.pos[0] + new_vel[0] * period
            target_e = s.pos[1] + new_vel[1] * period
            target_d = -args.takeoff_alt
            yaw_deg = float(np.degrees(np.arctan2(new_vel[1], new_vel[0]))) if speed > 0.1 else 0.0

            cmd_logs[i].append({
                "t": t_rel, "px": target_n, "py": target_e, "pz": target_d,
                "vx": new_vel[0], "vy": new_vel[1], "vz": 0.0, "yaw_deg": yaw_deg,
            })
            if not args.dry_run:
                try:
                    await drones[i].offboard.set_position_velocity_ned(
                        PositionNedYaw(target_n, target_e, target_d, yaw_deg),
                        VelocityNedYaw(new_vel[0], new_vel[1], 0.0, 0.0),
                    )
                except OffboardError as e:
                    print(f"[ctrl] hw_id {i} offboard error: {e}")

        if now - last_status >= 2.0 and logs:
            last_status = now
            masters = {i: live_states[i].is_master for i in follower_ids if i in live_states}
            cur = [i for i, m in masters.items() if m]
            print(f"[status] t={t_rel:5.1f}s  master(s)={cur}  "
                  f"isolated={sorted(isolated_now) or '-'}  clusters={len(clusters(adj))}")

        tick += 1
        sleep_for = (start_mono + tick * period) - time.monotonic()
        await asyncio.sleep(sleep_for if sleep_for > 0 else 0)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    for fid, rows in cmd_logs.items():
        pd.DataFrame(rows).to_csv(args.output_dir / f"drone_{fid}_commanded.csv", index=False)
    with (args.output_dir / "master_switch_events.csv").open("w") as f:
        f.write("t,cluster,old_master,new_master,reason\n")
        for t, cids, old, new, why in election.switch_log:
            f.write(f"{t:.2f},\"{cids}\",{old},{new},{why}\n")
    print(f"[ctrl] control loop done -- {tick} ticks, "
          f"{len(election.switch_log)} master-switch events "
          f"(log: {args.output_dir / 'master_switch_events.csv'})")


# -------------------------------------------------------------------- entrypoint

async def run(args):
    import json
    cfg = json.loads(args.config.read_text())
    all_ids = sorted(int(d["hw_id"]) for d in cfg["drones"])
    crashed = set(args.crash)
    run_ids = [i for i in all_ids if i not in crashed]
    if crashed:
        print(f"[main] --crash: excluding hw_id(s) {sorted(crashed)} from this run entirely")
    print(f"[main] dynamic-election swarm: {run_ids}")

    drones = {}
    for hw_id in run_ids:
        port = args.port_base + hw_id
        drones[hw_id] = await connect_drone(hw_id, port)

    await asyncio.gather(*(force_qgc_broadcast(hw_id, d) for hw_id, d in drones.items()))

    if not args.dry_run:
        await asyncio.gather(*(wait_gps_ok(hw_id, d) for hw_id, d in drones.items()))
        await asyncio.gather(*(arm_and_offboard_takeoff(hw_id, d, args.takeoff_alt)
                               for hw_id, d in drones.items()))

    # Seed state from each drone's current position before the control loop starts,
    # so goals (start + offset) are anchored correctly and the first tick isn't blind.
    states: dict[int, DroneState] = {}
    for hw_id, d in drones.items():
        pvn = None
        async for p in d.telemetry.position_velocity_ned():
            pvn = p; break
        n0, e0 = (pvn.position.north_m, pvn.position.east_m) if pvn else (0.0, 0.0)
        states[hw_id] = DroneState(
            hw_id=hw_id,
            pos=np.array([n0, e0]),
            vel=np.zeros(2),
            goal=np.array([n0 + args.goal_n, e0 + args.goal_e]),
        )
    for hw_id, offset in args.byzantine:
        if hw_id in states:
            states[hw_id].byzantine = True
            states[hw_id].spoof_offset = offset
            print(f"[main] --byzantine: hw_id {hw_id} self-reports offset by {offset} m")

    updated = {}
    tracker_tasks = [
        asyncio.create_task(drone_state_tracker(hw_id, d, states[hw_id], updated))
        for hw_id, d in drones.items()
    ]

    stop = asyncio.Event()
    tel_logs = {hw_id: [] for hw_id in drones}
    tel_tasks = [
        asyncio.create_task(telemetry_logger_task(hw_id, d, tel_logs[hw_id], stop))
        for hw_id, d in drones.items()
    ]

    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, lambda: stop.set())

    try:
        await control_loop(args, drones, states, tel_logs, stop)
    finally:
        stop.set()
        for t in tracker_tasks + tel_tasks:
            t.cancel()
        await graceful_shutdown(args, drones, leader_id=None)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    for hw_id, rows in tel_logs.items():
        if not rows:
            continue
        df = pd.DataFrame(rows)
        df["t"] = df["t"] - rows[0]["t"]
        df["yaw"] = np.radians(df["yaw_deg"])
        df.drop(columns=["yaw_deg"], inplace=True)
        df.insert(0, "idx", np.arange(len(df), dtype=int))
        for col in ("ax", "ay", "az"):
            df[col] = 0.0
        df["mode"] = 70
        df["ledr"], df["ledg"], df["ledb"] = 255, 255, 255
        df = df[["idx", "t", "px", "py", "pz", "vx", "vy", "vz",
                 "ax", "ay", "az", "yaw", "mode", "ledr", "ledg", "ledb"]]
        df.to_csv(args.output_dir / f"drone_{hw_id}.csv", index=False)
    print(f"[main] logs in {args.output_dir}/")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", type=Path, required=True, help="drones_config.json (is_leader field ignored)")
    p.add_argument("--comm-range-lora", type=float, default=30.0,
                    help="m, PRIMARY (LoRa) link range -- election/eligibility/flocking all key to this")
    p.add_argument("--comm-range-wifi", type=float, default=0.0,
                    help="m, OPTIONAL accelerant link range; 0 = absent (default). Not yet wired into any "
                         "function -- reserved for a future opportunistic bulk-transfer speedup.")
    p.add_argument("--goal-n", type=float, default=20.0, help="m, shared goal offset (north) from each drone's own start position")
    p.add_argument("--goal-e", type=float, default=0.0, help="m, shared goal offset (east) from each drone's own start position")
    p.add_argument("--takeoff-alt", type=float, default=10.0)
    p.add_argument("--control-rate", type=float, default=20.0)
    p.add_argument("--duration", type=float, default=None)
    p.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parent / "output" / "dynamic_election")
    p.add_argument("--port-base", type=int, default=MAVSDK_GRPC_BASE)
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--land-on-exit", action="store_true")
    p.add_argument("--crash", type=int, action="append", default=[], metavar="HW_ID",
                    help="exclude this hw_id entirely (scenario 2: single master/slave crash)")
    p.add_argument("--byzantine", type=parse_byzantine, action="append", default=[], metavar="HW_ID:ON:OE",
                    help="self-report offset in meters, eligibility gate only (scenario 3: spoofed position)")
    p.add_argument("--isolate", type=parse_isolate, action="append", default=[], metavar="HW_ID:T0:T1",
                    help="exclude from comm graph during [T0,T1) seconds (scenario 8: temporary isolation)")
    args = p.parse_args()

    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        print("\n[main] interrupted")


if __name__ == "__main__":
    main()
