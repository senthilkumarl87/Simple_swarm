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

SCOPE OF THIS PASS:
  - Real inter-UAV comm layer is still not here; comm_graph is computed centrally
    from real GPS positions against --comm-range-lora, not from an actual Tomoto
    radio/mesh link budget -- that still needs real hardware (Phase 2's
    duty-cycle/airtime characterization is physically unmeasurable without it).
    What IS now modeled: Tomoto's real broadcast RATE, via --lora-broadcast-interval
    (0 = disabled/continuous, matching every earlier run of this script; > 0 makes
    flock_force() use throttled, possibly-stale received neighbor state instead of
    live telemetry -- see swarm_election.update_neighbor_link_state()).
  - No independent ranging sensor behind the eligibility gate's d_AB -- structurally
    explicit now via swarm_election.measured_distance() (p.ranging_available stays
    False on the target hardware; flipping it without a real sensor behind it raises
    rather than silently lying to the gate).
  - Neighbor-influence decay (Sec. 6 case 3, slave<->slave) is now implemented:
    a neighbor that drops out of --comm-range-lora holds its last-known state and
    decays its flocking-force weight linearly to zero over
    --neighbor-influence-decay-window rather than vanishing instantly -- see
    swarm_election.update_neighbor_link_state(). --no-neighbor-decay reproduces
    the old hard-cutoff behavior as an ablation baseline.
  - Total isolation -> degraded mode -> RTH (Sec. 6 case 6 / Sec. 7.3) is now
    implemented: a drone with zero reachable neighbors drops cohesion/alignment
    (swarm_election.update_isolation_rth(), flock_force reads a.degraded_mode)
    and, past --isolation-rth-timeout, this script calls PX4's REAL
    action.return_to_launch() -- not a simulated flag, per the roadmap's own
    requirement. Degradation-tier classification (full/relay/partition/isolated,
    swarm_election.tier_of()) is surfaced in the periodic status line.
  - MASTER_LINK_TIMEOUT / SLAVE_ACK_TIMEOUT (Sec. 6 cases 1-2) are still not
    implemented -- remaining Phase 5 scope. Task reassignment (Sec. 7.2) stays
    blocked on the separate, not-yet-built task-allocation feature regardless
    of timeout detection.
  - Merge reconciliation on partition rejoin is now EXPLICIT: reconnecting two
    formerly-separate clusters creates a swarm_election.MergeState (winner decided
    immediately via the tie-break chain; a bulk D_merged payload then has to
    actually finish transferring -- chunked over LoRa's low bandwidth by default,
    sped up opportunistically over WiFi when in range, matching the LoRa-primary
    design rule) rather than treating the election outcome alone as "merged." See
    --merge-payload-bytes / --lora-bandwidth-bps / --wifi-bandwidth-bps.

FAULT INJECTION (for exercising the scenarios in the proposal's Section 9.3):
  --crash HW_ID              never connect/arm this drone -- simulates scenario 2
  --byzantine HW_ID:ON:OE    apply a constant self-report offset (m) to this
                              drone's position for the eligibility gate only --
                              simulates scenario 3 (spoofed position)
  --isolate HW_ID:T0:T1      sever every comm-graph edge to/from this drone during
                              [T0, T1) seconds of wall-clock run time -- it remains
                              a genuine singleton participant (runs its own election,
                              flocks on goal-seeking alone) rather than being deleted
                              from the graph, so it can actually diverge to its own
                              master and trigger a real merge event on reconnection.
                              Simulates scenario 8 (temporary isolation) / contributes
                              to scenario 6/7 (forced partition) when applied to a
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
from swarm_election import (DroneState, ElectionParams, ElectionState, comm_graph, wifi_graph, clusters,
                             flock_force, update_neighbor_link_state, advance_merge_sync,
                             tier_of, update_isolation_rth)


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
                # remaining_percent is already 0-100, not a 0-1 fraction -- confirmed via
                # Phase 1's live Gazebo test, which printed "8400%" before this fix. Without
                # the /100, np.clip(..., 0.0, 1.0) silently clamped every drone's energy to
                # 1.0 regardless of actual battery level, so the election's E_i term was
                # never actually exercised in the earlier SITL dynamic-election runs.
                state.energy = float(np.clip(b.remaining_percent / 100.0, 0.0, 1.0))
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
    params = ElectionParams(
        comm_range_lora=args.comm_range_lora, comm_range_wifi=args.comm_range_wifi,
        lora_broadcast_interval=args.lora_broadcast_interval,
        lora_bandwidth_bps=args.lora_bandwidth_bps, wifi_bandwidth_bps=args.wifi_bandwidth_bps,
        merge_payload_bytes=args.merge_payload_bytes,
        NEIGHBOR_STATE_TIMEOUT=args.neighbor_state_timeout,
        NEIGHBOR_INFLUENCE_DECAY_WINDOW=args.neighbor_influence_decay_window,
        neighbor_decay_enabled=not args.no_neighbor_decay,
        ISOLATION_RTH_TIMEOUT=args.isolation_rth_timeout,
    )
    completed_merges = 0
    rth_triggered: set[int] = set()   # hw_ids we've already fired return_to_launch() for

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

        # Comm graph: an isolated drone stays a genuine (singleton) participant --
        # it still runs its own election and flocking, just with every edge to/from
        # it severed -- rather than being deleted from the graph outright. This
        # matters for more than realism: a drone removed from the graph entirely
        # never runs _run_election() at all, so it can never diverge to its own
        # independently-elected master, which means it can never trigger a genuine
        # merge event on reconnection either -- the exact mechanism this script
        # exists to validate. Found by trying to exercise pending_merges with
        # --isolate and getting zero merges: the old behavior silently prevented
        # the scenario it was meant to produce.
        isolated_now = {hw_id for hw_id, (t0, t1) in isolate_windows.items() if t0 <= t_rel < t1}
        live_states = states
        adj = comm_graph(live_states, params.comm_range_lora)    # PRIMARY link only -- see swarm_election.py docstring
        for hw_id in isolated_now:
            severed = adj.pop(hw_id, set())
            for other in severed:
                adj.get(other, set()).discard(hw_id)
            adj[hw_id] = set()   # still its own singleton cluster, just with no edges
        update_neighbor_link_state(live_states, adj, params, t_rel)   # throttle + decay, Phase 2/5
        update_isolation_rth(live_states, adj, params, t_rel)         # degraded_mode + rth trigger, Phase 5
        wifi_adj = wifi_graph(live_states, params.comm_range_wifi)   # empty unless --comm-range-wifi > 0 and in range

        # RTH is a real, consequential MAVSDK action -- fire it exactly once per
        # drone, on the False->True transition only, not every tick it stays
        # True (which would spam return_to_launch() calls at the control rate).
        # Fire-and-forget: a failed RTL call shouldn't block the control loop,
        # but it IS reported, not silently swallowed.
        for hw_id, s in live_states.items():
            if s.rth and hw_id not in rth_triggered and hw_id in drones and not args.dry_run:
                rth_triggered.add(hw_id)
                print(f"[rth] t={t_rel:6.1f}s  hw_id {hw_id}: isolated longer than "
                      f"{params.ISOLATION_RTH_TIMEOUT}s -- calling return_to_launch()")

                async def _do_rth(hw_id=hw_id):
                    try:
                        await drones[hw_id].action.return_to_launch()
                        print(f"[rth] hw_id {hw_id}: return_to_launch() accepted")
                    except Exception as e:
                        print(f"[rth] hw_id {hw_id}: return_to_launch() FAILED: {e}")

                asyncio.create_task(_do_rth())

        for cluster_ids in clusters(adj):
            master = election.run_election(cluster_ids, live_states, adj, params, t_rel, "periodic")
            for i in cluster_ids:
                live_states[i].is_master = (i == master)
                if master is not None and last_master_print.get(i) != master:
                    print(f"[election] t={t_rel:6.1f}s  hw_id {i}: master -> {master}")
                    last_master_print[i] = master

        # Advance any merges still waiting on their bulk D_merged sync (explicit
        # "winner decided, sync pending" state -- Appendix C Phase 2 / proposal
        # Section 7.1). WiFi speeds this up opportunistically when the winner and
        # loser happen to also be in wifi_graph range of each other; otherwise it
        # completes over LoRa alone, just more slowly -- never blocked on WiFi.
        still_pending = []
        for m in election.pending_merges:
            wifi_connected = m.loser in wifi_adj.get(m.winner, set())
            advance_merge_sync(m, wifi_connected, params, period)
            if m.sync_complete:
                completed_merges += 1
                via = "WiFi" if wifi_connected else "LoRa"
                print(f"[merge] t={t_rel:6.1f}s  D_merged sync complete: winner={m.winner} "
                      f"loser={m.loser} (decided t={m.decided_at:.1f}s, synced via {via}, "
                      f"took {t_rel - m.decided_at:.1f}s)")
            else:
                still_pending.append(m)
        election.pending_merges = still_pending

        for i in follower_ids:
            if i in rth_triggered:
                # Confirmed empirically (live SITL): PX4 does NOT get pulled back
                # into OFFBOARD just because set_position_velocity_ned() keeps
                # arriving -- only offboard.start() requests that mode switch, and
                # this script never re-calls it, so RETURN_TO_LAUNCH held steady
                # for the rest of a test run with the send loop left running. Not
                # unsafe, but pointless and confusing: once a drone is told to
                # RTH, this is the one place that intent should actually show up
                # in the code, not just in a log line. Skip it entirely.
                continue
            # Isolated (but not yet RTH-triggered) drones flow through the normal
            # path: with adj[i] severed to empty above, flock_force naturally
            # reduces to goal-seeking only (no neighbors to repel/cohere/align
            # against) -- a more realistic degraded-but-still-autonomous behavior
            # than halting in place, and the one that lets it genuinely diverge
            # toward its own election outcome.
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
            cluster_list = clusters(adj)
            tiers = {i: tier_of(i, live_states, adj, cluster_list) for i in follower_ids if i in live_states}
            print(f"[status] t={t_rel:5.1f}s  master(s)={cur}  "
                  f"isolated={sorted(isolated_now) or '-'}  clusters={len(cluster_list)}  "
                  f"pending_merges={len(election.pending_merges)}  tiers={tiers}")

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
          f"{len(election.switch_log)} master-switch events, "
          f"{completed_merges} merge(s) fully synced, "
          f"{len(election.pending_merges)} merge(s) still pending at shutdown "
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
                    help="m, OPTIONAL accelerant link range; 0 = absent (default). Used only to speed up a "
                         "pending merge's D_merged sync opportunistically -- never required for correctness.")
    p.add_argument("--lora-broadcast-interval", type=float, default=0.0,
                    help="s, how often a drone actually RECEIVES a given neighbor's broadcast state over Tomoto; "
                         "0 = disabled (default), matching every earlier run of this script with continuous live "
                         "telemetry. > 0 makes flock_force use throttled, possibly-stale state instead -- see the "
                         "companion proposal's Section 10.4 for the empirical safety threshold this should be "
                         "checked against once Tomoto's real achievable rate is measured.")
    p.add_argument("--lora-bandwidth-bps", type=float, default=1000.0,
                    help="bits/s, placeholder Tomoto throughput used to pace a pending merge's D_merged transfer "
                         "when WiFi isn't available. Replace with a measured value once Tomoto's real throughput "
                         "is characterized (Appendix C, Phase 2).")
    p.add_argument("--wifi-bandwidth-bps", type=float, default=1_000_000.0,
                    help="bits/s, placeholder WiFi throughput used to pace a pending merge's D_merged transfer "
                         "when the winner and loser are also in --comm-range-wifi of each other.")
    p.add_argument("--merge-payload-bytes", type=float, default=2048.0,
                    help="placeholder D_merged (coverage/task database) size in bytes -- the real task-allocation "
                         "feature that would define this isn't built yet; this lets the merge sync TIMING "
                         "mechanism be exercised now without waiting on that unrelated feature.")
    p.add_argument("--neighbor-state-timeout", type=float, default=2.0,
                    help="s, a neighbor out of --comm-range-lora longer than this is fully excluded from flocking "
                         "(scenario 6 case 3, Section 6 of the proposal)")
    p.add_argument("--neighbor-influence-decay-window", type=float, default=1.0,
                    help="s, a neighbor's flocking-force weight ramps linearly from 1.0 to 0.0 over this window "
                         "after dropping out of range, rather than vanishing instantly")
    p.add_argument("--no-neighbor-decay", action="store_true",
                    help="ablation baseline: full weight right up to --neighbor-state-timeout, then an instant "
                         "drop, instead of the linear decay -- isolates the force-discontinuity comparison the "
                         "proposal's Section 6.3 argues decay avoids")
    p.add_argument("--isolation-rth-timeout", type=float, default=6.0,
                    help="s, zero reachable neighbors for longer than this triggers a real return_to_launch() "
                         "call (scenario 8, Section 6 case 6 / Section 7.3 of the proposal). Ignored in --dry-run.")
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
                    help="sever every comm-graph edge to/from this drone during [T0,T1) seconds -- it stays a "
                         "genuine singleton participant, not removed from the graph (scenario 8: temporary "
                         "isolation; reconnection afterward can trigger a real merge event)")
    args = p.parse_args()

    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        print("\n[main] interrupted")


if __name__ == "__main__":
    main()
