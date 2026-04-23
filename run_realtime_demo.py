#!/usr/bin/env python3
"""
Orchestrator for the loopback real-time swarm demo.

Spawns one realtime_leader.py subprocess plus one realtime_follower.py
per follower in drones_config.json, lets them run to completion, and
(optionally) renders the collected logs with visualize_trajectories.py
and animate_trajectories.py.

Every process is a separate OS process talking over localhost UDP, so
the flow is identical to what a distributed deployment would do — the
only difference from Option B (MAVSDK real flight) is that the follower
runs the same kinematic PD model as simulate_swarm.py instead of
sending offboard commands to a drone.
"""

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import pandas as pd

HERE = Path(__file__).resolve().parent


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config",      type=Path, required=True)
    p.add_argument("--formation",   type=Path, required=True)
    p.add_argument("--leader-path", type=Path, required=True)
    p.add_argument("--output-dir",  type=Path, default=HERE / "output" / "realtime")
    p.add_argument("--leader-rate", type=float, default=10.0, help="Leader broadcast rate (Hz)")
    p.add_argument("--sim-rate",    type=float, default=50.0, help="Follower inner-loop rate (Hz)")
    p.add_argument("--visualize",   action="store_true", help="Run visualize + animate after the demo")
    p.add_argument("--smooth",      action="store_true",
                   help="Enable α-β smoothing on followers (see realtime_follower.py).")
    p.add_argument("--smooth-alpha", type=float, default=0.5)
    p.add_argument("--smooth-beta",  type=float, default=0.1)
    args = p.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)

    # Resolve leader + followers
    with args.config.open() as f:
        cfg = json.load(f)
    leader_id = int(next(d["hw_id"] for d in cfg["drones"] if d.get("is_leader")))
    form = pd.read_csv(args.formation)
    formation_followers = set(form[form["follow"] == leader_id]["hw_id"].astype(int))
    follower_ids = [int(d["hw_id"]) for d in cfg["drones"]
                    if not d.get("is_leader") and int(d["hw_id"]) in formation_followers]

    print(f"[demo] leader={leader_id}  followers={follower_ids}  output={args.output_dir}")

    # --- Start followers first so their sockets are bound before the leader sends ---
    follower_procs = []
    for fid in follower_ids:
        out = args.output_dir / f"drone_{fid}.csv"
        cmd = [
            sys.executable, str(HERE / "realtime_follower.py"),
            "--config",    str(args.config),
            "--formation", str(args.formation),
            "--hw-id",     str(fid),
            "--sim-rate",  str(args.sim_rate),
            "--output",    str(out),
        ]
        if args.smooth:
            cmd += ["--smooth",
                    "--smooth-alpha", str(args.smooth_alpha),
                    "--smooth-beta",  str(args.smooth_beta)]
        follower_procs.append(subprocess.Popen(cmd))

    time.sleep(0.5)  # give followers time to bind

    # --- Launch leader ---
    leader_out = args.output_dir / f"drone_{leader_id}_leader.csv"
    leader_proc = subprocess.Popen([
        sys.executable, str(HERE / "realtime_leader.py"),
        "--config",      str(args.config),
        "--formation",   str(args.formation),
        "--leader-path", str(args.leader_path),
        "--rate",        str(args.leader_rate),
        "--output",      str(leader_out),
    ])

    # --- Wait for everything to finish ---
    leader_rc = leader_proc.wait()
    follower_rcs = [p.wait() for p in follower_procs]
    print(f"[demo] leader exit={leader_rc}  followers exit={follower_rcs}")

    # --- Visualize ---
    if args.visualize:
        subprocess.run([
            sys.executable, str(HERE / "visualize_trajectories.py"),
            "--trajectories-dir", str(args.output_dir),
            "--save-dir",         str(args.output_dir),
            "--no-show",
        ], check=False)
        subprocess.run([
            sys.executable, str(HERE / "animate_trajectories.py"),
            "--trajectories-dir", str(args.output_dir),
            "--output",           str(args.output_dir / "swarm_animation.gif"),
        ], check=False)

    print(f"\n[demo] logs in {args.output_dir}/")


if __name__ == "__main__":
    main()
