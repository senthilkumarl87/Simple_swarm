#!/usr/bin/env python3
"""
Realtime swarm demo — LEADER.

Replays a leader_path.csv in wall-clock time, unicasts its current state over
UDP to every follower listed in drones_config.json at --rate Hz, and logs its
own trajectory to --output for later visualization.

State wire format — see realtime_protocol.py.
"""

import argparse
import json
import socket
import sys
import time
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from realtime_protocol import encode_leader_packet, follower_port


def load_followers(config_path: Path, formation_path: Path):
    with config_path.open() as f:
        cfg = json.load(f)
    leaders = [d["hw_id"] for d in cfg["drones"] if d.get("is_leader")]
    if len(leaders) != 1:
        raise ValueError(f"drones_config.json must have exactly one leader; found {len(leaders)}")
    leader_id = int(leaders[0])

    form = pd.read_csv(formation_path)
    formation_followers = set(form[form["follow"] == leader_id]["hw_id"].astype(int))
    follower_ids = [int(d["hw_id"]) for d in cfg["drones"]
                    if not d.get("is_leader") and int(d["hw_id"]) in formation_followers]
    return leader_id, follower_ids


def load_leader_path(path: Path) -> pd.DataFrame:
    """Load the leader path and pre-compute tangent heading (if missing) and velocity."""
    df = pd.read_csv(path).sort_values("t").reset_index(drop=True)
    t = df["t"].to_numpy()
    if "heading" not in df.columns:
        dn = np.gradient(df["north"].to_numpy(), t)
        de = np.gradient(df["east"].to_numpy(),  t)
        df["heading"] = np.degrees(np.arctan2(de, dn))

    # Unwrap heading so interpolation across 360° doesn't go backwards,
    # and so we can take a clean time-derivative for the heading rate.
    df["heading_unwrapped"] = np.degrees(np.unwrap(np.radians(df["heading"].to_numpy())))
    df["vn"] =  np.gradient(df["north"].to_numpy(),     t)
    df["ve"] =  np.gradient(df["east"].to_numpy(),      t)
    df["vd"] = -np.gradient(df["altitude"].to_numpy(),  t)
    df["heading_rate_dps"] = np.gradient(df["heading_unwrapped"].to_numpy(), t)
    return df


def interp_state(df: pd.DataFrame, t_now: float):
    t_arr = df["t"].to_numpy()
    clamp = float(np.clip(t_now, t_arr[0], t_arr[-1]))
    return {
        "n":                float(np.interp(clamp, t_arr, df["north"])),
        "e":                float(np.interp(clamp, t_arr, df["east"])),
        "alt":              float(np.interp(clamp, t_arr, df["altitude"])),
        "heading_deg":      float(np.interp(clamp, t_arr, df["heading_unwrapped"])),
        "vn":               float(np.interp(clamp, t_arr, df["vn"])),
        "ve":               float(np.interp(clamp, t_arr, df["ve"])),
        "vd":               float(np.interp(clamp, t_arr, df["vd"])),
        "heading_rate_dps": float(np.interp(clamp, t_arr, df["heading_rate_dps"])),
    }


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config",      type=Path, required=True)
    p.add_argument("--formation",   type=Path, required=True)
    p.add_argument("--leader-path", type=Path, required=True)
    p.add_argument("--rate",        type=float, default=10.0, help="Broadcast rate (Hz)")
    p.add_argument("--output",      type=Path, required=True, help="Log CSV")
    p.add_argument("--host",        default="127.0.0.1")
    args = p.parse_args()

    leader_id, follower_ids = load_followers(args.config, args.formation)
    path = load_leader_path(args.leader_path)
    t_end = float(path["t"].iloc[-1])

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    targets = [(args.host, follower_port(fid)) for fid in follower_ids]
    period = 1.0 / args.rate

    print(f"[leader {leader_id}] {len(targets)} followers | {args.rate} Hz | "
          f"path duration {t_end:.1f}s | ports {[t[1] for t in targets]}")

    rows = []
    seq = 0
    t_start = time.monotonic()
    while True:
        t_now = time.monotonic() - t_start
        if t_now > t_end:
            break
        st = interp_state(path, t_now)
        pkt = encode_leader_packet(seq, leader_id, t_now, **st)
        for addr in targets:
            try:
                sock.sendto(pkt, addr)
            except OSError as e:
                print(f"[leader] send to {addr} failed: {e}")

        rows.append({
            "idx": seq, "t": t_now,
            "px": st["n"], "py": st["e"], "pz": -st["alt"],
            "vx": st["vn"], "vy": st["ve"], "vz": st["vd"],
            "ax": 0.0, "ay": 0.0, "az": 0.0,
            "yaw": float(np.radians(st["heading_deg"])),
            "mode": 70, "ledr": 255, "ledg": 64, "ledb": 64,
        })
        seq += 1

        # Sleep to next tick (drift-free: target-time based, not cumulative)
        sleep_for = (t_start + seq * period) - time.monotonic()
        if sleep_for > 0:
            time.sleep(sleep_for)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    pd.DataFrame(rows).to_csv(args.output, index=False)
    print(f"[leader {leader_id}] done — {seq} packets, wrote {args.output}")


if __name__ == "__main__":
    main()
