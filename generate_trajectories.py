#!/usr/bin/env python3
"""
Offline leader/follower swarm trajectory generator.

Inputs:
  --config     drones_config.json          (identifies leader via is_leader=true)
  --formation  spear_formation_swarm.csv   (hw_id,follow,offset_n,offset_e,offset_alt)
  --leader-path leader_path_with_heading.csv (t,north,east,altitude[,heading])

Output:
  One CSV per drone in --output-dir, schema compatible with offboard_from_csv.py:
    idx,t,px,py,pz,vx,vy,vz,ax,ay,az,yaw,mode,ledr,ledg,ledb
  Coordinates are NED (pz = -altitude). Yaw is radians.

Self-contained: does not depend on coordinator.py / src/ or the sibling
swarm_trajectory_calculator.py. Can be run from inside Simple_swarm/.
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

TRAJ_MODE_MANEUVER = 70
DEFAULT_LED = (255, 255, 255)
LEADER_LED = (255, 64, 64)


def load_drones_config(path: Path):
    """Return (leader_hw_id, [all_hw_ids]) from drones_config.json."""
    with path.open() as f:
        data = json.load(f)
    drones = data["drones"]
    leaders = [d["hw_id"] for d in drones if d.get("is_leader")]
    if len(leaders) != 1:
        raise ValueError(
            f"drones_config.json must have exactly one leader; found {len(leaders)}"
        )
    return leaders[0], [d["hw_id"] for d in drones]


def load_formation(path: Path, leader_hw_id: int):
    """Return {follower_hw_id: {offset_n, offset_e, offset_alt}} for rows that follow leader_hw_id."""
    df = pd.read_csv(path)
    followers = df[df["follow"] == leader_hw_id]
    formation = {}
    for _, row in followers.iterrows():
        formation[int(row["hw_id"])] = {
            "offset_n": float(row["offset_n"]),
            "offset_e": float(row["offset_e"]),
            "offset_alt": float(row["offset_alt"]),
        }
    if not formation:
        raise ValueError(
            f"No followers reference leader hw_id={leader_hw_id} in {path}"
        )
    return formation


def load_leader_path(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path).sort_values("t").reset_index(drop=True)
    required = {"t", "north", "east", "altitude"}
    missing = required - set(df.columns)
    if missing:
        raise ValueError(f"Leader path missing columns: {sorted(missing)}")
    if "heading" not in df.columns:
        # Auto-heading from path tangent, in degrees, 0 = north (atan2(east, north))
        dn = np.gradient(df["north"].to_numpy())
        de = np.gradient(df["east"].to_numpy())
        heading_deg = np.degrees(np.arctan2(de, dn))
        df["heading"] = heading_deg
        print("[info] No heading column; derived heading from path tangent.")
    return df


def follower_positions_over_time(
    leader_df: pd.DataFrame, offset_n: float, offset_e: float, offset_alt: float
):
    """Rotate (offset_n, offset_e) by per-sample leader heading and add to leader position."""
    heading_rad = np.radians(leader_df["heading"].to_numpy())
    cos_h, sin_h = np.cos(heading_rad), np.sin(heading_rad)
    rot_n = offset_n * cos_h - offset_e * sin_h
    rot_e = offset_n * sin_h + offset_e * cos_h
    north = leader_df["north"].to_numpy() + rot_n
    east = leader_df["east"].to_numpy() + rot_e
    alt = leader_df["altitude"].to_numpy() + offset_alt
    return north, east, alt


def derive_velocity_acceleration(t, north, east, alt):
    """Use the actual time column (handles non-uniform sampling)."""
    vx = np.gradient(north, t)          # north velocity
    vy = np.gradient(east, t)           # east velocity
    vz = -np.gradient(alt, t)           # NED down velocity
    ax = np.gradient(vx, t)
    ay = np.gradient(vy, t)
    az = np.gradient(vz, t)
    return vx, vy, vz, ax, ay, az


YAW_MOTION_SPEED_FLOOR = 0.1  # m/s — below this, motion-tangent yaw is unreliable


def motion_tangent_yaw(vx, vy, fallback_rad):
    """Per-sample yaw from horizontal velocity, unwrapped, with low-speed fallback."""
    speed = np.hypot(vx, vy)
    yaw = np.arctan2(vy, vx)
    low = speed < YAW_MOTION_SPEED_FLOOR
    yaw = np.where(low, fallback_rad, yaw)
    return np.unwrap(yaw)


def build_trajectory_df(t, north, east, alt, heading_deg, led_rgb, yaw_mode="motion"):
    """Produce the offboard_from_csv.py-compatible DataFrame.

    yaw_mode:
      "motion" — yaw tracks each drone's own direction of travel in real time.
      "leader" — yaw is forced to the per-sample leader heading (heading_deg arg).
    """
    vx, vy, vz, ax, ay, az = derive_velocity_acceleration(t, north, east, alt)
    leader_yaw_rad = np.radians(heading_deg)
    if yaw_mode == "leader":
        yaw_rad = leader_yaw_rad
    elif yaw_mode == "motion":
        yaw_rad = motion_tangent_yaw(vx, vy, fallback_rad=leader_yaw_rad)
    else:
        raise ValueError(f"yaw_mode must be 'motion' or 'leader', got {yaw_mode!r}")
    n = len(t)
    return pd.DataFrame({
        "idx":  np.arange(n, dtype=int),
        "t":    t,
        "px":   north,
        "py":   east,
        "pz":   -alt,                  # NED: down positive
        "vx":   vx,
        "vy":   vy,
        "vz":   vz,
        "ax":   ax,
        "ay":   ay,
        "az":   az,
        "yaw":  yaw_rad,
        "mode": np.full(n, TRAJ_MODE_MANEUVER, dtype=int),
        "ledr": np.full(n, led_rgb[0], dtype=int),
        "ledg": np.full(n, led_rgb[1], dtype=int),
        "ledb": np.full(n, led_rgb[2], dtype=int),
    })


def generate(config_path, formation_path, leader_path_path, output_dir, yaw_mode="motion"):
    leader_hw_id, all_hw_ids = load_drones_config(config_path)
    formation = load_formation(formation_path, leader_hw_id)
    leader_df = load_leader_path(leader_path_path)

    # Sanity: every follower in formation must be declared in drones_config
    unknown = [fid for fid in formation if fid not in all_hw_ids]
    if unknown:
        print(
            f"[warn] Formation references hw_ids not in drones_config.json: {unknown}",
            file=sys.stderr,
        )

    output_dir.mkdir(parents=True, exist_ok=True)
    t = leader_df["t"].to_numpy()
    written = []

    # Leader keeps its commanded heading — that path was chosen deliberately.
    leader_traj = build_trajectory_df(
        t,
        leader_df["north"].to_numpy(),
        leader_df["east"].to_numpy(),
        leader_df["altitude"].to_numpy(),
        leader_df["heading"].to_numpy(),
        LEADER_LED,
        yaw_mode="leader",
    )
    leader_file = output_dir / f"drone_{leader_hw_id}_leader.csv"
    leader_traj.to_csv(leader_file, index=False)
    written.append(leader_file)

    # Followers
    for hw_id, offs in sorted(formation.items()):
        n, e, a = follower_positions_over_time(
            leader_df, offs["offset_n"], offs["offset_e"], offs["offset_alt"]
        )
        traj = build_trajectory_df(
            t, n, e, a, leader_df["heading"].to_numpy(), DEFAULT_LED, yaw_mode=yaw_mode
        )
        f = output_dir / f"drone_{hw_id}.csv"
        traj.to_csv(f, index=False)
        written.append(f)

    print(f"[ok] Wrote {len(written)} trajectories to {output_dir}")
    for f in written:
        print(f"     {f.name}")
    return written


def main():
    here = Path(__file__).resolve().parent
    repo_root = here.parent
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", type=Path, default=repo_root / "drones_config.json")
    p.add_argument("--formation", type=Path, default=repo_root / "spear_formation_swarm.csv")
    p.add_argument("--leader-path", type=Path, default=repo_root / "leader_path_with_heading.csv")
    p.add_argument("--output-dir", type=Path, default=here / "output")
    p.add_argument(
        "--follower-yaw",
        choices=("leader", "motion"),
        default="leader",
        help="'leader' (default): follower yaw tracks the leader's heading at each timestep (rigid formation). "
             "'motion': follower yaw tracks its own direction of travel (tangent to its path).",
    )
    p.add_argument("--plot", action="store_true", help="Call visualize_trajectories after generating")
    args = p.parse_args()

    generate(args.config, args.formation, args.leader_path, args.output_dir, yaw_mode=args.follower_yaw)

    if args.plot:
        from visualize_trajectories import visualize
        visualize(args.output_dir, show=True, save_dir=args.output_dir)


if __name__ == "__main__":
    main()
