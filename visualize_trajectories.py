#!/usr/bin/env python3
"""
Visualize swarm trajectories produced by generate_trajectories.py.

Reads every drone_*.csv in --trajectories-dir and renders:
  1. 3D path plot (all drones)
  2. Top-down (north/east) plot
  3. Altitude vs time
  4. Pairwise minimum separation vs time (safety check)
"""

import argparse
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers 3d projection)

DRONE_FILE_RE = re.compile(r"^drone_(\d+)(?:_leader)?\.csv$")


def load_trajectories(trajectories_dir: Path):
    """Return dict {hw_id: (df, is_leader)} sorted by hw_id."""
    out = {}
    for f in sorted(trajectories_dir.glob("drone_*.csv")):
        m = DRONE_FILE_RE.match(f.name)
        if not m:
            continue
        hw_id = int(m.group(1))
        is_leader = "_leader" in f.name
        out[hw_id] = (pd.read_csv(f), is_leader)
    if not out:
        raise FileNotFoundError(f"No drone_*.csv files in {trajectories_dir}")
    return out


def plot_3d(ax, trajectories):
    for hw_id, (df, is_leader) in trajectories.items():
        style = dict(linewidth=2.5, color="red") if is_leader else dict(linewidth=1.2)
        label = f"Drone {hw_id}" + (" (leader)" if is_leader else "")
        # pz is NED-down; flip sign so up is up in the plot
        px = df["px"].to_numpy()
        py = df["py"].to_numpy()
        alt = -df["pz"].to_numpy()
        ax.plot(px, py, alt, label=label, **style)
        ax.scatter(px[0], py[0], alt[0], marker="o", s=30)
        ax.scatter(px[-1], py[-1], alt[-1], marker="^", s=30)
    ax.set_xlabel("North (m)")
    ax.set_ylabel("East (m)")
    ax.set_zlabel("Altitude (m)")
    ax.set_title("3D trajectories (● start, ▲ end)")
    ax.legend(loc="upper left", fontsize=8)


def plot_top_down(ax, trajectories, heading_arrows=8, formation_snapshots=6):
    """Top-down path plot with per-drone heading arrows and rotating-formation snapshots."""
    # Paths + per-drone heading arrows
    for hw_id, (df, is_leader) in trajectories.items():
        style = dict(linewidth=2.5, color="red") if is_leader else dict(linewidth=1.0)
        px = df["px"].to_numpy()
        py = df["py"].to_numpy()
        yaw = df["yaw"].to_numpy()
        ax.plot(py, px, label=f"Drone {hw_id}", **style)

        n = len(df)
        if n >= 2 and heading_arrows > 0:
            idxs = np.linspace(0, n - 1, min(heading_arrows, n), dtype=int)
            u = np.sin(yaw[idxs])  # east component
            v = np.cos(yaw[idxs])  # north component
            ax.quiver(
                py[idxs], px[idxs], u, v,
                color="red" if is_leader else "tab:gray",
                scale=25, width=0.004, alpha=0.85,
            )

    # Formation snapshots: polygon of all drone positions at sampled timesteps,
    # makes leader-heading rotation of the formation visually obvious.
    # Sampled on time (not index) so differing sample rates between drones still align.
    if formation_snapshots > 0:
        ids = list(trajectories.keys())
        t_min = max(trajectories[i][0]["t"].iloc[0]  for i in ids)
        t_max = min(trajectories[i][0]["t"].iloc[-1] for i in ids)
        snap_ts = np.linspace(t_min, t_max, formation_snapshots)
        cmap = plt.get_cmap("viridis")
        for k, tsn in enumerate(snap_ts):
            color = cmap(k / max(len(snap_ts) - 1, 1))
            pts_n, pts_e = [], []
            for i in ids:
                df = trajectories[i][0]
                pts_n.append(float(np.interp(tsn, df["t"].to_numpy(), df["px"].to_numpy())))
                pts_e.append(float(np.interp(tsn, df["t"].to_numpy(), df["py"].to_numpy())))
            pts_n.append(pts_n[0]); pts_e.append(pts_e[0])  # close polygon
            ax.plot(pts_e, pts_n, color=color, linewidth=1.2, alpha=0.9)
            ax.scatter(pts_e[:-1], pts_n[:-1], color=color, s=18, zorder=5)
            ax.text(pts_e[0], pts_n[0], f" t={tsn:.0f}s",
                    color=color, fontsize=7, va="center")

    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title("Top-down view — arrows = heading, polygons = formation snapshots")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)


def plot_altitude(ax, trajectories):
    for hw_id, (df, is_leader) in trajectories.items():
        style = dict(linewidth=2.5, color="red") if is_leader else dict(linewidth=1.0)
        ax.plot(df["t"].to_numpy(), -df["pz"].to_numpy(), label=f"Drone {hw_id}", **style)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Altitude (m)")
    ax.set_title("Altitude vs time")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)


def plot_formation_rotation(ax, trajectories, snapshots=8):
    """Formation shape with the leader pinned at origin — isolates the rotation by removing
    the leader's translation. Each snapshot is one timestep; color progresses over time.
    Samples every drone by interpolating on time (not index) so differing sample rates
    between leader and followers don't misalign the polygon."""
    ids = list(trajectories.keys())
    leader_id = next((i for i, (_, is_leader) in trajectories.items() if is_leader), None)
    if leader_id is None:
        ax.set_title("Formation rotation (no leader flag in filenames)")
        return
    follower_ids = [i for i in ids if i != leader_id]

    leader_df = trajectories[leader_id][0]
    # Snapshot times sit inside the time span that every drone covers
    t_min = max(trajectories[i][0]["t"].iloc[0]  for i in ids)
    t_max = min(trajectories[i][0]["t"].iloc[-1] for i in ids)
    snap_ts = np.linspace(t_min, t_max, snapshots)
    cmap = plt.get_cmap("viridis")

    ax.axhline(0, color="#ddd", linewidth=0.6, zorder=0)
    ax.axvline(0, color="#ddd", linewidth=0.6, zorder=0)
    ax.scatter(0, 0, marker="*", color="red", s=120, zorder=5, label="Leader (ref frame)")

    lt = leader_df["t"].to_numpy()
    lpx = leader_df["px"].to_numpy(); lpy = leader_df["py"].to_numpy()
    lyaw = np.unwrap(leader_df["yaw"].to_numpy())
    for k, tsn in enumerate(snap_ts):
        color = cmap(k / max(len(snap_ts) - 1, 1))
        lx = float(np.interp(tsn, lt, lpx))
        ly = float(np.interp(tsn, lt, lpy))
        t_label = tsn
        hdg = np.degrees(float(np.interp(tsn, lt, lyaw)))
        pts_n = []
        pts_e = []
        for fid in follower_ids:
            fdf = trajectories[fid][0]
            ft = fdf["t"].to_numpy()
            pts_n.append(float(np.interp(tsn, ft, fdf["px"].to_numpy())) - lx)
            pts_e.append(float(np.interp(tsn, ft, fdf["py"].to_numpy())) - ly)
        # close the polygon
        poly_e = pts_e + [pts_e[0]]
        poly_n = pts_n + [pts_n[0]]
        ax.plot(poly_e, poly_n, color=color, linewidth=1.3, alpha=0.9,
                label=f"t={t_label:.0f}s  hdg={hdg:+.0f}°")
        ax.scatter(pts_e, pts_n, color=color, s=28, zorder=4)

        # leader heading arrow
        ax.arrow(
            0, 0, np.sin(np.radians(hdg)) * 1.2, np.cos(np.radians(hdg)) * 1.2,
            head_width=0.25, color=color, alpha=0.6, length_includes_head=True,
        )

    ax.set_xlabel("East offset from leader (m)")
    ax.set_ylabel("North offset from leader (m)")
    ax.set_title("Formation in leader-translated frame (leader at origin)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=7, ncol=2)


def plot_min_separation(ax, trajectories):
    """At every sample, compute the minimum pairwise distance across drones."""
    ids = list(trajectories.keys())
    if len(ids) < 2:
        ax.set_title("Min separation (needs ≥2 drones)")
        return
    frames = [trajectories[i][0][["t", "px", "py", "pz"]].rename(
        columns={"px": f"n{i}", "py": f"e{i}", "pz": f"d{i}"}
    ) for i in ids]
    merged = frames[0]
    for f in frames[1:]:
        merged = merged.merge(f, on="t", how="inner")

    t = merged["t"].to_numpy()
    min_sep = np.full(len(t), np.inf)
    for idx_a in range(len(ids)):
        for idx_b in range(idx_a + 1, len(ids)):
            a, b = ids[idx_a], ids[idx_b]
            dn = merged[f"n{a}"].to_numpy() - merged[f"n{b}"].to_numpy()
            de = merged[f"e{a}"].to_numpy() - merged[f"e{b}"].to_numpy()
            dd = merged[f"d{a}"].to_numpy() - merged[f"d{b}"].to_numpy()
            sep = np.sqrt(dn * dn + de * de + dd * dd)
            min_sep = np.minimum(min_sep, sep)

    ax.plot(t, min_sep, color="tab:purple")
    ax.axhline(1.0, color="red", linestyle="--", alpha=0.5, label="1 m safety line")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Min pairwise distance (m)")
    ax.set_title("Closest pair over time")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)


def visualize(trajectories_dir: Path, show: bool = True, save_dir: Path | None = None):
    trajectories = load_trajectories(trajectories_dir)

    fig = plt.figure(figsize=(14, 10))
    ax3d = fig.add_subplot(2, 2, 1, projection="3d")
    ax_td = fig.add_subplot(2, 2, 2)
    ax_alt = fig.add_subplot(2, 2, 3)
    ax_sep = fig.add_subplot(2, 2, 4)

    plot_3d(ax3d, trajectories)
    plot_top_down(ax_td, trajectories)
    plot_altitude(ax_alt, trajectories)
    plot_formation_rotation(ax_sep, trajectories)
    fig.tight_layout()

    if save_dir is not None:
        save_dir = Path(save_dir)
        save_dir.mkdir(parents=True, exist_ok=True)
        out = save_dir / "swarm_trajectories.png"
        fig.savefig(out, dpi=150)
        print(f"[ok] Saved {out}")

    if show:
        plt.show()
    else:
        plt.close(fig)


def main():
    here = Path(__file__).resolve().parent
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--trajectories-dir", type=Path, default=here / "output")
    p.add_argument("--no-show", action="store_true", help="Only save the plot; don't open a window")
    p.add_argument("--save-dir", type=Path, default=None, help="If set, saves swarm_trajectories.png here")
    args = p.parse_args()

    visualize(args.trajectories_dir, show=not args.no_show, save_dir=args.save_dir or args.trajectories_dir)


if __name__ == "__main__":
    main()
