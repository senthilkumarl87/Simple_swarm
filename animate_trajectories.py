#!/usr/bin/env python3
"""
3D animation of the generated swarm trajectories.

Reads every drone_*.csv in --trajectories-dir and animates:
  * current position of each drone (marker)
  * fading trail of recent positions
  * formation polygon connecting all drones in the current frame
  * heading arrow per drone (from the yaw column)
  * leader highlighted in red

Saves a GIF to --output by default; pass --show to also pop a window.
"""

import argparse
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

DRONE_FILE_RE = re.compile(r"^drone_(\d+)(?:_leader)?\.csv$")


def load_trajectories(trajectories_dir: Path):
    out = {}
    for f in sorted(trajectories_dir.glob("drone_*.csv")):
        m = DRONE_FILE_RE.match(f.name)
        if not m:
            continue
        out[int(m.group(1))] = (pd.read_csv(f), "_leader" in f.name)
    if not out:
        raise FileNotFoundError(f"No drone_*.csv files in {trajectories_dir}")
    return out


def resample_to_frames(trajectories, max_frames):
    """Downsample by index so every drone shares the same frame count.

    All drone trajectories are assumed to share the same `t` column (they
    come from the same leader path). We just pick evenly-spaced indices.
    """
    n = len(next(iter(trajectories.values()))[0])
    if n <= max_frames:
        return np.arange(n)
    return np.linspace(0, n - 1, max_frames, dtype=int)


def axes_limits(trajectories, margin=2.0):
    xs, ys, zs = [], [], []
    for _, (df, _) in trajectories.items():
        xs.append(df["px"].to_numpy())
        ys.append(df["py"].to_numpy())
        zs.append(-df["pz"].to_numpy())
    xs = np.concatenate(xs); ys = np.concatenate(ys); zs = np.concatenate(zs)
    return (
        (xs.min() - margin, xs.max() + margin),
        (ys.min() - margin, ys.max() + margin),
        (zs.min() - margin, zs.max() + margin),
    )


def animate(trajectories_dir: Path, output_path: Path, fps: int,
            trail_length: int, max_frames: int, show: bool):
    trajectories = load_trajectories(trajectories_dir)
    frames = resample_to_frames(trajectories, max_frames)
    xlim, ylim, zlim = axes_limits(trajectories)

    # Arrow length ~5% of the largest axis span
    spans = [xlim[1] - xlim[0], ylim[1] - ylim[0], zlim[1] - zlim[0]]
    arrow_len = 0.05 * max(spans)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlim(xlim); ax.set_ylim(ylim); ax.set_zlim(zlim)
    ax.set_xlabel("North (m)"); ax.set_ylabel("East (m)"); ax.set_zlabel("Altitude (m)")

    ids = list(trajectories.keys())
    is_leader = {i: trajectories[i][1] for i in ids}

    # Pre-build per-drone artists
    trails = {}
    markers = {}
    for i in ids:
        color = "red" if is_leader[i] else None  # matplotlib auto-cycles followers
        (trail_line,) = ax.plot([], [], [], linewidth=1.2, alpha=0.6, color=color)
        (marker,) = ax.plot([], [], [], marker="o", markersize=8 if is_leader[i] else 6,
                            color=trail_line.get_color(), linestyle="")
        trails[i] = trail_line
        markers[i] = marker

    (formation_line,) = ax.plot([], [], [], linewidth=1.4, color="tab:gray", alpha=0.7)
    title = ax.set_title("")
    # 3D quivers can't be updated in place — we recreate them each frame and track the objects.
    heading_arrows = []

    def init():
        for art in list(trails.values()) + list(markers.values()):
            art.set_data_3d([], [], [])
        formation_line.set_data_3d([], [], [])
        return list(trails.values()) + list(markers.values()) + [formation_line, title]

    def update(fi):
        frame_idx = int(frames[fi])
        # per-drone position + trail
        poly_x, poly_y, poly_z = [], [], []
        # clear previous heading arrows
        while heading_arrows:
            heading_arrows.pop().remove()
        for i in ids:
            df = trajectories[i][0]
            x = df["px"].to_numpy()
            y = df["py"].to_numpy()
            z = -df["pz"].to_numpy()
            yaw = df["yaw"].iloc[frame_idx]

            start = max(0, frame_idx - trail_length)
            trails[i].set_data_3d(x[start:frame_idx + 1], y[start:frame_idx + 1], z[start:frame_idx + 1])
            markers[i].set_data_3d([x[frame_idx]], [y[frame_idx]], [z[frame_idx]])

            dx = arrow_len * np.cos(yaw)  # north component
            dy = arrow_len * np.sin(yaw)  # east component
            q = ax.quiver(
                x[frame_idx], y[frame_idx], z[frame_idx],
                dx, dy, 0,
                color="red" if is_leader[i] else "tab:gray",
                arrow_length_ratio=0.35, linewidth=1.2,
            )
            heading_arrows.append(q)

            poly_x.append(x[frame_idx]); poly_y.append(y[frame_idx]); poly_z.append(z[frame_idx])

        # close the formation polygon
        if poly_x:
            poly_x.append(poly_x[0]); poly_y.append(poly_y[0]); poly_z.append(poly_z[0])
            formation_line.set_data_3d(poly_x, poly_y, poly_z)

        t_now = trajectories[ids[0]][0]["t"].iloc[frame_idx]
        hdg_now = np.degrees(trajectories[ids[0]][0]["yaw"].iloc[frame_idx])
        title.set_text(f"t = {t_now:6.2f}s    leader heading = {hdg_now:+.1f}°")
        return list(trails.values()) + list(markers.values()) + [formation_line, title] + heading_arrows

    anim = FuncAnimation(fig, update, init_func=init, frames=len(frames),
                         interval=1000 / fps, blit=False)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    anim.save(output_path, writer=PillowWriter(fps=fps))
    print(f"[ok] Saved {output_path}")

    if show:
        plt.show()
    else:
        plt.close(fig)


def main():
    here = Path(__file__).resolve().parent
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--trajectories-dir", type=Path, default=here / "output")
    p.add_argument("--output", type=Path, default=here / "output" / "swarm_animation.gif")
    p.add_argument("--fps", type=int, default=15)
    p.add_argument("--max-frames", type=int, default=120,
                   help="Downsample long trajectories to at most this many animation frames.")
    p.add_argument("--trail-length", type=int, default=25,
                   help="How many past samples to show as the fading trail.")
    p.add_argument("--show", action="store_true", help="Open an interactive window after saving")
    args = p.parse_args()

    animate(args.trajectories_dir, args.output, args.fps,
            args.trail_length, args.max_frames, args.show)


if __name__ == "__main__":
    main()
