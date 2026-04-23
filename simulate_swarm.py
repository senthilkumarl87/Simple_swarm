#!/usr/bin/env python3
"""
Lightweight kinematic playback simulator for the generated swarm trajectories.

Reads the commanded trajectories produced by generate_trajectories.py,
replays them as setpoints into a per-drone double-integrator PD tracker with
velocity/acceleration/yaw-rate saturation, and reports how cleanly the
commanded plan can actually be flown.

Outputs
  <output_dir>/simulated/drone_<id>[_leader].csv   — achieved trajectories
                                                     (same schema as the commanded CSVs)
  console summary                                   — tracking error, saturation %,
                                                     min pairwise separation, altitude floor

Exit code is 1 if any safety threshold is exceeded (useful for CI).
"""

import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np
import pandas as pd

DRONE_FILE_RE = re.compile(r"^drone_(\d+)(?:_leader)?\.csv$")

DEFAULT_LIMITS = dict(
    max_speed_xy=10.0,    # m/s horizontal
    max_speed_up=5.0,     # m/s climb
    max_speed_down=3.0,   # m/s descent
    max_accel_xy=5.0,     # m/s^2 horizontal
    max_accel_z=3.0,      # m/s^2 vertical
    max_yaw_rate=1.5708,  # rad/s (~90 deg/s)
    kp_pos=4.0,
    kv_vel=3.0,
    kyaw=4.0,
    sim_dt=0.02,          # 50 Hz integrator
)

DEFAULT_THRESHOLDS = dict(
    max_tracking_error=2.0,  # m — per-drone peak |cmd - actual|
    min_separation=0.5,      # m — closest allowed pair distance
    min_altitude=0.0,        # m AGL
)


def load_commanded(trajectories_dir: Path):
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


def upsample(df: pd.DataFrame, sim_dt: float) -> pd.DataFrame:
    """Linearly interpolate the commanded CSV onto a regular sim_dt grid.
    Yaw is unwrapped first so angular wraps don't corrupt the interpolation."""
    t_cmd = df["t"].to_numpy()
    t_sim = np.arange(t_cmd[0], t_cmd[-1] + sim_dt * 0.5, sim_dt)
    yaw_unwrapped = np.unwrap(df["yaw"].to_numpy())
    return pd.DataFrame({
        "t": t_sim,
        "px":  np.interp(t_sim, t_cmd, df["px"]),
        "py":  np.interp(t_sim, t_cmd, df["py"]),
        "pz":  np.interp(t_sim, t_cmd, df["pz"]),
        "vx":  np.interp(t_sim, t_cmd, df["vx"]),
        "vy":  np.interp(t_sim, t_cmd, df["vy"]),
        "vz":  np.interp(t_sim, t_cmd, df["vz"]),
        "yaw": np.interp(t_sim, t_cmd, yaw_unwrapped),
    })


def simulate_one(cmd: pd.DataFrame, limits: dict):
    """Integrate one drone forward through the commanded setpoints.

    Model: double integrator on position/velocity plus first-order yaw.
    Control law: acc = kp*(pos_set - pos) + kv*(vel_set - vel), saturated.
    Horizontal and vertical axes are saturated independently."""
    n = len(cmd)
    dt = limits["sim_dt"]
    pos = np.array([cmd["px"].iloc[0], cmd["py"].iloc[0], cmd["pz"].iloc[0]], dtype=float)
    vel = np.array([cmd["vx"].iloc[0], cmd["vy"].iloc[0], cmd["vz"].iloc[0]], dtype=float)
    yaw = float(cmd["yaw"].iloc[0])

    sat_accel = sat_vel = sat_yaw = 0
    out_pos = np.empty((n, 3))
    out_vel = np.empty((n, 3))
    out_acc = np.empty((n, 3))
    out_yaw = np.empty(n)

    for i in range(n):
        pos_set = np.array([cmd["px"].iloc[i], cmd["py"].iloc[i], cmd["pz"].iloc[i]])
        vel_set = np.array([cmd["vx"].iloc[i], cmd["vy"].iloc[i], cmd["vz"].iloc[i]])
        yaw_set = cmd["yaw"].iloc[i]

        # PD control
        acc = limits["kp_pos"] * (pos_set - pos) + limits["kv_vel"] * (vel_set - vel)

        # Horizontal accel clamp (norm)
        acc_xy_norm = float(np.linalg.norm(acc[:2]))
        if acc_xy_norm > limits["max_accel_xy"]:
            acc[:2] *= limits["max_accel_xy"] / acc_xy_norm
            sat_accel += 1

        # Vertical accel clamp (signed)
        az_unclipped = acc[2]
        acc[2] = np.clip(az_unclipped, -limits["max_accel_z"], limits["max_accel_z"])
        if acc[2] != az_unclipped:
            sat_accel += 1

        # Integrate
        vel = vel + acc * dt

        # Horizontal speed clamp
        vxy_norm = float(np.linalg.norm(vel[:2]))
        if vxy_norm > limits["max_speed_xy"]:
            vel[:2] *= limits["max_speed_xy"] / vxy_norm
            sat_vel += 1

        # Vertical speed clamp (NED: vz<0 is climb, vz>0 is descent)
        if vel[2] < -limits["max_speed_up"]:
            vel[2] = -limits["max_speed_up"]
            sat_vel += 1
        elif vel[2] > limits["max_speed_down"]:
            vel[2] = limits["max_speed_down"]
            sat_vel += 1

        pos = pos + vel * dt

        # Yaw: first-order tracker with rate limit
        yaw_err = yaw_set - yaw
        yaw_rate_cmd = limits["kyaw"] * yaw_err
        if abs(yaw_rate_cmd) > limits["max_yaw_rate"]:
            sat_yaw += 1
        yaw_rate_cmd = float(np.clip(yaw_rate_cmd, -limits["max_yaw_rate"], limits["max_yaw_rate"]))
        yaw = yaw + yaw_rate_cmd * dt

        out_pos[i] = pos
        out_vel[i] = vel
        out_acc[i] = acc
        out_yaw[i] = yaw

    achieved = pd.DataFrame({
        "idx":  np.arange(n, dtype=int),
        "t":    cmd["t"].to_numpy(),
        "px":   out_pos[:, 0],
        "py":   out_pos[:, 1],
        "pz":   out_pos[:, 2],
        "vx":   out_vel[:, 0],
        "vy":   out_vel[:, 1],
        "vz":   out_vel[:, 2],
        "ax":   out_acc[:, 0],
        "ay":   out_acc[:, 1],
        "az":   out_acc[:, 2],
        "yaw":  out_yaw,
        "mode": np.full(n, 70, dtype=int),
        "ledr": np.full(n, 255, dtype=int),
        "ledg": np.full(n, 255, dtype=int),
        "ledb": np.full(n, 255, dtype=int),
    })

    # Per-drone tracking error
    err = np.sqrt((cmd["px"] - achieved["px"]) ** 2
                  + (cmd["py"] - achieved["py"]) ** 2
                  + (cmd["pz"] - achieved["pz"]) ** 2).to_numpy()

    stats = dict(
        sat_accel_pct=100.0 * sat_accel / n,
        sat_vel_pct=100.0 * sat_vel / n,
        sat_yaw_pct=100.0 * sat_yaw / n,
        peak_pos_err=float(err.max()),
        rms_pos_err=float(np.sqrt(np.mean(err ** 2))),
    )
    return achieved, stats


def pairwise_min_separation(sim_results):
    """At each timestep, min distance between any two drones (across all pairs).
    Returns (t, min_sep) arrays."""
    ids = list(sim_results.keys())
    if len(ids) < 2:
        df0 = sim_results[ids[0]][0]
        return df0["t"].to_numpy(), np.full(len(df0), np.inf)
    t = sim_results[ids[0]][0]["t"].to_numpy()
    min_sep = np.full_like(t, np.inf)
    for a in range(len(ids)):
        for b in range(a + 1, len(ids)):
            da = sim_results[ids[a]][0]
            db = sim_results[ids[b]][0]
            d = np.sqrt((da["px"] - db["px"]) ** 2 +
                        (da["py"] - db["py"]) ** 2 +
                        (da["pz"] - db["pz"]) ** 2).to_numpy()
            min_sep = np.minimum(min_sep, d)
    return t, min_sep


def print_report(stats_by_id, min_sep, min_alt, thresholds, is_leader):
    ok = True
    print("\n=== Simulation report ===")
    print(f"{'drone':>6}  {'peak err':>10}  {'rms err':>10}  {'sat a%':>7}  {'sat v%':>7}  {'sat y%':>7}")
    for hw_id, s in sorted(stats_by_id.items()):
        tag = " (leader)" if is_leader[hw_id] else ""
        marker = "  ✗" if s["peak_pos_err"] > thresholds["max_tracking_error"] else ""
        ok &= s["peak_pos_err"] <= thresholds["max_tracking_error"]
        print(f"{hw_id:>6}  {s['peak_pos_err']:>10.3f}  {s['rms_pos_err']:>10.3f}  "
              f"{s['sat_accel_pct']:>7.1f}  {s['sat_vel_pct']:>7.1f}  {s['sat_yaw_pct']:>7.1f}{marker}{tag}")

    print(f"\nMin pairwise separation: {min_sep:.3f} m (threshold {thresholds['min_separation']} m)")
    if min_sep < thresholds["min_separation"]:
        ok = False
        print("  ✗ SAFETY: drones got too close.")

    print(f"Min altitude:            {min_alt:.3f} m (threshold {thresholds['min_altitude']} m)")
    if min_alt < thresholds["min_altitude"]:
        ok = False
        print("  ✗ SAFETY: altitude below floor.")

    if ok:
        print("\nAll checks passed.")
    else:
        print("\nOne or more checks FAILED.")
    return ok


def animate_overlay(cmd_by_id, sim_by_id, is_leader, save_path: Path,
                    fps: int = 15, max_frames: int = 120, trail_length: int = 25):
    """3D animation with commanded (ghost) and achieved (solid) positions per drone,
    plus a line connecting them that visualizes instantaneous tracking error."""
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    ids = list(sim_by_id.keys())
    n_samples = len(sim_by_id[ids[0]])
    frames = np.arange(n_samples) if n_samples <= max_frames else \
        np.linspace(0, n_samples - 1, max_frames, dtype=int)

    # Axis limits from combined commanded + simulated
    xs, ys, zs = [], [], []
    for i in ids:
        for df in (cmd_by_id[i], sim_by_id[i]):
            xs.append(df["px"].to_numpy())
            ys.append(df["py"].to_numpy())
            zs.append(-df["pz"].to_numpy())
    xs = np.concatenate(xs); ys = np.concatenate(ys); zs = np.concatenate(zs)
    margin = 2.0
    xlim = (xs.min() - margin, xs.max() + margin)
    ylim = (ys.min() - margin, ys.max() + margin)
    zlim = (zs.min() - margin, zs.max() + margin)

    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlim(xlim); ax.set_ylim(ylim); ax.set_zlim(zlim)
    ax.set_xlabel("North (m)"); ax.set_ylabel("East (m)"); ax.set_zlabel("Altitude (m)")

    # Per-drone artists
    sim_trails, sim_markers, cmd_ghosts, err_lines = {}, {}, {}, {}
    for i in ids:
        leader = is_leader[i]
        color = "red" if leader else None
        (trail,)   = ax.plot([], [], [], linewidth=1.2, alpha=0.7, color=color)
        (marker,)  = ax.plot([], [], [], marker="o", markersize=8 if leader else 6,
                             linestyle="", color=trail.get_color())
        (ghost,)   = ax.plot([], [], [], marker="o", markersize=8 if leader else 6,
                             markerfacecolor="none", linestyle="",
                             markeredgecolor=trail.get_color(), alpha=0.55)
        (err_ln,)  = ax.plot([], [], [], linewidth=0.8, linestyle=":",
                             color=trail.get_color(), alpha=0.7)
        sim_trails[i]  = trail
        sim_markers[i] = marker
        cmd_ghosts[i]  = ghost
        err_lines[i]   = err_ln

    (formation_line,) = ax.plot([], [], [], linewidth=1.3, color="tab:gray", alpha=0.75)
    title = ax.set_title("")

    # Legend stub
    ax.plot([], [], [], "o", markerfacecolor="k", markeredgecolor="k",
            linestyle="", label="achieved")
    ax.plot([], [], [], "o", markerfacecolor="none", markeredgecolor="k",
            linestyle="", label="commanded")
    ax.legend(loc="upper left", fontsize=8)

    def update(fi):
        idx = int(frames[fi])
        poly_x, poly_y, poly_z = [], [], []
        max_err = 0.0

        for i in ids:
            sim = sim_by_id[i]
            cmd = cmd_by_id[i]

            sx = sim["px"].to_numpy(); sy = sim["py"].to_numpy(); sz = -sim["pz"].to_numpy()
            cx = cmd["px"].to_numpy(); cy = cmd["py"].to_numpy(); cz = -cmd["pz"].to_numpy()

            start = max(0, idx - trail_length)
            sim_trails[i].set_data_3d(sx[start:idx + 1], sy[start:idx + 1], sz[start:idx + 1])
            sim_markers[i].set_data_3d([sx[idx]], [sy[idx]], [sz[idx]])
            cmd_ghosts[i].set_data_3d([cx[idx]], [cy[idx]], [cz[idx]])
            err_lines[i].set_data_3d([sx[idx], cx[idx]], [sy[idx], cy[idx]], [sz[idx], cz[idx]])

            err = float(np.sqrt((sx[idx] - cx[idx]) ** 2 +
                                (sy[idx] - cy[idx]) ** 2 +
                                (sz[idx] - cz[idx]) ** 2))
            if err > max_err:
                max_err = err

            poly_x.append(sx[idx]); poly_y.append(sy[idx]); poly_z.append(sz[idx])

        if poly_x:
            poly_x.append(poly_x[0]); poly_y.append(poly_y[0]); poly_z.append(poly_z[0])
            formation_line.set_data_3d(poly_x, poly_y, poly_z)

        t_now = sim_by_id[ids[0]]["t"].iloc[idx]
        title.set_text(f"t = {t_now:6.2f}s    max tracking error = {max_err:.2f} m")
        return [title]

    anim = FuncAnimation(fig, update, frames=len(frames),
                         interval=1000 / fps, blit=False)
    save_path.parent.mkdir(parents=True, exist_ok=True)
    anim.save(save_path, writer=PillowWriter(fps=fps))
    plt.close(fig)
    print(f"[ok] Saved overlay animation to {save_path}")


def plot_overlay(commanded, simulated, save_path: Path):
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    fig = plt.figure(figsize=(14, 6))
    ax3d = fig.add_subplot(1, 2, 1, projection="3d")
    ax_err = fig.add_subplot(1, 2, 2)

    for hw_id in commanded:
        cmd = commanded[hw_id][0]
        sim = simulated[hw_id][0]
        is_leader = commanded[hw_id][1]
        base_color = "red" if is_leader else None
        (l_cmd,) = ax3d.plot(cmd["px"].to_numpy(), cmd["py"].to_numpy(), -cmd["pz"].to_numpy(),
                             linewidth=1.8, color=base_color,
                             label=f"cmd {hw_id}" + (" (leader)" if is_leader else ""))
        ax3d.plot(sim["px"].to_numpy(), sim["py"].to_numpy(), -sim["pz"].to_numpy(),
                  linewidth=1.0, linestyle="--", color=l_cmd.get_color(),
                  label=f"sim {hw_id}")

        err = np.sqrt((cmd["px"] - sim["px"]) ** 2
                      + (cmd["py"] - sim["py"]) ** 2
                      + (cmd["pz"] - sim["pz"]) ** 2)
        ax_err.plot(cmd["t"].to_numpy(), err.to_numpy(), color=l_cmd.get_color(),
                    label=f"Drone {hw_id}")

    ax3d.set_xlabel("North (m)"); ax3d.set_ylabel("East (m)"); ax3d.set_zlabel("Altitude (m)")
    ax3d.set_title("Commanded (solid) vs simulated (dashed)")
    ax3d.legend(fontsize=7, loc="upper left")

    ax_err.set_xlabel("Time (s)"); ax_err.set_ylabel("Tracking error (m)")
    ax_err.set_title("Position tracking error"); ax_err.grid(True, alpha=0.3)
    ax_err.legend(fontsize=8)

    fig.tight_layout()
    save_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(save_path, dpi=150)
    plt.close(fig)
    print(f"[ok] Saved overlay plot to {save_path}")


def main():
    here = Path(__file__).resolve().parent
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--trajectories-dir", type=Path, default=here / "output",
                   help="Directory containing commanded drone_*.csv files.")
    p.add_argument("--output-dir", type=Path, default=here / "output" / "simulated",
                   help="Where to write the achieved trajectories.")
    p.add_argument("--limits", type=Path, default=None,
                   help="Optional JSON file overriding default kinematic limits.")
    p.add_argument("--plot", action="store_true",
                   help="Save an overlay plot (commanded vs simulated) next to output-dir.")
    p.add_argument("--animate", action="store_true",
                   help="Save a 3D overlay GIF (commanded ghost vs achieved solid) next to output-dir.")
    p.add_argument("--fps", type=int, default=15)
    p.add_argument("--max-frames", type=int, default=120)
    args = p.parse_args()

    limits = dict(DEFAULT_LIMITS)
    if args.limits:
        with args.limits.open() as f:
            limits.update(json.load(f))

    thresholds = dict(DEFAULT_THRESHOLDS)

    commanded = load_commanded(args.trajectories_dir)
    print(f"[info] Loaded {len(commanded)} commanded trajectories from {args.trajectories_dir}")

    simulated = {}
    stats_by_id = {}
    is_leader = {}
    args.output_dir.mkdir(parents=True, exist_ok=True)

    # Step 1: upsample each commanded trajectory to sim rate
    upsampled = {hw_id: upsample(df, limits["sim_dt"]) for hw_id, (df, _) in commanded.items()}

    # Step 2: simulate each drone independently
    for hw_id, cmd_up in upsampled.items():
        achieved, stats = simulate_one(cmd_up, limits)
        simulated[hw_id] = (achieved, commanded[hw_id][1])
        stats_by_id[hw_id] = stats
        is_leader[hw_id] = commanded[hw_id][1]

        name = f"drone_{hw_id}_leader.csv" if is_leader[hw_id] else f"drone_{hw_id}.csv"
        achieved.to_csv(args.output_dir / name, index=False)

    # Step 3: global safety aggregates
    _, sep = pairwise_min_separation(simulated)
    min_sep = float(np.min(sep))
    min_alt = float(min(-s[0]["pz"].min() for s in simulated.values()))  # NED: alt = -pz

    ok = print_report(stats_by_id, min_sep, min_alt, thresholds, is_leader)
    print(f"\n[info] Wrote {len(simulated)} simulated trajectories to {args.output_dir}")

    if args.animate:
        # Both commanded (upsampled) and simulated are at sim rate and share timestamps.
        animate_overlay(
            cmd_by_id={i: upsampled[i] for i in simulated},
            sim_by_id={i: df for i, (df, _) in simulated.items()},
            is_leader=is_leader,
            save_path=args.output_dir / "commanded_vs_simulated.gif",
            fps=args.fps,
            max_frames=args.max_frames,
        )

    if args.plot:
        # Downsample simulated back to commanded timestamps so overlay looks clean
        downsampled_sim = {}
        for hw_id, (sim_df, leader) in simulated.items():
            cmd_df = commanded[hw_id][0]
            t_cmd = cmd_df["t"].to_numpy()
            yaw_unwrapped = np.unwrap(sim_df["yaw"].to_numpy())
            down = pd.DataFrame({
                "t":  t_cmd,
                "px": np.interp(t_cmd, sim_df["t"], sim_df["px"]),
                "py": np.interp(t_cmd, sim_df["t"], sim_df["py"]),
                "pz": np.interp(t_cmd, sim_df["t"], sim_df["pz"]),
                "yaw": np.interp(t_cmd, sim_df["t"], yaw_unwrapped),
            })
            downsampled_sim[hw_id] = (down, leader)
        plot_overlay(commanded, downsampled_sim, args.output_dir / "commanded_vs_simulated.png")

    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
