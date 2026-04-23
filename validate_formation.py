#!/usr/bin/env python3
"""
Validate spear formation from Option B telemetry.

At every sample after the assembly transient (default: t > 7 s), verify:

  1. Each follower is at  leader_position + R(leader_heading) · offset_body
     — position is maintained under leader motion AND rotation.

  2. The "tip" of the spear always points in the leader's heading
     direction — i.e. the vector from the tail (deepest-aft follower, by
     largest |offset_n| among negative offsets) to the leader is aligned
     with the leader's heading vector.

Reports per-follower tracking error, per-axis decomposition, and the
heading-alignment angle error for the tail → tip vector vs leader heading.
"""

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd


def load_formation(path: Path, leader_id: int):
    df = pd.read_csv(path)
    out = {}
    for _, row in df[df["follow"] == leader_id].iterrows():
        out[int(row["hw_id"])] = (float(row["offset_n"]),
                                  float(row["offset_e"]),
                                  float(row["offset_alt"]))
    return out


def rotate(offset_n, offset_e, heading_deg):
    th = np.radians(heading_deg)
    c, s = np.cos(th), np.sin(th)
    return offset_n * c - offset_e * s, offset_n * s + offset_e * c


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--trajectories-dir", type=Path, required=True,
                    help="Directory with drone_*_leader.csv and drone_*.csv from Option B.")
    ap.add_argument("--config",    type=Path, required=True)
    ap.add_argument("--formation", type=Path, required=True)
    ap.add_argument("--start-after", type=float, default=7.0,
                    help="Skip samples before this t (seconds) — hides the takeoff transient.")
    ap.add_argument("--tol-pos",    type=float, default=2.0,
                    help="Tolerance on peak 3D formation position error (m).")
    ap.add_argument("--tol-heading", type=float, default=5.0,
                    help="Tolerance on heading-alignment angle error (degrees).")
    args = ap.parse_args()

    leader_id = int(next(
        d["hw_id"] for d in json.loads(args.config.read_text())["drones"]
        if d.get("is_leader")))
    formation = load_formation(args.formation, leader_id)

    tdir = args.trajectories_dir
    leader = pd.read_csv(tdir / f"drone_{leader_id}_leader.csv")
    # leader CSV uses the commanded path; leader "yaw" is radians of heading
    t_ref = leader["t"].to_numpy()
    t_valid = t_ref[t_ref >= args.start_after]

    leader_n = np.interp(t_valid, leader["t"], leader["px"])
    leader_e = np.interp(t_valid, leader["t"], leader["py"])
    leader_d = np.interp(t_valid, leader["t"], leader["pz"])
    leader_hdg_rad = np.interp(t_valid, leader["t"], np.unwrap(leader["yaw"].to_numpy()))
    leader_hdg_deg = np.degrees(leader_hdg_rad)

    print(f"Leader hw_id={leader_id}")
    print(f"Followers: {sorted(formation.keys())}")
    print(f"Validation window: t > {args.start_after}s  "
          f"({len(t_valid)} samples across {t_valid[0]:.1f}s → {t_valid[-1]:.1f}s)")
    print()

    # --- Check 1: each follower at leader + R(heading)·offset ---
    print("=== Per-follower formation tracking ===")
    print(f"{'drone':>6}  {'offset_body(n,e,alt)':<22}  {'peak 3D err':>12}  {'rms 3D err':>10}  {'peak N / E / D':>22}")
    all_peaks = []
    for fid, (on, oe, oa) in sorted(formation.items()):
        f = pd.read_csv(tdir / f"drone_{fid}.csv")
        fn = np.interp(t_valid, f["t"], f["px"])
        fe = np.interp(t_valid, f["t"], f["py"])
        fd = np.interp(t_valid, f["t"], f["pz"])

        rot_n, rot_e = rotate(on, oe, leader_hdg_deg)
        expect_n = leader_n + rot_n
        expect_e = leader_e + rot_e
        expect_d = leader_d - oa   # altitude offset is +up, pz is NED down

        err_n = fn - expect_n
        err_e = fe - expect_e
        err_d = fd - expect_d
        err3d = np.sqrt(err_n**2 + err_e**2 + err_d**2)

        peak3d = float(err3d.max())
        rms3d  = float(np.sqrt((err3d**2).mean()))
        peakN = float(np.abs(err_n).max())
        peakE = float(np.abs(err_e).max())
        peakD = float(np.abs(err_d).max())
        all_peaks.append((fid, peak3d, rms3d, peakN, peakE, peakD))
        mark = "  OK" if peak3d <= args.tol_pos else "  FAIL"
        print(f"{fid:>6}  ({on:+.1f},{oe:+.1f},{oa:+.1f})".ljust(30) +
              f"{peak3d:>10.2f} m  {rms3d:>8.2f} m   "
              f"{peakN:>5.2f} / {peakE:>5.2f} / {peakD:>5.2f}{mark}")
    worst_pos_peak = max(p[1] for p in all_peaks)
    pos_ok = worst_pos_peak <= args.tol_pos

    # --- Check 2: tail → leader vector aligned with heading ---
    # Pick the "tail" follower: the one with the smallest offset_n (most negative).
    tail_fid, tail_off = min(formation.items(), key=lambda kv: kv[1][0])
    print()
    print(f"=== Tip-points-to-heading check ===")
    print(f"Tail follower chosen: hw_id={tail_fid}  offset={tail_off}  (most-aft)")

    tail_df = pd.read_csv(tdir / f"drone_{tail_fid}.csv")
    tail_n = np.interp(t_valid, tail_df["t"], tail_df["px"])
    tail_e = np.interp(t_valid, tail_df["t"], tail_df["py"])

    # vector from tail to leader (tip direction)
    vec_n = leader_n - tail_n
    vec_e = leader_e - tail_e
    # angle of that vector in world frame (0 = +N, +90° = +E, compass-style)
    tip_heading_deg = np.degrees(np.arctan2(vec_e, vec_n))

    # shortest-angle difference between leader heading and tip direction, ∈ [-180,180]
    def wrap180(a):
        return ((a + 180.0) % 360.0) - 180.0
    hdg_err = wrap180(tip_heading_deg - leader_hdg_deg)

    peak_hdg_err = float(np.abs(hdg_err).max())
    rms_hdg_err  = float(np.sqrt((hdg_err**2).mean()))
    hdg_ok = peak_hdg_err <= args.tol_heading

    print(f"  Peak |angle error|: {peak_hdg_err:>6.2f}°   (tolerance {args.tol_heading}°)")
    print(f"  RMS  |angle error|: {rms_hdg_err:>6.2f}°")
    print(f"  {'  OK' if hdg_ok else '  FAIL'}")

    # --- Sample a few timesteps for eyeballing ---
    print()
    print("=== Sampled timesteps (actual vs expected for each follower) ===")
    picks = [0, len(t_valid)//4, len(t_valid)//2, 3*len(t_valid)//4, len(t_valid)-1]
    for pi in picks:
        t = t_valid[pi]
        ln, le, lh = leader_n[pi], leader_e[pi], leader_hdg_deg[pi]
        print(f"\n t={t:5.1f}s  leader @ ({ln:+.2f}, {le:+.2f}) hdg={lh:+.1f}°")
        for fid, (on, oe, oa) in sorted(formation.items()):
            rot_n, rot_e = rotate(on, oe, lh)
            exp_n, exp_e = ln + rot_n, le + rot_e
            f = pd.read_csv(tdir / f"drone_{fid}.csv")
            act_n = float(np.interp(t, f["t"], f["px"]))
            act_e = float(np.interp(t, f["t"], f["py"]))
            d = np.sqrt((act_n-exp_n)**2 + (act_e-exp_e)**2)
            print(f"   drone {fid}: expected ({exp_n:+.2f}, {exp_e:+.2f})  "
                  f"actual ({act_n:+.2f}, {act_e:+.2f})  Δ={d:.2f} m")

    print()
    print(f"=== Summary ===")
    print(f"  Peak position error across all followers: {worst_pos_peak:.2f} m "
          f"(tol {args.tol_pos} m) — {'PASS' if pos_ok else 'FAIL'}")
    print(f"  Peak heading-alignment error:             {peak_hdg_err:.2f}° "
          f"(tol {args.tol_heading}°) — {'PASS' if hdg_ok else 'FAIL'}")
    raise SystemExit(0 if (pos_ok and hdg_ok) else 1)


if __name__ == "__main__":
    main()
