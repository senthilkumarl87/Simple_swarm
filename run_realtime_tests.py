#!/usr/bin/env python3
"""
Test procedure for the loopback real-time swarm demo (Option C).

For each scenario in test_inputs/ (except the intentionally-infeasible one,
which has no meaning at real-time / kinematic level), runs the full
leader + followers pipeline end-to-end and verifies:

  1. All processes exit cleanly (return code 0).
  2. Leader emitted roughly the expected number of packets
     (leader_rate × path_duration, within ±10%).
  3. Leader loop period stayed close to the target (jitter p95 < 30 ms).
  4. Follower loop period stayed close to the target (jitter p95 < 30 ms).
  5. Each follower's achieved trajectory tracks the offline-planner ground
     truth within a per-scenario tolerance (peak 3D position error).

Exit status: 0 if every scenario passes, 1 otherwise.
"""

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
INPUTS_DIR = HERE / "test_inputs"
OUTPUTS_DIR = HERE / "test_outputs"
RT_OUTPUTS_DIR = HERE / "test_outputs_realtime"

# Per-scenario tolerance on max 3D tracking error (realtime achieved vs offline commanded).
# Realtime path has an initial alignment transient because the follower starts
# holding formation only after the first packet, and PD lag accumulates on sharp turns.
DEFAULT_TRACKING_TOL_M = 1.0
TRACKING_TOL_M = {
    "01_basic_spear":         0.6,
    "02_circle_auto_heading": 0.6,
    "03_vertical_spiral":     1.0,
    "05_hover_rotate":        0.8,
}

SKIP_SCENARIOS = {"04_infeasible_tight_limits"}  # kinematic safety test, not a realtime test

LEADER_RATE = 10.0   # Hz
SIM_RATE = 50.0      # Hz


def load_scenario(scenario_dir: Path):
    meta = json.loads((scenario_dir / "meta.json").read_text())
    cfg = json.loads((scenario_dir / "drones_config.json").read_text())
    form = pd.read_csv(scenario_dir / "formation.csv")
    leader_path = pd.read_csv(scenario_dir / "leader_path.csv")
    leader_id = int(next(d["hw_id"] for d in cfg["drones"] if d.get("is_leader")))
    formation_followers = set(form[form["follow"] == leader_id]["hw_id"].astype(int))
    follower_ids = [int(d["hw_id"]) for d in cfg["drones"]
                    if not d.get("is_leader") and int(d["hw_id"]) in formation_followers]
    duration = float(leader_path["t"].iloc[-1])
    return meta, leader_id, follower_ids, duration


def ensure_offline_ground_truth(scenario_dir: Path):
    """Generate the offline trajectory if run_tests.py hasn't been run yet."""
    generated = OUTPUTS_DIR / scenario_dir.name / "generated"
    if generated.exists() and any(generated.glob("drone_*.csv")):
        return generated
    cmd = [
        sys.executable, str(HERE / "generate_trajectories.py"),
        "--config",      str(scenario_dir / "drones_config.json"),
        "--formation",   str(scenario_dir / "formation.csv"),
        "--leader-path", str(scenario_dir / "leader_path.csv"),
        "--output-dir",  str(generated),
    ]
    subprocess.run(cmd, check=True, capture_output=True)
    return generated


def timing_jitter(df: pd.DataFrame, target_period: float):
    """Return (max absolute delta-t deviation, p95 deviation) in ms."""
    t = df["t"].to_numpy()
    if len(t) < 3:
        return float("inf"), float("inf")
    dts = np.diff(t)
    dev = np.abs(dts - target_period) * 1000.0   # ms
    return float(dev.max()), float(np.percentile(dev, 95))


def run_scenario(scenario_dir: Path, verbose: bool):
    meta, leader_id, follower_ids, duration = load_scenario(scenario_dir)
    name = scenario_dir.name

    rt_out = RT_OUTPUTS_DIR / name
    rt_out.mkdir(parents=True, exist_ok=True)
    # Clean any stale artifacts so leftover CSVs don't mask a broken run
    for f in rt_out.glob("drone_*.csv"):
        f.unlink()

    ground_truth_dir = ensure_offline_ground_truth(scenario_dir)

    # --- Launch demo ---
    cmd = [
        sys.executable, str(HERE / "run_realtime_demo.py"),
        "--config",      str(scenario_dir / "drones_config.json"),
        "--formation",   str(scenario_dir / "formation.csv"),
        "--leader-path", str(scenario_dir / "leader_path.csv"),
        "--output-dir",  str(rt_out),
        "--leader-rate", str(LEADER_RATE),
        "--sim-rate",    str(SIM_RATE),
    ]
    wall_start = time.monotonic()
    proc = subprocess.run(cmd, capture_output=True, text=True,
                          timeout=duration + 30)
    wall_elapsed = time.monotonic() - wall_start

    checks = []
    if proc.returncode != 0:
        return False, f"orchestrator exit={proc.returncode}\n{proc.stderr}"

    # --- (1) Every expected log exists ---
    leader_csv = rt_out / f"drone_{leader_id}_leader.csv"
    follower_csvs = {fid: rt_out / f"drone_{fid}.csv" for fid in follower_ids}
    missing = [p.name for p in [leader_csv, *follower_csvs.values()] if not p.exists()]
    if missing:
        return False, f"missing log files: {missing}"

    # --- (2) Leader packet count ~= rate × duration ---
    leader_df = pd.read_csv(leader_csv)
    expected = LEADER_RATE * duration
    lo, hi = 0.9 * expected, 1.1 * expected
    if not (lo <= len(leader_df) <= hi):
        return False, f"leader emitted {len(leader_df)} packets, expected ~{expected:.0f}"
    checks.append(f"leader pkts={len(leader_df)}/{expected:.0f}")

    # --- (3) Leader timing jitter ---
    jit_max, jit_p95 = timing_jitter(leader_df, 1.0 / LEADER_RATE)
    if jit_p95 > 30.0:
        return False, f"leader p95 jitter {jit_p95:.1f} ms > 30 ms"
    checks.append(f"lead_jit_p95={jit_p95:.1f}ms")

    # --- (4) Follower timing jitter ---
    worst_follower_jitter = 0.0
    for fid, f in follower_csvs.items():
        df = pd.read_csv(f)
        _, p95 = timing_jitter(df, 1.0 / SIM_RATE)
        worst_follower_jitter = max(worst_follower_jitter, p95)
    if worst_follower_jitter > 30.0:
        return False, f"worst follower p95 jitter {worst_follower_jitter:.1f} ms > 30 ms"
    checks.append(f"fol_jit_p95={worst_follower_jitter:.1f}ms")

    # --- (5) Tracking error vs offline ground truth ---
    tol = TRACKING_TOL_M.get(name, DEFAULT_TRACKING_TOL_M)
    peak_err_worst = 0.0
    for fid, f in follower_csvs.items():
        rt = pd.read_csv(f)
        gt = pd.read_csv(ground_truth_dir / f"drone_{fid}.csv")
        # Interpolate realtime onto ground-truth timestamps; skip the first second
        # to let the PD transient after initialization settle.
        t_gt = gt["t"].to_numpy()
        window = t_gt >= 1.0
        def ip(col):
            return np.interp(t_gt[window], rt["t"].to_numpy(), rt[col].to_numpy())
        err = np.sqrt((gt["px"][window].to_numpy() - ip("px")) ** 2
                      + (gt["py"][window].to_numpy() - ip("py")) ** 2
                      + (gt["pz"][window].to_numpy() - ip("pz")) ** 2)
        peak = float(err.max())
        if peak > tol:
            return False, f"drone {fid} peak tracking err {peak:.3f} m > tol {tol} m"
        peak_err_worst = max(peak_err_worst, peak)
    checks.append(f"track_peak={peak_err_worst:.2f}m≤{tol}m")

    checks.append(f"wall={wall_elapsed:.1f}s")
    return True, "  ".join(checks)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--filter", default=None, help="Only run scenarios whose folder name contains this substring.")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    scenarios = sorted(d for d in INPUTS_DIR.iterdir() if d.is_dir())
    scenarios = [s for s in scenarios if s.name not in SKIP_SCENARIOS]
    if args.filter:
        scenarios = [s for s in scenarios if args.filter in s.name]
    if not scenarios:
        print("no scenarios to run"); sys.exit(2)

    print(f"Running {len(scenarios)} realtime scenario(s) — each runs in wall-clock time.\n")
    all_ok = True
    for s in scenarios:
        t0 = time.monotonic()
        ok, msg = run_scenario(s, args.verbose)
        took = time.monotonic() - t0
        status = "PASS" if ok else "FAIL"
        print(f"[{status}] {s.name} ({took:.1f}s): {msg}")
        all_ok &= ok

    print("\n" + ("All realtime scenarios passed." if all_ok else "One or more scenarios FAILED."))
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
