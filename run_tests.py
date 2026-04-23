#!/usr/bin/env python3
"""
End-to-end test runner for Simple_swarm.

For every scenario in test_inputs/, runs the full pipeline:
  generate_trajectories.py  →  simulate_swarm.py
and verifies the outcome matches meta.json expectations.

Scenario meta.json keys:
  name                  - display name
  expect_simulation     - "ok" | "fail"  (expected simulator exit status)
  expect_peak_err_max   - optional upper bound on max tracking error (meters)
  expect_min_sep_min    - optional lower bound on minimum pairwise separation (meters)
  follower_yaw          - optional, "leader" | "motion"

Exit status: 0 if every scenario passes, 1 otherwise.
"""

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
INPUTS_DIR = HERE / "test_inputs"
OUTPUTS_DIR = HERE / "test_outputs"
GENERATE = HERE / "generate_trajectories.py"
SIMULATE = HERE / "simulate_swarm.py"


PEAK_ERR_RE = re.compile(r"^\s*\d+\s+([\d.]+)\s+([\d.]+)", re.MULTILINE)
MIN_SEP_RE = re.compile(r"Min pairwise separation:\s+([\d.]+)")


def parse_report(stdout: str):
    """Pull (max peak_err, min pairwise separation) from simulate_swarm.py's report."""
    peak_errs = [float(m.group(1)) for m in PEAK_ERR_RE.finditer(stdout)]
    max_peak = max(peak_errs) if peak_errs else None
    sep_match = MIN_SEP_RE.search(stdout)
    min_sep = float(sep_match.group(1)) if sep_match else None
    return max_peak, min_sep


def run_scenario(scenario_dir: Path, verbose: bool):
    meta = json.loads((scenario_dir / "meta.json").read_text())
    name = meta.get("name", scenario_dir.name)
    out_dir = OUTPUTS_DIR / scenario_dir.name
    generated_dir = out_dir / "generated"
    simulated_dir = out_dir / "simulated"

    # --- 1. Generate
    gen_cmd = [
        sys.executable, str(GENERATE),
        "--config",      str(scenario_dir / "drones_config.json"),
        "--formation",   str(scenario_dir / "formation.csv"),
        "--leader-path", str(scenario_dir / "leader_path.csv"),
        "--output-dir",  str(generated_dir),
    ]
    if "follower_yaw" in meta:
        gen_cmd += ["--follower-yaw", meta["follower_yaw"]]
    g = subprocess.run(gen_cmd, capture_output=True, text=True)
    if g.returncode != 0:
        return False, f"generate failed: {g.stderr.strip() or g.stdout.strip()}"

    # --- 2. Simulate
    sim_cmd = [
        sys.executable, str(SIMULATE),
        "--trajectories-dir", str(generated_dir),
        "--output-dir",       str(simulated_dir),
    ]
    if (scenario_dir / "limits.json").exists():
        sim_cmd += ["--limits", str(scenario_dir / "limits.json")]
    s = subprocess.run(sim_cmd, capture_output=True, text=True)
    sim_ok = (s.returncode == 0)
    report = s.stdout

    # --- 3. Check expectations
    expect = meta.get("expect_simulation", "ok")
    if expect == "ok" and not sim_ok:
        return False, f"expected simulation ok, got failure. Report:\n{report}"
    if expect == "fail" and sim_ok:
        return False, "expected simulation to fail, but it passed"

    max_peak, min_sep = parse_report(report)
    detail = []
    if "expect_peak_err_max" in meta and max_peak is not None:
        cap = meta["expect_peak_err_max"]
        if max_peak > cap:
            return False, f"max peak tracking err {max_peak:.2f} m > cap {cap} m"
        detail.append(f"peak_err≤{cap}m (saw {max_peak:.2f})")
    if "expect_min_sep_min" in meta and min_sep is not None:
        floor = meta["expect_min_sep_min"]
        if min_sep < floor:
            return False, f"min pairwise separation {min_sep:.2f} m < floor {floor} m"
        detail.append(f"min_sep≥{floor}m (saw {min_sep:.2f})")

    detail.append(f"sim {'ok' if sim_ok else 'failed (expected)'}")
    if verbose:
        detail.append(f"output={simulated_dir.relative_to(HERE)}")
    return True, "; ".join(detail)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--filter", default=None, help="Only run scenarios whose folder name contains this substring.")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    if not INPUTS_DIR.exists():
        print(f"[fatal] {INPUTS_DIR} does not exist. Run: python3 _gen_test_inputs.py")
        sys.exit(2)

    scenarios = sorted(d for d in INPUTS_DIR.iterdir() if d.is_dir())
    if args.filter:
        scenarios = [d for d in scenarios if args.filter in d.name]
    if not scenarios:
        print("[fatal] no scenarios to run")
        sys.exit(2)

    all_ok = True
    for s in scenarios:
        ok, msg = run_scenario(s, args.verbose)
        status = "PASS" if ok else "FAIL"
        print(f"[{status}] {s.name}: {msg}")
        all_ok &= ok

    print("\n" + ("All scenarios passed." if all_ok else "One or more scenarios FAILED."))
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
