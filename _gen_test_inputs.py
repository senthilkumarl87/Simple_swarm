#!/usr/bin/env python3
"""
Regenerate the test_inputs/ directory with a fixed set of scenarios.

Run once after cloning / when scenarios change:
    python3 _gen_test_inputs.py

Each scenario writes:
    test_inputs/<name>/drones_config.json
    test_inputs/<name>/formation.csv
    test_inputs/<name>/leader_path.csv
    test_inputs/<name>/meta.json
    test_inputs/<name>/limits.json   (only for scenarios that need custom limits)
"""

import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
INPUTS = HERE / "test_inputs"


def write_drones_config(path: Path, hw_ids, leader_id):
    data = {"drones": [
        {"drone_id": i, "hw_id": i, "ip": f"udp://:{14539 + i}",
         "port": 14539 + i, "is_leader": (i == leader_id)}
        for i in hw_ids
    ]}
    path.write_text(json.dumps(data, indent=2) + "\n")


def write_formation(path: Path, leader_id, followers):
    """followers: list of (hw_id, offset_n, offset_e, offset_alt)."""
    lines = ["hw_id,follow,offset_n,offset_e,offset_alt",
             f"{leader_id},0,0,0,0"]
    for hw_id, n, e, a in followers:
        lines.append(f"{hw_id},{leader_id},{n},{e},{a}")
    path.write_text("\n".join(lines) + "\n")


def write_leader_path(path: Path, t, north, east, altitude, heading=None):
    header = "t,north,east,altitude" + (",heading" if heading is not None else "")
    rows = [header]
    n = len(t)
    for i in range(n):
        if heading is not None:
            rows.append(f"{t[i]:.3f},{north[i]:.5f},{east[i]:.5f},{altitude[i]:.5f},{heading[i]:.5f}")
        else:
            rows.append(f"{t[i]:.3f},{north[i]:.5f},{east[i]:.5f},{altitude[i]:.5f}")
    path.write_text("\n".join(rows) + "\n")


def write_meta(path: Path, meta: dict):
    path.write_text(json.dumps(meta, indent=2) + "\n")


# -- Scenario 1: basic spear, gentle curve ------------------------------------

def scenario_01_basic_spear(root: Path):
    d = root / "01_basic_spear"
    d.mkdir(parents=True, exist_ok=True)
    write_drones_config(d / "drones_config.json", hw_ids=[1, 2, 3, 4, 5], leader_id=1)
    write_formation(d / "formation.csv", leader_id=1, followers=[
        (2, 2.0, -2.0, 0.0),
        (3, -2.0, -2.0, 0.0),
        (4, 3.0, -3.0, 0.0),
        (5, -3.0, -3.0, 0.0),
    ])
    t = np.arange(0, 16, 1, dtype=float)
    north = 2.0 * t
    east = 0.1 * t * t
    alt = 10.0 + 0.2 * t
    heading = 5.0 * t
    write_leader_path(d / "leader_path.csv", t, north, east, alt, heading)
    write_meta(d / "meta.json", {
        "name": "01_basic_spear",
        "description": "5-drone spear, gentle curve + climbing. Sanity baseline.",
        "expect_simulation": "ok",
        "expect_peak_err_max": 1.5,
        "expect_min_sep_min": 1.0,
    })


# -- Scenario 2: circle with NO heading (auto-tangent) ------------------------

def scenario_02_circle_auto_heading(root: Path):
    d = root / "02_circle_auto_heading"
    d.mkdir(parents=True, exist_ok=True)
    write_drones_config(d / "drones_config.json", hw_ids=[1, 2, 3], leader_id=1)
    write_formation(d / "formation.csv", leader_id=1, followers=[
        (2, -1.5, -1.5, 0.0),
        (3, -1.5, 1.5, 0.0),
    ])
    t = np.arange(0, 21, 1, dtype=float)
    omega = 2 * np.pi / 20.0
    north = 5.0 * np.cos(omega * t)
    east = 5.0 * np.sin(omega * t)
    alt = np.full_like(t, 10.0)
    # deliberately omit heading — exercises the auto-tangent path in the generator
    write_leader_path(d / "leader_path.csv", t, north, east, alt, heading=None)
    write_meta(d / "meta.json", {
        "name": "02_circle_auto_heading",
        "description": "Circle with heading column omitted; generator must derive tangent heading.",
        "expect_simulation": "ok",
        "expect_peak_err_max": 1.5,
        "expect_min_sep_min": 1.0,
    })


# -- Scenario 3: vertical spiral ---------------------------------------------

def scenario_03_vertical_spiral(root: Path):
    d = root / "03_vertical_spiral"
    d.mkdir(parents=True, exist_ok=True)
    write_drones_config(d / "drones_config.json", hw_ids=[1, 2, 3, 4], leader_id=1)
    write_formation(d / "formation.csv", leader_id=1, followers=[
        (2,  1.0,  1.0, 0.0),
        (3, -1.0,  1.0, 0.0),
        (4,  0.0, -1.5, 0.5),
    ])
    t = np.arange(0, 21, 1, dtype=float)
    period = 10.0
    omega = 2 * np.pi / period
    north = 3.0 * np.cos(omega * t)
    east = 3.0 * np.sin(omega * t)
    alt = 5.0 + 0.3 * t
    heading_deg = (90.0 + np.degrees(omega * t)) % 360.0  # tangent to CCW circle
    write_leader_path(d / "leader_path.csv", t, north, east, alt, heading_deg)
    write_meta(d / "meta.json", {
        "name": "03_vertical_spiral",
        "description": "Helical leader path with climbing altitude. Tests vertical tracking.",
        "expect_simulation": "ok",
        "expect_peak_err_max": 2.0,
        "expect_min_sep_min": 0.8,
    })


# -- Scenario 4: infeasible plan (tight limits must trip the safety check) ---

def scenario_04_infeasible(root: Path):
    d = root / "04_infeasible_tight_limits"
    d.mkdir(parents=True, exist_ok=True)
    write_drones_config(d / "drones_config.json", hw_ids=[1, 2, 3], leader_id=1)
    write_formation(d / "formation.csv", leader_id=1, followers=[
        (2, -1.0, -1.0, 0.0),
        (3, -1.0,  1.0, 0.0),
    ])
    # aggressive leader path: large translations + sharp heading reversal
    t = np.arange(0, 11, 1, dtype=float)
    north = 20.0 * t                      # 20 m/s commanded — fast
    east = 5.0 * np.sin(t)
    alt = 10.0 + 2.0 * t                  # 2 m/s climb
    heading = 45.0 * t                    # 45 deg/s slew
    write_leader_path(d / "leader_path.csv", t, north, east, alt, heading)
    # Intentionally tight limits so the simulator must fail safety checks
    (d / "limits.json").write_text(json.dumps({
        "max_speed_xy": 2.0,
        "max_speed_up": 1.0,
        "max_accel_xy": 1.0,
        "max_accel_z": 0.5,
        "max_yaw_rate": 0.2,
    }, indent=2) + "\n")
    write_meta(d / "meta.json", {
        "name": "04_infeasible_tight_limits",
        "description": "Fast path + tight limits. Expect the simulator to flag infeasibility.",
        "expect_simulation": "fail",
    })


# -- Scenario 5: large swarm, hovering leader rotating in place --------------

def scenario_05_hover_rotate(root: Path):
    d = root / "05_hover_rotate"
    d.mkdir(parents=True, exist_ok=True)
    hw_ids = list(range(1, 9))  # 1 leader + 7 followers
    write_drones_config(d / "drones_config.json", hw_ids=hw_ids, leader_id=1)
    # 7 followers evenly spaced around a circle of radius 4 m at +0.5 m alt
    followers = []
    for k, hw_id in enumerate(hw_ids[1:], start=0):
        ang = 2 * np.pi * k / len(hw_ids[1:])
        followers.append((hw_id, 4.0 * np.cos(ang), 4.0 * np.sin(ang), 0.5))
    write_formation(d / "formation.csv", leader_id=1, followers=followers)
    t = np.arange(0, 13, 1, dtype=float)
    north = np.zeros_like(t)
    east = np.zeros_like(t)
    alt = np.full_like(t, 15.0)
    heading_deg = 30.0 * t  # rotate 360° over 12 s
    write_leader_path(d / "leader_path.csv", t, north, east, alt, heading_deg)
    write_meta(d / "meta.json", {
        "name": "05_hover_rotate",
        "description": "Leader hovers and rotates; 7 followers on a ring must orbit around it.",
        "expect_simulation": "ok",
        "expect_peak_err_max": 1.5,
        "expect_min_sep_min": 2.5,
    })


def main():
    INPUTS.mkdir(parents=True, exist_ok=True)
    scenario_01_basic_spear(INPUTS)
    scenario_02_circle_auto_heading(INPUTS)
    scenario_03_vertical_spiral(INPUTS)
    scenario_04_infeasible(INPUTS)
    scenario_05_hover_rotate(INPUTS)
    for d in sorted(INPUTS.iterdir()):
        print(f"  {d.name}/")
    print(f"[ok] Wrote 5 scenarios under {INPUTS}")


if __name__ == "__main__":
    main()
