#!/usr/bin/env python3
"""
Realtime swarm demo — FOLLOWER.

Listens for leader state on UDP, computes its target setpoint from the
formation offset rotated by leader heading, tracks it with a PD
controller in a fixed-rate loop, and logs its own state.

Between leader packets the follower linearly extrapolates the leader
position from the last received velocity — so it runs its inner loop at
--sim-rate (default 50 Hz) regardless of how slowly the leader
broadcasts (default 10 Hz).

Exits when the leader has been silent for --silence-timeout seconds.
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
from realtime_protocol import decode_leader_packet, follower_port
from realtime_filters import AlphaBetaFilter


DEFAULT_LIMITS = dict(
    max_speed_xy=10.0,
    max_speed_up=5.0,
    max_speed_down=3.0,
    max_accel_xy=5.0,
    max_accel_z=3.0,
    max_yaw_rate=1.5708,
    kp_pos=4.0,
    kv_vel=3.0,
    kyaw=4.0,
)


def load_my_offset(config_path: Path, formation_path: Path, my_hw_id: int):
    with config_path.open() as f:
        cfg = json.load(f)
    leader_id = int(next(d["hw_id"] for d in cfg["drones"] if d.get("is_leader")))
    form = pd.read_csv(formation_path)
    row = form[(form["hw_id"] == my_hw_id) & (form["follow"] == leader_id)]
    if row.empty:
        raise ValueError(f"hw_id {my_hw_id} is not declared as a follower of leader {leader_id}")
    return (float(row["offset_n"].iloc[0]),
            float(row["offset_e"].iloc[0]),
            float(row["offset_alt"].iloc[0]))


def rotate_offset(offset_n, offset_e, heading_deg):
    th = np.radians(heading_deg)
    c, s = np.cos(th), np.sin(th)
    return offset_n * c - offset_e * s, offset_n * s + offset_e * c


def drain_socket(sock):
    """Return the most recent packet on the socket, or None if empty."""
    latest = None
    while True:
        try:
            data, _ = sock.recvfrom(4096)
            latest = decode_leader_packet(data)
        except BlockingIOError:
            return latest


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config",    type=Path, required=True)
    p.add_argument("--formation", type=Path, required=True)
    p.add_argument("--hw-id",     type=int, required=True)
    p.add_argument("--sim-rate",  type=float, default=50.0, help="Inner loop rate (Hz)")
    p.add_argument("--output",    type=Path, required=True)
    p.add_argument("--silence-timeout", type=float, default=2.0,
                   help="Exit after this many seconds with no leader packets")
    p.add_argument("--first-packet-timeout", type=float, default=15.0,
                   help="Abort startup if no leader packet arrives within this window")
    p.add_argument("--smooth", action="store_true",
                   help="Enable α-β smoothing on received leader position (and derive velocity "
                        "from it). Off by default — recommended only when running across a real "
                        "network where packet jitter/loss is visible; on loopback UDP it only "
                        "adds lag.")
    p.add_argument("--smooth-alpha", type=float, default=0.5,
                   help="α-β filter position gain (ignored unless --smooth)")
    p.add_argument("--smooth-beta", type=float, default=0.1,
                   help="α-β filter velocity gain (ignored unless --smooth)")
    args = p.parse_args()

    on, oe, oa = load_my_offset(args.config, args.formation, args.hw_id)
    limits = DEFAULT_LIMITS
    dt = 1.0 / args.sim_rate

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("127.0.0.1", follower_port(args.hw_id)))
    sock.setblocking(False)

    print(f"[follower {args.hw_id}] offset=({on:+.2f}, {oe:+.2f}, {oa:+.2f})m "
          f"listening on port {follower_port(args.hw_id)}")

    # --- Wait for first leader packet to initialize state ---
    deadline = time.monotonic() + args.first_packet_timeout
    last_packet = None
    while last_packet is None:
        if time.monotonic() > deadline:
            print(f"[follower {args.hw_id}] no leader packet within {args.first_packet_timeout}s; exiting")
            return
        last_packet = drain_socket(sock)
        if last_packet is None:
            time.sleep(0.01)
    last_packet_rx_mono = time.monotonic()

    # Optional α-β smoothing on leader position (three independent axes).
    # When on, we take position + velocity from the filter instead of the
    # raw packet, so filtering is internally consistent across both channels.
    filt_n = filt_e = filt_alt = None
    filter_last_updated = None
    last_seq = -1
    if args.smooth:
        filt_n   = AlphaBetaFilter(args.smooth_alpha, args.smooth_beta)
        filt_e   = AlphaBetaFilter(args.smooth_alpha, args.smooth_beta)
        filt_alt = AlphaBetaFilter(args.smooth_alpha, args.smooth_beta)
        filt_n.reset(last_packet["n"],    last_packet["vn"])
        filt_e.reset(last_packet["e"],    last_packet["ve"])
        filt_alt.reset(last_packet["alt"], -last_packet["vd"])   # alt is +up; vd is NED down
        filter_last_updated = last_packet_rx_mono
        last_seq = int(last_packet.get("seq", -1))
        print(f"[follower {args.hw_id}] α-β smoothing ON (α={args.smooth_alpha}, β={args.smooth_beta})")

    rot_n, rot_e = rotate_offset(on, oe, last_packet["heading_deg"])
    pos = np.array([
        last_packet["n"] + rot_n,
        last_packet["e"] + rot_e,
        -(last_packet["alt"] + oa),
    ])
    vel = np.array([last_packet["vn"], last_packet["ve"], last_packet["vd"]])
    yaw = float(np.radians(last_packet["heading_deg"]))

    rows = []
    seq = 0
    t_start = time.monotonic()
    last_rx_time = t_start

    while True:
        now = time.monotonic()
        if now - last_rx_time > args.silence_timeout:
            print(f"[follower {args.hw_id}] leader silent for {args.silence_timeout}s; exiting")
            break

        latest = drain_socket(sock)
        if latest is not None:
            # Reject out-of-order packets when we have a monotonic seq. With
            # loopback UDP this never trips, but across a real network it
            # keeps the α-β filter from going backwards on reordered UDP.
            pkt_seq = int(latest.get("seq", -1))
            if pkt_seq < last_seq:
                pass    # stale; ignore
            else:
                last_seq = pkt_seq
                last_packet = latest
                last_packet_rx_mono = now
                last_rx_time = now
                if args.smooth:
                    dt_upd = now - (filter_last_updated or now)
                    # First update after init may have dt=0; filter no-ops safely.
                    filt_n.update(latest["n"],    dt_upd)
                    filt_e.update(latest["e"],    dt_upd)
                    filt_alt.update(latest["alt"], dt_upd)
                    filter_last_updated = now

        # --- Extrapolate leader state to "now" ---
        dt_since_rx = now - last_packet_rx_mono
        if args.smooth:
            # Use filter state; velocity is derived from the filter, not the packet.
            dt_pred = now - (filter_last_updated or now)
            lead_n,   lead_vn   = filt_n.predict(dt_pred)
            lead_e,   lead_ve   = filt_e.predict(dt_pred)
            lead_alt, lead_valt = filt_alt.predict(dt_pred)    # valt is +up
            lead_vd = -lead_valt
            # Reload packet-velocity locals so the rest of the loop stays uniform
            pkt_vn, pkt_ve, pkt_vd = lead_vn, lead_ve, lead_vd
        else:
            lead_n   = last_packet["n"]   + last_packet["vn"] * dt_since_rx
            lead_e   = last_packet["e"]   + last_packet["ve"] * dt_since_rx
            lead_alt = last_packet["alt"] + (-last_packet["vd"]) * dt_since_rx
            pkt_vn, pkt_ve, pkt_vd = last_packet["vn"], last_packet["ve"], last_packet["vd"]
        # Heading is not α-β filtered (see README.md §7.4): we use the raw
        # reported heading plus its rate. Filtering heading needs unwrap handling
        # and typically hurts more than it helps given PX4/leader heading is
        # already smooth.
        lead_heading = (last_packet["heading_deg"]
                        + last_packet.get("heading_rate_dps", 0.0) * dt_since_rx)

        # --- Target setpoint from formation offset ---
        rot_n, rot_e = rotate_offset(on, oe, lead_heading)
        target_pos = np.array([lead_n + rot_n, lead_e + rot_e, -(lead_alt + oa)])
        # Feed-forward target velocity = leader linear velocity + rotational
        # velocity of this follower's offset around the leader.
        # d/dt of the rotated offset when heading rotates at ω rad/s:
        #   d(rot_n)/dt = -ω·rot_e
        #   d(rot_e)/dt = +ω·rot_n
        omega = float(np.radians(last_packet.get("heading_rate_dps", 0.0)))
        target_vel = np.array([
            pkt_vn - omega * rot_e,
            pkt_ve + omega * rot_n,
            pkt_vd,
        ])
        target_yaw = float(np.radians(lead_heading))

        # --- PD controller with independent horizontal/vertical saturation ---
        acc = limits["kp_pos"] * (target_pos - pos) + limits["kv_vel"] * (target_vel - vel)
        axy = float(np.linalg.norm(acc[:2]))
        if axy > limits["max_accel_xy"]:
            acc[:2] *= limits["max_accel_xy"] / axy
        acc[2] = float(np.clip(acc[2], -limits["max_accel_z"], limits["max_accel_z"]))

        vel = vel + acc * dt
        vxy = float(np.linalg.norm(vel[:2]))
        if vxy > limits["max_speed_xy"]:
            vel[:2] *= limits["max_speed_xy"] / vxy
        if vel[2] < -limits["max_speed_up"]:
            vel[2] = -limits["max_speed_up"]
        elif vel[2] > limits["max_speed_down"]:
            vel[2] = limits["max_speed_down"]

        pos = pos + vel * dt

        # Yaw: first-order, shortest-angle error, rate-limited
        yaw_err = target_yaw - yaw
        yaw_err = (yaw_err + np.pi) % (2 * np.pi) - np.pi
        yaw_rate = float(np.clip(limits["kyaw"] * yaw_err,
                                 -limits["max_yaw_rate"], limits["max_yaw_rate"]))
        yaw = yaw + yaw_rate * dt

        rows.append({
            "idx": seq, "t": now - t_start,
            "px": pos[0], "py": pos[1], "pz": pos[2],
            "vx": vel[0], "vy": vel[1], "vz": vel[2],
            "ax": acc[0], "ay": acc[1], "az": acc[2],
            "yaw": yaw, "mode": 70,
            "ledr": 255, "ledg": 255, "ledb": 255,
        })
        seq += 1

        sleep_for = (t_start + seq * dt) - time.monotonic()
        if sleep_for > 0:
            time.sleep(sleep_for)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    pd.DataFrame(rows).to_csv(args.output, index=False)
    print(f"[follower {args.hw_id}] done — {seq} ticks, wrote {args.output}")


if __name__ == "__main__":
    main()
