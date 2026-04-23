"""
Shared UDP protocol for the loopback real-time swarm demo.

Packet is a tiny JSON object so it's easy to eyeball in tcpdump and to
swap for a binary format later without churning the business logic.
"""

import json

# Follower hw_id N binds to 127.0.0.1:REALTIME_PORT_BASE + N.
# Chosen to stay clear of the MAVSDK (14540+) and GCS (34550) port ranges
# used elsewhere in the repo.
REALTIME_PORT_BASE = 37540


def follower_port(hw_id: int) -> int:
    return REALTIME_PORT_BASE + int(hw_id)


def encode_leader_packet(seq, hw_id, t_leader,
                         n, e, alt, heading_deg,
                         vn, ve, vd,
                         heading_rate_dps=0.0) -> bytes:
    """heading_rate_dps is dθ/dt in degrees/second; followers need it to
    feed-forward the rotational component of their target velocity when
    the leader is turning (especially while hovering-and-rotating)."""
    return json.dumps({
        "seq":              int(seq),
        "hw_id":            int(hw_id),
        "t_leader":         float(t_leader),
        "n":                float(n),
        "e":                float(e),
        "alt":              float(alt),
        "heading_deg":      float(heading_deg),
        "vn":               float(vn),
        "ve":               float(ve),
        "vd":               float(vd),
        "heading_rate_dps": float(heading_rate_dps),
    }).encode("utf-8")


def decode_leader_packet(data: bytes) -> dict:
    return json.loads(data.decode("utf-8"))
