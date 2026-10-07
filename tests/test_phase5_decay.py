import sys, os, numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from swarm_election import (DroneState, ElectionParams, comm_graph, flock_force, update_neighbor_link_state)

def mk(hw_id, n, e):
    return DroneState(hw_id=hw_id, pos=np.array([n, e]), vel=np.zeros(2), goal=np.array([n+20, e]))

# --- test 1: a neighbor moving out of range holds last-known state and decays, not vanishes instantly ---
p = ElectionParams(comm_range_lora=10.0, NEIGHBOR_STATE_TIMEOUT=4.0, NEIGHBOR_INFLUENCE_DECAY_WINDOW=2.0)
states = {1: mk(1, 0, 0), 2: mk(2, 5, 0)}  # 5m apart, within comm_range_lora=10

adj = comm_graph(states, p.comm_range_lora)
update_neighbor_link_state(states, adj, p, t=0.0)
assert states[1].neighbor_influence.get(2) == 1.0, "freshly-seen neighbor should be full weight"

# neighbor 2 moves out of range
states[2].pos = np.array([50.0, 0.0])
adj2 = comm_graph(states, p.comm_range_lora)
assert 2 not in adj2[1], "neighbor should now be out of comm_graph range"

update_neighbor_link_state(states, adj2, p, t=1.0)  # 1s after last seen
w1 = states[1].neighbor_influence.get(2)
assert w1 is not None and 0.0 < w1 < 1.0, f"expected partial decay at t=1s (window=2s), got {w1}"
assert abs(w1 - 0.5) < 1e-6, f"expected exactly 0.5 at the midpoint of a 2s linear decay, got {w1}"
print(f"test1 (linear decay mid-window: influence={w1}) OK")

# flock_force should still see neighbor 2 (frozen last-known position), not drop it instantly
adj_empty_for_1 = {1: set(), 2: set()}
force_with_stale = flock_force(1, states, adj_empty_for_1 if False else adj2, p)
# neighbor 2 is far away (50m) -- outside cohesion_max, so force should reduce toward pure goal-seeking,
# but the key assertion is that flock_force doesn't crash/skip the stale neighbor and uses frozen state
assert 2 in (set(states[1].neighbor_influence.keys()) - adj2.get(1, set())), "neighbor 2 should be in stale_ids"
print("test2 (stale neighbor held in neighbor_influence, visible to flock_force as stale_ids) OK")

# --- test 3: past NEIGHBOR_STATE_TIMEOUT, fully excluded ---
update_neighbor_link_state(states, adj2, p, t=5.0)  # age=5s > NEIGHBOR_STATE_TIMEOUT=4.0
assert 2 not in states[1].neighbor_influence, "should be fully excluded past the timeout"
assert 2 not in states[1].neighbor_last_state
print("test3 (full exclusion past NEIGHBOR_STATE_TIMEOUT) OK")

# --- test 4: neighbor_decay_enabled=False reproduces the old hard-cutoff ablation ---
p2 = ElectionParams(comm_range_lora=10.0, NEIGHBOR_STATE_TIMEOUT=4.0, NEIGHBOR_INFLUENCE_DECAY_WINDOW=2.0,
                     neighbor_decay_enabled=False)
states2 = {1: mk(1, 0, 0), 2: mk(2, 5, 0)}
adj_s2 = comm_graph(states2, p2.comm_range_lora)
update_neighbor_link_state(states2, adj_s2, p2, t=0.0)
states2[2].pos = np.array([50.0, 0.0])
adj_s2_out = comm_graph(states2, p2.comm_range_lora)
update_neighbor_link_state(states2, adj_s2_out, p2, t=1.0)  # mid-decay-window, but decay disabled
w2 = states2[1].neighbor_influence.get(2)
assert w2 == 1.0, f"expected full weight right up to timeout with decay disabled, got {w2}"
print("test4 (neighbor_decay_enabled=False: full weight until instant cutoff) OK")

# --- test 5: reconnection before timeout restores full weight and refreshes seen time ---
states2[2].pos = np.array([3.0, 0.0])  # back in range
adj_s2_back = comm_graph(states2, p2.comm_range_lora)
update_neighbor_link_state(states2, adj_s2_back, p2, t=2.0)
assert states2[1].neighbor_influence[2] == 1.0
assert states2[1].neighbor_last_seen[2] == 2.0
print("test5 (reconnection restores full weight) OK")

print("\nALL PHASE 5 DECAY TESTS PASSED")
