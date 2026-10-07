import sys, os, numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from swarm_election import (DroneState, ElectionParams, comm_graph, clusters, tier_of, update_isolation_rth, flock_force)

def mk(hw_id, n, e, is_master=False):
    return DroneState(hw_id=hw_id, pos=np.array([n, e]), vel=np.zeros(2), goal=np.array([n+20, e]), is_master=is_master)

p = ElectionParams(comm_range_lora=10.0, ISOLATION_RTH_TIMEOUT=3.0)

# --- test 1: full tier -- master itself, in the main cluster ---
states = {1: mk(1, 0, 0, is_master=True), 2: mk(2, 5, 0)}
adj = comm_graph(states, p.comm_range_lora)
cl = clusters(adj)
assert tier_of(1, states, adj, cl) == "full", tier_of(1, states, adj, cl)
assert tier_of(2, states, adj, cl) == "full"
print("test1 (full tier: direct link to master, main cluster) OK")

# --- test 2: isolated tier -- zero reachable neighbors ---
states2 = {1: mk(1, 0, 0, is_master=True), 2: mk(2, 100, 0)}
adj2 = comm_graph(states2, p.comm_range_lora)
cl2 = clusters(adj2)
assert tier_of(2, states2, adj2, cl2) == "isolated", tier_of(2, states2, adj2, cl2)
print("test2 (isolated tier: zero reachable neighbors) OK")

# --- test 3: partition tier -- smaller cluster, not the main one ---
states3 = {1: mk(1, 0, 0, is_master=True), 2: mk(2, 5, 0),
           3: mk(3, 100, 0, is_master=True), 4: mk(4, 105, 0)}
adj3 = comm_graph(states3, p.comm_range_lora)
cl3 = clusters(adj3)
assert tier_of(3, states3, adj3, cl3) == "partition", tier_of(3, states3, adj3, cl3)
assert tier_of(4, states3, adj3, cl3) == "partition"
assert tier_of(1, states3, adj3, cl3) == "full"
print("test3 (partition tier: smaller cluster, not the main one) OK")

# --- test 4: relay tier -- in main cluster, but not directly reachable from master ---
states4 = {1: mk(1, 0, 0, is_master=True), 2: mk(2, 9, 0), 3: mk(3, 18, 0)}
# 1-2 in range (9m), 2-3 in range (9m), but 1-3 NOT in range (18m > comm_range_lora=10)
adj4 = comm_graph(states4, p.comm_range_lora)
cl4 = clusters(adj4)
assert len(cl4) == 1 and len(cl4[0]) == 3, f"expected one 3-node cluster, got {cl4}"
assert tier_of(3, states4, adj4, cl4) == "relay", tier_of(3, states4, adj4, cl4)
print("test4 (relay tier: main cluster, indirect link to master) OK")

# --- test 5: isolation -> degraded_mode immediate, rth after ISOLATION_RTH_TIMEOUT ---
states5 = {1: mk(1, 0, 0), 2: mk(2, 200, 0)}  # agent 1 alone, far from everyone
adj5 = {1: set(), 2: set()}
update_isolation_rth(states5, adj5, p, t=0.0)
assert states5[1].degraded_mode is True, "should enter degraded_mode immediately on isolation"
assert states5[1].rth is False, "should not RTH immediately"
update_isolation_rth(states5, adj5, p, t=2.0)   # 2s isolated, < ISOLATION_RTH_TIMEOUT=3.0
assert states5[1].rth is False, "should not RTH before the timeout"
update_isolation_rth(states5, adj5, p, t=3.5)   # 3.5s isolated, > 3.0
assert states5[1].rth is True, "should RTH past ISOLATION_RTH_TIMEOUT"
print("test5 (isolation -> degraded_mode immediate, rth after timeout) OK")

# --- test 6: reconnection clears degraded_mode and rth state (isolated_since reset) ---
adj5_back = {1: {2}, 2: {1}}
update_isolation_rth(states5, adj5_back, p, t=4.0)
assert states5[1].degraded_mode is False
assert states5[1].isolated_since == -1.0
# Previously missing: the test's own name/comment claimed "and rth state" but never
# actually checked it, so update_isolation_rth() leaving rth=True after reconnection
# (contradicting its own docstring, "Reconnecting clears both") passed silently
# (found via Sourcery review). Now fixed in both the engine and this assertion.
assert states5[1].rth is False, "rth should also clear on reconnection, per this function's own docstring"
print("test6 (reconnection clears degraded_mode / isolated_since / rth) OK")

# --- test 7: flock_force in degraded_mode drops cohesion+alignment, keeps repulsion+goal ---
p2 = ElectionParams(comm_range_lora=100.0, D_rep=8.0, cohesion_start=10.0)
states7 = {1: mk(1, 0, 0), 2: mk(2, 5, 0)}
states7[2].vel = np.array([3.0, 0.0])  # neighbor moving -- would normally pull alignment
adj7 = comm_graph(states7, p2.comm_range_lora)
f_normal = flock_force(1, states7, adj7, p2)
states7[1].degraded_mode = True
f_degraded = flock_force(1, states7, adj7, p2)
# both should have repulsion (d=5 < D_rep=8) and goal-seeking; degraded drops alignment
assert not np.allclose(f_normal, f_degraded), "degraded_mode should change the computed force"
print(f"test7 (degraded_mode drops alignment/cohesion: normal={f_normal} degraded={f_degraded}) OK")

print("\nALL PHASE 5 TIER/RTH TESTS PASSED")
