import sys, os, numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from swarm_election import (DroneState, ElectionParams, ElectionState, MergeState, advance_merge_sync,
                             comm_graph, wifi_graph, clusters, flock_force, update_neighbor_link_state,
                             measured_distance, eligibility_vote)

def mk(hw_id, n, e, energy=1.0):
    return DroneState(hw_id=hw_id, pos=np.array([n, e]), vel=np.zeros(2), energy=energy, goal=np.array([n+20, e]))

p0 = ElectionParams(comm_range_lora=30.0)
states = {1: mk(1, 0, 0), 2: mk(2, 3, 0)}
adj = comm_graph(states, p0.comm_range_lora)
f1_before = flock_force(1, states, adj, p0)
states[2].pos = np.array([15.0, 0.0])
f1_after = flock_force(1, states, adj, p0)
assert not np.allclose(f1_before, f1_after), "disabled staleness should track live position changes instantly"
assert f1_before[0] < 0 and f1_after[0] > 0, "expected repulsion (push away) to flip to cohesion (pull toward)"
print("test1 (staleness disabled = live, unchanged default behavior) OK")

p1 = ElectionParams(comm_range_lora=30.0, lora_broadcast_interval=5.0)
states2 = {1: mk(1, 0, 0), 2: mk(2, 3, 0)}
adj2 = comm_graph(states2, p1.comm_range_lora)
f1 = flock_force(1, states2, adj2, p1)
assert np.allclose(f1, flock_force(1, states2, adj2, p1)), "deterministic before any reception"
update_neighbor_link_state(states2, adj2, p1, t=0.0)
assert 2 in states2[1].neighbor_last_state
stored_pos_initial = states2[1].neighbor_last_state[2][0].copy()
states2[2].pos = np.array([3.0, 10.0])
update_neighbor_link_state(states2, adj2, p1, t=1.0)
assert np.allclose(states2[1].neighbor_last_state[2][0], stored_pos_initial), \
    "should still be using STALE state before the next broadcast is due"
update_neighbor_link_state(states2, adj2, p1, t=5.0)
assert np.allclose(states2[1].neighbor_last_state[2][0], states2[2].pos), \
    "should refresh to the new position once the broadcast interval elapses"
print("test2 (LoRa staleness throttles reception correctly) OK")

p2 = ElectionParams(comm_range_lora=30.0, merge_payload_bytes=1000.0)
states3 = {1: mk(1, 0, 0, energy=1.0), 2: mk(2, 100, 0, energy=0.5)}
es = ElectionState()
adj_split = {1: set(), 2: set()}
es.run_election({1}, states3, adj_split, p2, t=0.0, reason="initial")
es.run_election({2}, states3, adj_split, p2, t=0.0, reason="initial")
assert es.master_of[1] == 1 and es.master_of[2] == 2
assert len(es.pending_merges) == 0
states3[2].pos = np.array([5.0, 0.0])
adj_merged = comm_graph(states3, p2.comm_range_lora)
winner = es.run_election({1, 2}, states3, adj_merged, p2, t=10.0, reason="merge-probe")
assert len(es.pending_merges) == 1, f"expected exactly one pending merge, got {len(es.pending_merges)}"
m = es.pending_merges[0]
assert winner == m.winner
assert {m.winner, m.loser} == {1, 2}
assert m.payload_bytes_total == 1000.0
assert not m.sync_complete
assert es.master_of[1] == winner and es.master_of[2] == winner, \
    f"master_of must be unified for EVERY cluster member after a merge, got {es.master_of}"
print(f"test3 (merge event detected: winner={m.winner} loser={m.loser}) OK")

# --- regression test for the exact bug found live in SITL: calling run_election
# again on the SAME already-merged cluster must NOT re-detect a spurious merge
# (this specifically exercises best == former_masters[0], the case the first
# version of the fix silently skipped unifying master_of for) ---
winner2 = es.run_election({1, 2}, states3, adj_merged, p2, t=10.2, reason="periodic")
assert len(es.pending_merges) == 1, \
    f"re-running election on an already-merged cluster must not create a new merge, got {len(es.pending_merges)}"
assert winner2 == winner
print("test3b (no spurious re-merge on a subsequent tick of an already-merged cluster) OK")

p3 = ElectionParams(lora_bandwidth_bps=800.0, wifi_bandwidth_bps=8_000_000.0, merge_payload_bytes=100.0)
m2 = MergeState(winner=1, loser=2, decided_at=0.0, payload_bytes_total=100.0)
advance_merge_sync(m2, wifi_connected=False, p=p3, dt=0.5)
assert abs(m2.payload_bytes_sent - 50.0) < 1e-6, m2.payload_bytes_sent
assert not m2.sync_complete
advance_merge_sync(m2, wifi_connected=False, p=p3, dt=0.5)
assert m2.sync_complete, "should complete over LoRa alone, just slower -- never requires WiFi"
print("test4a (merge completes over LoRa alone, no WiFi needed) OK")

m3 = MergeState(winner=1, loser=2, decided_at=0.0, payload_bytes_total=100.0)
advance_merge_sync(m3, wifi_connected=True, p=p3, dt=0.0001)
assert m3.payload_bytes_sent > 50.0, "WiFi should transfer far more per tick than LoRa would"
print("test4b (WiFi opportunistically speeds up the same transfer) OK")

p4 = ElectionParams(ranging_available=True)
try:
    measured_distance(mk(1, 0, 0), mk(2, 1, 0), p4)
    assert False, "expected NotImplementedError"
except NotImplementedError:
    print("test5 (ranging_available=True with no real sensor fails loudly, not silently) OK")

print("\nALL PHASE 2/3 GAP TESTS PASSED")
