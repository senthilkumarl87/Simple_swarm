import sys, os, numpy as np
from unittest.mock import patch
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from swarm_election import DroneState, ElectionParams, ElectionState, comm_graph, suitability_score, eligibility_vote
from realtime_swarm_dynamic import compute_capacity_from_load

def mk(hw_id, n, e, compute_capacity=1.0):
    return DroneState(hw_id=hw_id, pos=np.array([n, e]), vel=np.zeros(2), compute_capacity=compute_capacity,
                       goal=np.array([n+20, e]))

p = ElectionParams(comm_range_lora=30.0, w_E=0.0, w_P=0.0, w_C=0.0, w_L=1.0)  # isolate L_i's effect

# --- test 1: with all other weights zeroed, a higher compute_capacity must score higher ---
states = {1: mk(1, 0, 0, compute_capacity=0.2), 2: mk(2, 5, 0, compute_capacity=0.9)}
adj = comm_graph(states, p.comm_range_lora)
elig = eligibility_vote([1, 2], states, adj, p)
scores = suitability_score([1, 2], states, adj, elig, p)
assert scores[2] > scores[1], f"expected drone 2 (higher L_i) to score higher, got {scores}"
print(f"test1 (suitability_score responds to L_i when isolated: {scores}) OK")

# --- test 2: election winner flips when only compute_capacity differs ---
es = ElectionState()
winner = es.run_election({1, 2}, states, adj, p, t=0.0, reason="initial")
assert winner == 2, f"expected drone 2 (higher L_i) to win when it's the only differentiator, got {winner}"
print(f"test2 (election winner follows L_i when it's the only differentiator) OK")

# --- test 3: compute_capacity_from_load returns a real, sane value (not the old hardcoded 1.0) ---
cap = compute_capacity_from_load()
assert 0.0 <= cap <= 1.0
print(f"test3 (real CPU-load-based capacity: {cap:.3f}, in valid [0,1] range) OK")

# --- test 4: mock os.getloadavg() so this actually proves the function READS load,
# not just that its real-machine output happens to land in [0,1] -- a hardcoded
# `return 1.0` regression would still pass test 3 above (found via Sourcery review) ---
with patch("realtime_swarm_dynamic.os.getloadavg", return_value=(0.0, 0.0, 0.0)), \
     patch("realtime_swarm_dynamic.os.cpu_count", return_value=4):
    cap_idle = compute_capacity_from_load()
assert cap_idle == 1.0, f"expected 1.0 capacity at zero load, got {cap_idle}"

with patch("realtime_swarm_dynamic.os.getloadavg", return_value=(6.4, 0.0, 0.0)), \
     patch("realtime_swarm_dynamic.os.cpu_count", return_value=4):
    cap_loaded = compute_capacity_from_load()
assert abs(cap_loaded - 0.0) < 1e-9, f"expected 0.0 capacity at load == fully_loaded_at, got {cap_loaded}"

with patch("realtime_swarm_dynamic.os.getloadavg", return_value=(3.2, 0.0, 0.0)), \
     patch("realtime_swarm_dynamic.os.cpu_count", return_value=4):
    cap_half = compute_capacity_from_load()
assert abs(cap_half - 0.5) < 1e-9, f"expected 0.5 capacity at half of fully_loaded_at, got {cap_half}"
print(f"test4 (mocked load -> derived capacity: idle={cap_idle}, half={cap_half}, loaded={cap_loaded}) OK")

print("\nALL COMPUTE_CAPACITY TESTS PASSED")
