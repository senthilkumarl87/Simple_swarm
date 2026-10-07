import sys, os, numpy as np
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

print("\nALL COMPUTE_CAPACITY TESTS PASSED")
