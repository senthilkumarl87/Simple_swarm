import sys, os, numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from swarm_election import (DroneState, ElectionParams, ElectionState, comm_graph, wifi_graph, clusters, flock_force, eligibility_vote, suitability_score)

p = ElectionParams(comm_range_lora=30.0)

def mk(hw_id, n, e, energy=1.0):
    return DroneState(hw_id=hw_id, pos=np.array([n, e]), vel=np.zeros(2), energy=energy, goal=np.array([n+20, e]))

states = {1: mk(1, 0, 0), 2: mk(2, 5, 0), 3: mk(3, 100, 0), 4: mk(4, 105, 0)}
adj = comm_graph(states, p.comm_range_lora)
cl = clusters(adj)
assert len(cl) == 2, f"expected 2 clusters, got {cl}"
print("test1 (LoRa-range partition into 2 clusters) OK:", cl)

wg = wifi_graph(states, p.comm_range_wifi)
assert all(len(v) == 0 for v in wg.values()), f"expected empty wifi graph when comm_range_wifi=0, got {wg}"
print("test2 (WiFi absent -> empty wifi_graph, never silently assumed present) OK")

wg2 = wifi_graph(states, comm_range_wifi=10.0)
assert 2 in wg2[1], f"expected drones 1,2 (5m apart) to show up in a 10m wifi_graph, got {wg2}"
adj_unchanged = comm_graph(states, p.comm_range_lora)
assert adj_unchanged == adj, "comm_graph must not be affected by wifi_graph existing"
print("test3 (WiFi present as an overlay, doesn't change the primary LoRa graph) OK:", wg2[1])

states2 = {1: mk(1, 0, 0, energy=1.0), 2: mk(2, 5, 0, energy=0.2), 3: mk(3, 10, 0, energy=0.3)}
adj2 = comm_graph(states2, p.comm_range_lora)
es = ElectionState()
m1 = es.run_election({1,2,3}, states2, adj2, p, t=0.0, reason="initial")
assert m1 == 1
states2[1].energy = 0.05
states2[2].energy = 1.0
m2 = es.run_election({1,2,3}, states2, adj2, p, t=1.0, reason="periodic")
assert m2 == 1, f"hysteresis should still hold, got {m2}"
m3 = es.run_election({1,2,3}, states2, adj2, p, t=10.0, reason="periodic")
assert m3 == 2, f"expected switch after hold time, got {m3}"
print("test4 (election/hysteresis over LoRa graph, unchanged behavior) OK")

print("\nALL TESTS PASSED")
