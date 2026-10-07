import sys, os, numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from swarm_election import DroneState, ElectionParams, ElectionState, comm_graph

def mk(hw_id, n, e, energy=1.0, rth=False):
    return DroneState(hw_id=hw_id, pos=np.array([n, e]), vel=np.zeros(2), energy=energy,
                       goal=np.array([n+20, e]), rth=rth)

p = ElectionParams(comm_range_lora=30.0)

# --- test 1: an rth=True drone with a clearly dominant score must NOT win the election ---
# energy values were previously backwards here (0.3 for the "dominant" RTH'd drone,
# 0.6 for the winner) -- drone 2 would have won on score alone even with the RTH
# filter removed, so this never actually exercised the exclusion (found via Sourcery
# review). Fixed so drone 1's score is genuinely dominant on every other factor, and
# only the RTH exclusion is what keeps it from winning.
states = {1: mk(1, 0, 0, energy=0.9, rth=True),   # much higher energy but leaving
          2: mk(2, 5, 0, energy=0.3)}
adj = comm_graph(states, p.comm_range_lora)
es = ElectionState()
winner = es.run_election({1, 2}, states, adj, p, t=0.0, reason="initial")
assert winner == 2, f"expected drone 2 (not RTH'd) to win despite lower raw energy, got {winner}"
print(f"test1 (RTH'd drone excluded from candidacy even with a dominant score) OK")

# --- test 2: a lone rth=True drone in its own singleton cluster gets NO master, not itself ---
es2 = ElectionState()
winner2 = es2.run_election({3}, {3: mk(3, 0, 0, rth=True)}, {3: set()}, p, t=0.0, reason="initial")
assert winner2 is None, f"expected no master for a lone RTH'd drone, got {winner2}"
print("test2 (lone RTH'd drone: no master, not self-elected) OK")

# --- test 3: a currently-elected master that then sets rth=True must be replaced next tick ---
states3 = {1: mk(1, 0, 0, energy=0.9), 2: mk(2, 5, 0, energy=0.2)}
adj3 = comm_graph(states3, p.comm_range_lora)
es3 = ElectionState()
w1 = es3.run_election({1, 2}, states3, adj3, p, t=0.0, reason="initial")
assert w1 == 1
states3[1].rth = True   # the current master now needs to leave
w2 = es3.run_election({1, 2}, states3, adj3, p, t=0.1, reason="periodic")
assert w2 == 2, f"expected immediate handover to drone 2 once the master's rth flips True, got {w2}"
print("test3 (sitting master going RTH is immediately replaced, not held by hysteresis) OK")

print("\nALL RTH-ELIGIBILITY TESTS PASSED")
