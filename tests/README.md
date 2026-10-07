# Unit tests for `swarm_election.py`

Plain assert-based scripts (not pytest-style `def test_*()` functions) — each runs its assertions directly when
executed, printing `OK`/`PASSED` on success and raising `AssertionError` on failure. Run individually with
`python3 tests/test_X.py`, or all at once with `tests/run_all.sh`.

These accumulated over the course of building the dynamic-election swarm (`swarm_election.py`,
`realtime_swarm_dynamic.py`) and were only ever run from a session scratchpad directory before being moved here —
no change to their logic, only to the `sys.path` setup so they resolve the repo root portably instead of a
hardcoded machine-specific path.

| File | Covers |
|---|---|
| `test_swarm_election.py` | Core election/hysteresis, LoRa-primary / WiFi-optional comm graphs |
| `test_phase2_phase3_gaps.py` | Merge reconciliation (`MergeState`/`advance_merge_sync`), the ranging-sensor hook (`measured_distance`, fails loudly without a real sensor) |
| `test_phase5_decay.py` | Neighbor-influence decay (ramp vs. hard-cutoff ablation, reconnection recovery) |
| `test_phase5_tiers_rth.py` | Degradation tiers (`tier_of`), isolation → degraded_mode → RTH state machine |
| `test_rth_eligibility.py` | RTH'd drones excluded from election candidacy (hard eligibility floor) |
| `test_compute_capacity.py` | `L_i` / `compute_capacity_from_load` actually affects the suitability score and election outcome |

No fault-injection or SITL/Gazebo coverage here by design — those are exercised live against real PX4/Gazebo (see
`gz_worlds/README.md`), not reproduced as offline unit tests.
