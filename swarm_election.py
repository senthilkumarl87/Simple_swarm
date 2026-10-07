"""
Dynamic master election + three-force flocking -- pure logic, no MAVSDK dependency.

Ported from the research workspace's reference implementation
(swarm_files/swarm_sim_core.py in the mbc3 project) onto real per-drone telemetry
instead of a simulated Agent population. Horizontal-plane (NED north/east) only,
mirroring the 2D reference sim; altitude is held/commanded separately by the caller.

Design intentionally mirrors swarm_sim_core.py function-for-function so behavior
already validated there (see uav_swarm_fault_tolerance_proposal.tex, Section 10)
carries over:
  - _eligibility_vote   -> eligibility_vote
  - _suitability_score  -> suitability_score
  - _tie_break_key      -> tie_break_key
  - _run_election       -> run_election (same hysteresis branch structure)
  - _flock_force        -> flock_force (same repulsion/cohesion/alignment/goal terms)

KNOWN GAP (see UAV_Swarm_Update_Claude_Code_Spec.md, Appendix C, Phase 2): the
eligibility gate's d_AB is supposed to come from an independent ranging sensor
(UWB, etc.), not from the same GPS stream as the self-report. No such sensor is
wired up yet, so d_AB here is derived from the same MAVSDK position telemetry as
d_hat_AB -- the gate is exercised and testable (via --byzantine fault injection,
which applies a synthetic offset to a drone's self-report only) but is NOT a real
Byzantine/GPS-spoofing defense until a real independent ranging source replaces
this. Documented here rather than silently assumed to work.

RADIO MODEL (Appendix C, Phase 2): Tomoto (LoRa) is the PRIMARY link and must be
sufficient on its own -- comm_graph(), election, eligibility, and flocking are all
keyed to comm_range_lora. WiFi is OPTIONAL: comm_range_wifi defaults to 0 (absent),
and nothing in this module requires it. Where this module is extended for bulk
state transfer (merge reconciliation's D_merged), that must work over LoRa alone
as the baseline, with WiFi used only opportunistically as a speedup -- see
wifi_graph() below, kept separate from comm_graph() on purpose so a caller can
never accidentally make correctness depend on it.
"""

from __future__ import annotations

import itertools
from dataclasses import dataclass, field

import numpy as np


@dataclass
class ElectionParams:
    comm_range_lora: float = 150.0          # m, PRIMARY link -- election/eligibility/flocking all key to this
    comm_range_wifi: float = 0.0            # m, OPTIONAL accelerant; 0 = absent. Never required for correctness.
    max_relay_hops: int = 1                  # Tomoto is broadcast with no addressed routing (confirmed) -- this
                                              # is the correct value for this radio, not a placeholder to raise.

    MASTER_SWITCH_MARGIN: float = 0.05
    MASTER_MIN_HOLD_TIME: float = 5.0        # s
    w_E: float = 0.40
    w_P: float = 0.25
    w_C: float = 0.25
    w_L: float = 0.10
    eligibility_tau: float = 2.0             # m, pairwise distance-consistency tolerance
    ranging_available: bool = False          # True only once real independent ranging hardware (e.g. UWB) is
                                              # wired up -- see measured_distance() below. Tomoto has none
                                              # (confirmed), so this stays False on the target hardware; flipping
                                              # it with no real sensor behind it would be lying to the gate.

    # Radio realism (Appendix C Phase 2): 0.0 (default) preserves flock_force's
    # original always-live-telemetry behavior exactly. Set > 0 to make it use
    # throttled (received-on-schedule, possibly stale) neighbor state instead,
    # modeling Tomoto/LoRa's real broadcast rate -- see
    # update_neighbor_reception() below. Must be called once per control-loop
    # tick for this to take effect; flock_force() alone does not advance time.
    lora_broadcast_interval: float = 0.0     # s, 0 = disabled (continuous/live, WiFi-like assumption)
    lora_bandwidth_bps: float = 1000.0       # bits/s, conservative placeholder pending a real Tomoto measurement
    wifi_bandwidth_bps: float = 1_000_000.0  # bits/s, conservative placeholder for an opportunistic WiFi link
    merge_payload_bytes: float = 2048.0      # placeholder D_merged (coverage/task database) size -- the real
                                              # task-allocation feature that would define this isn't built yet;
                                              # this lets the merge TIMING mechanism be built and tested now
                                              # without waiting on that unrelated feature.

    D_safe: float = 5.0
    D_rep: float = 8.0
    repulsion_gain: float = 4.0
    cohesion_start: float = 10.0
    cohesion_max: float = 40.0
    cohesion_gain: float = 0.6
    alignment_max_dist: float = 25.0
    alignment_gain: float = 0.6
    max_accel: float = 3.0
    max_speed: float = 4.0


@dataclass
class DroneState:
    hw_id: int
    pos: np.ndarray            # [n, e], m
    vel: np.ndarray            # [vn, ve], m/s
    energy: float = 1.0        # 0..1, from battery.remaining_percent / 100
    compute_capacity: float = 1.0   # placeholder -- not wired to real CPU load yet
    healthy: bool = True
    byzantine: bool = False
    spoof_offset: np.ndarray = field(default_factory=lambda: np.zeros(2))
    goal: np.ndarray = field(default_factory=lambda: np.zeros(2))
    is_master: bool = False
    master_since: float = 0.0

    # LoRa broadcast-reception throttle (only used when p.lora_broadcast_interval > 0):
    # per-neighbor last-received (position, velocity) and next-due reception time.
    # Distinct from live ground truth -- see update_neighbor_reception() below.
    neighbor_last_state: dict = field(default_factory=dict)
    neighbor_next_rx_due: dict = field(default_factory=dict)

    def reported_pos(self) -> np.ndarray:
        return self.pos + (self.spoof_offset if self.byzantine else 0.0)


def measured_distance(si: DroneState, sj: DroneState, p: ElectionParams) -> float:
    """The eligibility gate's independently-measured d_AB (Appendix A). Single
    integration point for real ranging hardware: today p.ranging_available is
    always False on the target hardware (Tomoto has no ranging, confirmed), so
    this falls back to the same GPS-derived position as the self-report --
    meaning the gate only catches a self-report that's inconsistent with itself
    across neighbor pairs, not one that's wrong but internally consistent. If
    real ranging hardware (e.g. UWB) is added later, this is the one function
    to change -- not eligibility_vote() itself."""
    if p.ranging_available:
        raise NotImplementedError(
            "ranging_available=True but no real ranging source is wired up -- "
            "implement the real sensor read here before enabling this flag"
        )
    return float(np.linalg.norm(si.pos - sj.pos))


# ---------------------------------------------------------------- comm graph ----

def comm_graph(states: dict[int, DroneState], comm_range: float) -> dict[int, set[int]]:
    """The PRIMARY (LoRa) link graph. Election, eligibility, and flocking are all
    computed against this -- call with p.comm_range_lora. Must never be called with
    the optional WiFi range in place of this; use wifi_graph() for that."""
    ids = [i for i, s in states.items() if s.healthy]
    adj = {i: set() for i in ids}
    for i, j in itertools.combinations(ids, 2):
        d = float(np.linalg.norm(states[i].pos - states[j].pos))
        if d <= comm_range:
            adj[i].add(j)
            adj[j].add(i)
    return adj


def wifi_graph(states: dict[int, DroneState], comm_range_wifi: float) -> dict[int, set[int]]:
    """The OPTIONAL (WiFi) accelerant graph. Empty whenever comm_range_wifi <= 0
    (WiFi absent), by construction -- a caller that only ever consults this when
    it's non-empty can't accidentally make correctness depend on WiFi being
    present. Intended use: check `j in wifi_graph(...).get(i, set())` before
    attempting a fast bulk transfer (e.g. merge reconciliation's D_merged); fall
    back to a chunked transfer over the LoRa link otherwise."""
    if comm_range_wifi <= 0:
        return {i: set() for i in states}
    return comm_graph(states, comm_range_wifi)


def update_neighbor_reception(states: dict[int, DroneState], adj: dict[int, set[int]],
                               p: ElectionParams, t: float) -> None:
    """Throttles how often each drone's view of a reachable neighbor's state
    actually refreshes, modeling Tomoto/LoRa's real broadcast rate instead of
    assuming continuous live telemetry -- mirrors swarm_sim_core.py's
    _update_link_failure_state refresh block (Appendix C, Phase 2).

    No-op when p.lora_broadcast_interval <= 0 (the default), so every scenario
    that doesn't opt into this keeps flock_force()'s original always-live
    behavior exactly. MUST be called once per control-loop tick, before
    flock_force(), for throttling to take effect -- flock_force() itself does
    not advance time or know what tick it's on.
    """
    if p.lora_broadcast_interval <= 0:
        return
    for i, a in states.items():
        if not a.healthy:
            continue
        for j in adj.get(i, set()):
            nb = states.get(j)
            if nb is None:
                continue
            due = a.neighbor_next_rx_due.get(j)
            if due is None or t >= due:
                a.neighbor_last_state[j] = (nb.pos.copy(), nb.vel.copy())
                a.neighbor_next_rx_due[j] = t + p.lora_broadcast_interval


def clusters(adj: dict[int, set[int]]) -> list[set[int]]:
    seen = set()
    out = []
    for start in adj:
        if start in seen:
            continue
        stack, comp = [start], set()
        while stack:
            u = stack.pop()
            if u in comp:
                continue
            comp.add(u)
            stack.extend(adj[u] - comp)
        seen |= comp
        out.append(comp)
    return out


# ------------------------------------------------------------------ election ----

def eligibility_vote(cluster_ids, states: dict[int, DroneState], adj, p: ElectionParams) -> dict[int, bool]:
    """Pairwise distance-consistency vote (swarm_sim_core.py::_eligibility_vote)."""
    votes = {i: 0 for i in cluster_ids}
    for i, j in itertools.combinations(cluster_ids, 2):
        if j not in adj.get(i, set()):
            continue
        si, sj = states[i], states[j]
        d_hat = float(np.linalg.norm(si.reported_pos() - sj.reported_pos()))
        d_meas = measured_distance(si, sj, p)
        if abs(d_hat - d_meas) < p.eligibility_tau:
            votes[i] += 1
            votes[j] += 1
        else:
            votes[i] -= 1
            votes[j] -= 1
    return {i: v >= 0 for i, v in votes.items()}


def suitability_score(cluster_ids, states: dict[int, DroneState], adj, eligible: dict[int, bool],
                       p: ElectionParams) -> dict[int, float]:
    """swarm_sim_core.py::_suitability_score."""
    centroid = np.mean([states[i].pos for i in cluster_ids], axis=0)
    max_dist = p.comm_range_lora * p.max_relay_hops
    scores = {}
    for i in cluster_ids:
        s = states[i]
        E_i = float(np.clip(s.energy, 0, 1))
        P_i = float(np.clip(1 - np.linalg.norm(s.pos - centroid) / max_dist, 0, 1))
        verified_neighbors = sum(
            1 for j in adj.get(i, set()) if j in cluster_ids and eligible.get(j, False)
        )
        denom = max(len(cluster_ids) - 1, 1)
        C_i = float(np.clip(verified_neighbors / denom, 0, 1))
        L_i = float(np.clip(s.compute_capacity, 0, 1))
        scores[i] = p.w_E * E_i + p.w_P * P_i + p.w_C * C_i + p.w_L * L_i
    return scores


def tie_break_key(i, scores, cluster_ids, adj, eligible, states):
    verified_neighbors = sum(
        1 for j in adj.get(i, set()) if j in cluster_ids and eligible.get(j, False)
    )
    return (round(scores[i], 6), verified_neighbors, round(states[i].energy, 6), -i)


# ---------------------------------------------------------- merge reconciliation ----

@dataclass
class MergeState:
    """An in-progress merge after two previously-separate partitions reconnect
    (proposal Section 7.1's algobox). Makes explicit the partial-connectivity
    case Appendix C Phase 2 identifies: a winner can be decided over LoRa
    (small payload -- scores/IDs, steps 1-3 of the algobox) before the bulk
    D_merged database sync (steps 4-5) actually completes, since that may have
    to happen over Tomoto's low bandwidth alone rather than assuming a fast
    link is available. `winner`/`loser` match the algobox's terms directly;
    the loser demotes to slave once sync_complete, not before (step 3 of the
    algobox happens immediately via the election itself -- what this tracks
    is specifically whether the state the loser needs to adopt has actually
    arrived yet)."""
    winner: int
    loser: int
    decided_at: float
    payload_bytes_total: float
    payload_bytes_sent: float = 0.0

    @property
    def sync_complete(self) -> bool:
        return self.payload_bytes_sent >= self.payload_bytes_total


def advance_merge_sync(merge: MergeState, wifi_connected: bool, p: ElectionParams, dt: float) -> None:
    """Advances one control-loop tick's worth of a pending merge's bulk
    D_merged transfer, chunked over whichever link is actually available this
    tick. WiFi is used opportunistically when in range (wifi_connected, from
    wifi_graph()) purely as a speedup -- never a requirement, per the
    LoRa-primary design rule; Tomoto/LoRa's rate is always the floor this
    still has to complete at. Mutates `merge` in place."""
    rate_bps = p.wifi_bandwidth_bps if wifi_connected else p.lora_bandwidth_bps
    merge.payload_bytes_sent = min(
        merge.payload_bytes_total,
        merge.payload_bytes_sent + rate_bps * dt / 8.0,
    )


class ElectionState:
    """Per-cluster persistent election state (current master + hold timer),
    keyed by a stable identity derived from cluster membership so a partition
    and a merge each naturally start a fresh hysteresis clock for the new
    cluster shape, matching swarm_sim_core.py's per-cluster-id bookkeeping."""

    def __init__(self):
        self.master_of: dict[int, int] = {}       # hw_id -> current master hw_id, per drone's cluster
        self.pending_merges: list[MergeState] = []  # merges whose D_merged sync hasn't completed yet
        self.master_since: dict[int, float] = {}   # master hw_id -> t when it became master
        self.switch_log: list[tuple] = []

    def run_election(self, cluster_ids: set[int], states: dict[int, DroneState], adj,
                      p: ElectionParams, t: float, reason: str):
        cluster_ids = sorted(cluster_ids)
        eligible_map = eligibility_vote(cluster_ids, states, adj, p)
        eligible_ids = [i for i in cluster_ids if eligible_map[i]]

        # Collect EVERY distinct former master still present in this cluster, not
        # just the first one found -- when a partition heals, both sides' former
        # masters show up here, which is exactly the merge-event signal (see
        # pending_merges below). A plain partition or a no-op tick has at most one.
        former_masters = sorted({
            self.master_of.get(i) for i in cluster_ids
            if self.master_of.get(i) is not None and self.master_of.get(i) in cluster_ids
        })

        if not eligible_ids:
            for i in cluster_ids:
                self.master_of[i] = None
            return None

        scores = suitability_score(cluster_ids, states, adj, eligible_map, p)
        best = max(eligible_ids, key=lambda i: tie_break_key(i, scores, cluster_ids, adj, eligible_map, states))

        if len(former_masters) > 1:
            # Merge reconciliation (proposal Section 7.1 algobox): forced,
            # immediate convergence to the tie-break winner for EVERY cluster
            # member -- NOT routed through the hysteresis-gated switch below.
            # Must unify master_of unconditionally here, even when best already
            # equals former_masters[0]: skipping that case (as an earlier version
            # did, via a shared do_switch() that only ran on the non-merge path)
            # left OTHER members' master_of permanently pointing at their own
            # former local master, which re-detected "merge" on every single
            # subsequent tick forever. Found by watching pending_merges grow
            # unboundedly in a live SITL run rather than trusting the first
            # green unit test, which happened not to exercise best==former_masters[0].
            for loser in former_masters:
                if loser != best:
                    self.pending_merges.append(MergeState(
                        winner=best, loser=loser, decided_at=t,
                        payload_bytes_total=p.merge_payload_bytes,
                    ))
            for i in cluster_ids:
                self.master_of[i] = best
            self.master_since[best] = t
            self.switch_log.append((t, tuple(cluster_ids), tuple(former_masters), best, f"merge ({reason})"))
            return best

        current_master = former_masters[0] if former_masters else None

        def do_switch(new_id, why):
            for i in cluster_ids:
                self.master_of[i] = new_id
            self.master_since[new_id] = t
            self.switch_log.append((t, tuple(cluster_ids), current_master, new_id, why))

        if current_master is None:
            do_switch(best, reason)
            return best
        if current_master not in eligible_ids:
            do_switch(best, f"current master ineligible ({reason})")
            return best
        if best == current_master:
            return current_master
        held = t - self.master_since.get(current_master, t)
        if scores[best] > scores[current_master] + p.MASTER_SWITCH_MARGIN:
            if held >= p.MASTER_MIN_HOLD_TIME:
                do_switch(best, reason)
                return best
            # else: hysteresis suppresses the switch -- correct behavior, not a bug
        return current_master


# -------------------------------------------------------------- flocking ----

def flock_force(hw_id: int, states: dict[int, DroneState], adj, p: ElectionParams) -> np.ndarray:
    """swarm_sim_core.py::_flock_force, without the neighbor-influence-decay
    machinery (that needs real link-timeout state the centralized sim doesn't
    yet track per Phase 5 of the hardware roadmap) -- repulsion, cohesion,
    distance-dependent alignment, plus goal-seeking, over currently-reachable
    neighbors only.

    When p.lora_broadcast_interval > 0, uses each neighbor's last-RECEIVED
    state (set by update_neighbor_reception(), called once per tick by the
    caller before this) rather than live ground truth -- modeling Tomoto/LoRa's
    real broadcast rate per the safety question Appendix C Phase 2 raises and
    the companion proposal's Section 10.4 sweep answers empirically (safe
    through a 5s interval at this engine's default parameters, fails sharply
    past 5.5s). Defaults to the original always-live behavior when disabled."""
    a = states[hw_id]
    f_rep = np.zeros(2)
    f_coh = np.zeros(2)
    f_align = np.zeros(2)
    coh_count = 0

    for j in adj.get(hw_id, set()):
        if p.lora_broadcast_interval <= 0:
            b_pos, b_vel = states[j].pos, states[j].vel
        else:
            state = a.neighbor_last_state.get(j)
            if state is None:
                continue  # no reception yet from this neighbor
            b_pos, b_vel = state
        d = float(np.linalg.norm(a.pos - b_pos))
        if d < 1e-6:
            continue

        if d < p.D_rep:
            strength = p.repulsion_gain * (p.D_rep - d) / max(d, 0.5)
            f_rep += strength * (a.pos - b_pos) / d

        if d > p.cohesion_start:
            direction = (b_pos - a.pos) / d
            capped_extent = min(d, p.cohesion_max) - p.cohesion_start
            f_coh += capped_extent * direction
            coh_count += 1

        if d < p.alignment_max_dist:
            align_w = max(0.0, 1 - d / p.alignment_max_dist)
            f_align += align_w * (b_vel - a.vel)

    if coh_count > 0:
        f_coh = p.cohesion_gain * f_coh / coh_count

    to_goal = a.goal - a.pos
    dist_goal = float(np.linalg.norm(to_goal))
    f_goal = (to_goal / dist_goal) if dist_goal > 1e-6 else np.zeros(2)

    total = f_goal + f_rep + p.alignment_gain * f_align + f_coh

    mag = float(np.linalg.norm(total))
    if mag > p.max_accel:
        total = total * (p.max_accel / mag)
    return total
