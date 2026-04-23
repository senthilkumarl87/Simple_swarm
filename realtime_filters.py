"""
Optional smoothers for the real-time path.

Kept deliberately minimal — just an α-β tracker. For the offline planner and
for Option C on loopback UDP there's nothing to filter (clean CSV, no
packet loss, sub-ms jitter), so nothing here is imported by default. Option
C's follower enables this only when `--smooth` is passed.
"""

from dataclasses import dataclass


@dataclass
class AlphaBetaFilter:
    """
    Constant-velocity α-β filter for a single scalar axis.

    Model: position + velocity; prediction assumes constant velocity between
    measurements. On each update the innovation (measurement − prediction)
    is split between position (α) and velocity (β/dt) corrections.

    Typical gains:
        α = 0.5, β = 0.1   — gentle smoothing, small lag (default)
        α = 0.8, β = 0.3   — tighter tracking, more responsive
        α = 0.3, β = 0.05  — heavy smoothing, more lag

    Stability requires α ∈ (0, 1) and 0 < β < 2·(2 − α) − 4·√(1 − α), but
    the math stays well-behaved across any reasonable pair of small
    positive numbers.
    """
    alpha: float = 0.5
    beta: float = 0.1

    def __post_init__(self):
        self.x = None          # position estimate
        self.v = 0.0           # velocity estimate

    @property
    def initialized(self) -> bool:
        return self.x is not None

    def reset(self, x0: float, v0: float = 0.0) -> None:
        self.x = float(x0)
        self.v = float(v0)

    def update(self, z: float, dt: float) -> tuple:
        """Predict forward by dt, then correct with observation z.

        Returns (x, v). dt must be > 0; a dt ≤ 0 is treated as a no-op
        correction (useful when reordered packets arrive with stale
        timestamps — caller should typically reject those upstream).
        """
        if self.x is None:
            self.reset(z)
            return self.x, self.v
        if dt <= 0:
            return self.x, self.v

        x_pred = self.x + self.v * dt
        v_pred = self.v
        residual = z - x_pred
        self.x = x_pred + self.alpha * residual
        self.v = v_pred + (self.beta / dt) * residual
        return self.x, self.v

    def predict(self, dt: float) -> tuple:
        """Pure prediction (no correction). Use between measurements."""
        if self.x is None:
            return 0.0, self.v
        if dt <= 0:
            return self.x, self.v
        return self.x + self.v * dt, self.v
