"""
pid_controller.py

PID control law (Equation 16) and the Routh-Hurwitz stability check
(Equation 13). check_stability() reports whether a set of gains satisfies
the paper's own stability condition; the paper's baseline (kp=2, ki=3,
kd=0.5) fails it. Full derivation, the sign-convention choice, and the
tau-factor finding are documented in VERIFICATION_LOG.md.
"""


def check_stability(kp, ki, kd, h):
    """Return (is_stable: bool, reasons: list[str])."""
    reasons = []
    if not (kp > 0):
        reasons.append(f"kp must be > 0 (got {kp})")
    if not (kd > kp):
        reasons.append(f"kd must be > kp (got kd={kd}, kp={kp})")
    upper_bound = kp * (kd - kp) * (1 + h)
    if not (0 < ki < upper_bound):
        reasons.append(f"ki must satisfy 0 < ki < {upper_bound:.4f} (got ki={ki})")
    return (len(reasons) == 0), reasons


class PIDController:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0.0
        self.prev_gap_rate_input = 0.0  # for derivative-on-error-rate term

    def reset(self):
        self.integral = 0.0

    def compute(self, delta, gap_rate):
        """
        delta    : spacing error (positive means the gap is too large, so
                   the controller must accelerate to close it).
        gap_rate : rate of change of the gap (the derivative term).
        Gains can be swapped each call by the fuzzy layer (Equation 23).
        Positive-form sign convention; see VERIFICATION_LOG.md.
        """
        self.integral += delta * self.dt
        u = (self.kd * gap_rate + self.kp * delta + self.ki * self.integral)
        return u


if __name__ == "__main__":
    # Checkpoint: paper's own baseline parameters must pass their own
    # stability test (kp=2, ki=3, kd=0.5, h=1.5)
    kp, ki, kd, h = 2.0, 3.0, 0.5, 1.5
    stable, reasons = check_stability(kp, ki, kd, h)
    print(f"Baseline (kp={kp}, ki={ki}, kd={kd}, h={h}) stable? {stable}")
    for r in reasons:
        print("  -", r)
    # kd=0.5 is not > kp=2, so the paper's baseline fails its own Eq 13.
    # A genuine reproduction finding; see VERIFICATION_LOG.md.
