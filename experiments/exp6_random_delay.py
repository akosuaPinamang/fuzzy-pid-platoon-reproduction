"""
exp6_random_delay.py

4-car platoon under randomly varying communication delay (Equation 27:
Td = 0.5s + Uniform(-0.5, 0.5)). Target: Figures 9 and 10 in the paper.
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from pid_controller import PIDController
from fuzzy_layer import FuzzyPIDController, FuzzyGainCorrector
from platoon_sim import run_platoon
from delay import random_delay

if __name__ == "__main__":
    kp, ki, kd, h = 2.0, 3.0, 0.5, 1.5
    dt = 0.01
    delay_fn = random_delay(0.5, dt, seed=42)

    pid_ctrls = [PIDController(kp=kp, ki=ki, kd=kd, dt=dt) for _ in range(3)]
    pid_res = run_platoon(n_followers=3, controllers=pid_ctrls, h=h, dt=dt,
                            duration=40.0, delay_fn=delay_fn)

    # Scaling updated to match exp3's finding: out_scales=(0.1,0.05,0.05)
    # with e_scale=1.0, ec_scale=0.05 is the config that showed a
    # measurably tighter Fuzzy-PID band than plain PID (see
    # VERIFICATION_LOG.md Section 8, Exp3).
    shared_fuzzy = FuzzyGainCorrector(e_scale=1.0, ec_scale=0.05, out_scales=(0.1, 0.05, 0.05))
    fuzzy_ctrls = [FuzzyPIDController(kp_base=kp, ki_base=ki, kd_base=kd, dt=dt,
                                        shared_fuzzy=shared_fuzzy) for _ in range(3)]
    fuzzy_res = run_platoon(n_followers=3, controllers=fuzzy_ctrls, h=h, dt=dt,
                              duration=40.0, delay_fn=delay_fn)

    t = pid_res["t"]
    e_pid = pid_res["spacing_errs"][0]
    e_fuzzy = fuzzy_res["spacing_errs"][0]

    print("Lead-2nd spacing error with random communication delay (mean 0.5s):")
    print(f"  Plain PID:  [{e_pid.min():.3f}, {e_pid.max():.3f}] m")
    print(f"  Fuzzy-PID:  [{e_fuzzy.min():.3f}, {e_fuzzy.max():.3f}] m")

    plt.figure(figsize=(7, 4))
    plt.plot(t, e_pid, label="Plain PID")
    plt.plot(t, e_fuzzy, label="Fuzzy-PID")
    plt.title("Lead-2nd spacing error, random delay (target: Figures 9-10)")
    plt.xlabel("Time (s)"); plt.ylabel("Spacing error (m)")
    plt.legend(); plt.grid(alpha=0.3)
    plt.tight_layout()
    out_path = os.path.join(os.path.dirname(__file__), "..", "plots", "exp6_random_delay.png")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    plt.savefig(out_path, dpi=130)
    print(f"Saved plot to {out_path}")
