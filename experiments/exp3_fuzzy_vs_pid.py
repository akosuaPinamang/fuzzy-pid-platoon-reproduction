"""
exp3_fuzzy_vs_pid.py

4-car platoon: plain PID vs Fuzzy-PID, no communication delay.
Target: Figure 6 in the paper (fuzzy-PID should show a tighter error band
than plain PID).
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from pid_controller import PIDController
from fuzzy_layer import FuzzyPIDController, FuzzyGainCorrector
from platoon_sim import run_platoon

if __name__ == "__main__":
    kp, ki, kd, h = 2.0, 3.0, 0.5, 1.5

    pid_controllers = [PIDController(kp=kp, ki=ki, kd=kd, dt=0.01) for _ in range(3)]
    pid_result = run_platoon(n_followers=3, controllers=pid_controllers, h=h, dt=0.01, duration=40.0)

    # The fuzzy output magnitude (out_scales) controls whether the layer
    # has a visible effect. With these values the Fuzzy-PID error band is
    # about 10 percent tighter than plain PID, which matches the paper's
    # direction but not its reported 32 percent. The gap is due to the
    # paper's inconsistent fuzzy scaling factors; see VERIFICATION_LOG.md.
    shared_fuzzy = FuzzyGainCorrector(e_scale=1.0, ec_scale=0.05, out_scales=(0.1, 0.05, 0.05))
    fuzzy_controllers = [
        FuzzyPIDController(kp_base=kp, ki_base=ki, kd_base=kd, dt=0.01,
                            shared_fuzzy=shared_fuzzy)
        for _ in range(3)
    ]
    fuzzy_result = run_platoon(n_followers=3, controllers=fuzzy_controllers, h=h, dt=0.01, duration=40.0)

    t = pid_result["t"]
    pid_errs = pid_result["spacing_errs"]
    fuzzy_errs = fuzzy_result["spacing_errs"]

    print("Lead-2nd vehicle spacing error range:")
    print(f"  Plain PID:  [{pid_errs[0].min():.3f}, {pid_errs[0].max():.3f}] m   (paper: -0.6 to 0.6 m)")
    print(f"  Fuzzy-PID:  [{fuzzy_errs[0].min():.3f}, {fuzzy_errs[0].max():.3f}] m   (paper: -0.23 to 0.58 m)")

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    axes[0].plot(t, pid_errs[0])
    axes[0].set_title("Plain PID: Lead-2nd spacing error")
    axes[0].set_xlabel("Time (s)"); axes[0].set_ylabel("Spacing error (m)")
    axes[0].grid(alpha=0.3)

    axes[1].plot(t, fuzzy_errs[0], color="darkorange")
    axes[1].set_title("Fuzzy-PID: Lead-2nd spacing error")
    axes[1].set_xlabel("Time (s)"); axes[1].set_ylabel("Spacing error (m)")
    axes[1].grid(alpha=0.3)

    plt.suptitle("Figure 6 comparison target: Fuzzy-PID should show a tighter error band")
    plt.tight_layout()
    out_path = os.path.join(os.path.dirname(__file__), "..", "plots", "exp3_fuzzy_vs_pid.png")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    plt.savefig(out_path, dpi=130)
    print(f"Saved plot to {out_path}")
