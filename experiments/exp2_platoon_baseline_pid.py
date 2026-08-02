"""
exp2_platoon_baseline_pid.py

4-car platoon (1 lead + 3 followers), plain PID, no communication delay.
Reproduces the PID curves in Figure 6a-c.
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from pid_controller import PIDController
from platoon_sim import run_platoon

if __name__ == "__main__":
    kp, ki, kd, h = 2.0, 3.0, 0.5, 1.5
    controllers = [PIDController(kp=kp, ki=ki, kd=kd, dt=0.01) for _ in range(3)]
    result = run_platoon(n_followers=3, controllers=controllers, h=h, dt=0.01, duration=40.0)

    t = result["t"]
    errs = result["spacing_errs"]

    print("Spacing error range per vehicle pair (plain PID, no delay):")
    labels = ["Lead-2nd", "2nd-3rd", "3rd-4th"]
    for i in range(3):
        print(f"  {labels[i]}: [{errs[i].min():.3f}, {errs[i].max():.3f}] m")

    # String stability check: does the error amplitude shrink going down the platoon?
    amp = [np.max(np.abs(errs[i][int(10/0.01):int(30/0.01)])) for i in range(3)]
    print(f"Peak |error| within 10-30s window per vehicle pair: {[f'{a:.3f}' for a in amp]}")
    print("(String stability expects this to not grow down the platoon.)")

    plt.figure(figsize=(7, 4))
    for i in range(3):
        plt.plot(t, errs[i], label=labels[i])
    plt.title("4-car platoon, plain PID, no delay (target: resembles Figure 6a PID curves)")
    plt.xlabel("Time (s)")
    plt.ylabel("Spacing error (m)")
    plt.legend()
    plt.grid(alpha=0.3)
    plt.tight_layout()
    out_path = os.path.join(os.path.dirname(__file__), "..", "plots", "exp2_platoon_baseline_pid.png")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    plt.savefig(out_path, dpi=130)
    print(f"Saved plot to {out_path}")
