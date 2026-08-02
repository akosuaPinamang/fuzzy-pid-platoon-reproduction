"""
exp7_heterogeneous_platoon.py

4-car platoon (1 lead + 3 followers) where each follower has a different
lag value (Table 3: tau2=0.5, tau3=0.8, tau4=1.0), random delay
Td ~ Tc(0.5s) + noise, and Gaussian noise added to the lead vehicle's
speed signal. Compares Ep/Mp/sigma_p per vehicle pair and platoon-wide
against Table 4 in the paper:
    Vehicle 1-2: sigma_p=0.1284, Ep=0.1458, Mp=0.7332
    Vehicle 2-3: sigma_p=0.1334, Ep=0.1149, Mp=0.5314
    Vehicle 3-4: sigma_p=0.1303, Ep=0.1115, Mp=0.5225
    Platoon:     sigma_p=0.1367, Ep=0.1307, Mp=0.5957
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from pid_controller import PIDController
from platoon_sim import run_platoon
from delay import random_delay
from metrics import all_metrics

if __name__ == "__main__":
    kp, ki, kd, h = 2.0, 3.0, 0.5, 1.5
    dt = 0.01
    taus = [0.5, 0.8, 1.0]  # Table 3

    delay_fn = random_delay(0.0, dt, noise_low=-1.0, noise_high=1.0, seed=7)
    # NOTE: Td ~ N(0,1)s per Table 3 in the paper (mean 0, spread 1s) is
    # itself another instance of the paper's ambiguous "N(...)" notation
    # (Section 2.3's note applies here too). We use a zero-mean uniform
    # spread of +/-1s as the most consistent reading available.

    controllers = [PIDController(kp=kp, ki=ki, kd=kd, dt=dt) for _ in range(3)]
    result = run_platoon(n_followers=3, controllers=controllers, h=h, taus=taus,
                          dt=dt, duration=60.0, delay_fn=delay_fn, speed_noise_std=1.0, seed=7)

    errs = result["spacing_errs"]
    labels = ["Vehicle 1-2", "Vehicle 2-3", "Vehicle 3-4"]
    paper_table4 = {
        "Vehicle 1-2": {"sigma_p": 0.1284, "Ep": 0.1458, "Mp": 0.7332},
        "Vehicle 2-3": {"sigma_p": 0.1334, "Ep": 0.1149, "Mp": 0.5314},
        "Vehicle 3-4": {"sigma_p": 0.1303, "Ep": 0.1115, "Mp": 0.5225},
    }

    print("Heterogeneous platoon results vs Table 4:")
    print(f"{'Pair':<14}{'Ours Ep':>10}{'Paper Ep':>10}{'Ours Mp':>10}{'Paper Mp':>10}{'Ours sig':>10}{'Paper sig':>10}")
    for i, label in enumerate(labels):
        m = all_metrics(errs[i:i+1])
        p = paper_table4[label]
        print(f"{label:<14}{m['Ep']:>10.4f}{p['Ep']:>10.4f}{m['Mp']:>10.4f}{p['Mp']:>10.4f}"
              f"{m['sigma_p']:>10.4f}{p['sigma_p']:>10.4f}")

    m_all = all_metrics(errs)
    print(f"\nPlatoon-wide: Ep={m_all['Ep']:.4f} (paper 0.1307), "
          f"Mp={m_all['Mp']:.4f} (paper 0.5957), sigma_p={m_all['sigma_p']:.4f} (paper 0.1367)")

    fig, axes = plt.subplots(1, 3, figsize=(6, 4))
    fig.delaxes(axes[1]); fig.delaxes(axes[2])
    t = result["t"]
    axes[0].plot(t, errs[0], label="1-2")
    axes[0].plot(t, errs[1], label="2-3")
    axes[0].plot(t, errs[2], label="3-4")
    axes[0].set_title("Heterogeneous platoon spacing error")
    axes[0].set_xlabel("Time (s)"); axes[0].set_ylabel("Spacing error (m)")
    axes[0].legend(); axes[0].grid(alpha=0.3)
    plt.tight_layout()
    out_path = os.path.join(os.path.dirname(__file__), "..", "plots", "exp7_heterogeneous_platoon.png")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    plt.savefig(out_path, dpi=130)
    print(f"Saved plot to {out_path}")
