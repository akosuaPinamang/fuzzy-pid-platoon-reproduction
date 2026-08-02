"""
exp4_tau_sweep.py

Sweep the vehicle lag constant tau from 0.1 to 1.0 (paper's Figure 7),
compute Ep/Mp/sigma_p for both plain PID and Fuzzy-PID at each value, on
a single follower vehicle (matches the paper's single-vehicle tau study).
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
from metrics import all_metrics

if __name__ == "__main__":
    kp, ki, kd, h = 2.0, 3.0, 0.5, 1.5
    dt = 0.01
    taus = np.arange(0.1, 1.01, 0.1)

    # Scaling updated to match exp3's finding: out_scales=(0.1,0.05,0.05)
    # with e_scale=1.0, ec_scale=0.05 is the config that showed a
    # measurably tighter Fuzzy-PID band than plain PID (see
    # VERIFICATION_LOG.md Section 8, Exp3).
    shared_fuzzy = FuzzyGainCorrector(e_scale=1.0, ec_scale=0.05, out_scales=(0.1, 0.05, 0.05))

    pid_metrics = {"Ep": [], "Mp": [], "sigma_p": []}
    fuzzy_metrics = {"Ep": [], "Mp": [], "sigma_p": []}

    for tau in taus:
        pid_ctrl = [PIDController(kp=kp, ki=ki, kd=kd, dt=dt)]
        pid_res = run_platoon(n_followers=1, controllers=pid_ctrl, h=h, taus=[tau], dt=dt, duration=40.0)
        m = all_metrics(pid_res["spacing_errs"])
        for key in pid_metrics: pid_metrics[key].append(m[key])

        fuzzy_ctrl = [FuzzyPIDController(kp_base=kp, ki_base=ki, kd_base=kd, dt=dt, shared_fuzzy=shared_fuzzy)]
        fuzzy_res = run_platoon(n_followers=1, controllers=fuzzy_ctrl, h=h, taus=[tau], dt=dt, duration=40.0)
        m = all_metrics(fuzzy_res["spacing_errs"])
        for key in fuzzy_metrics: fuzzy_metrics[key].append(m[key])

    print("tau sweep results (single vehicle):")
    print(f"{'tau':>5} {'PID Ep':>8} {'Fz Ep':>8} {'PID Mp':>8} {'Fz Mp':>8}")
    for i, tau in enumerate(taus):
        print(f"{tau:5.1f} {pid_metrics['Ep'][i]:8.4f} {fuzzy_metrics['Ep'][i]:8.4f} "
              f"{pid_metrics['Mp'][i]:8.4f} {fuzzy_metrics['Mp'][i]:8.4f}")

    fig, axes = plt.subplots(1, 3, figsize=(13, 4))
    for ax, key, title in zip(axes, ["Mp", "Ep", "sigma_p"],
                                ["Max space error", "Mean space error", "Mean std dev"]):
        ax.plot(taus, pid_metrics[key], "o-", label="PID")
        ax.plot(taus, fuzzy_metrics[key], "s-", label="Fuzzy-PID")
        ax.set_title(title); ax.set_xlabel("tau (s)"); ax.legend(); ax.grid(alpha=0.3)
    plt.suptitle("Tau sweep (target: resembles Figure 7)")
    plt.tight_layout()
    out_path = os.path.join(os.path.dirname(__file__), "..", "plots", "exp4_tau_sweep.png")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    plt.savefig(out_path, dpi=130)
    print(f"Saved plot to {out_path}")
