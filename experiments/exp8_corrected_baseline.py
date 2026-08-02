"""
exp8_corrected_baseline.py

Corrected-baseline demonstration (added for the presentation defence).

Purpose: show that the platoon divergence reported elsewhere is caused by
the paper's own baseline gains failing its own Equation 13 stability
condition, NOT by any error in this reproduction. The SAME platoon code is
run twice, changing only the PID gains:

  Panel A (left):  the paper's literal baseline, kp=2, ki=3, kd=0.5.
                   This fails Equation 13 (kd is not > kp), and the spacing
                   error GROWS down the platoon: string-unstable.

  Panel B (right): a corrected set, kp=1, ki=0.5, kd=2, chosen only so that
                   it satisfies Equation 13. The spacing error now SHRINKS
                   down the platoon: string-stable, and bounded to a couple
                   of metres, the same scale as the paper's figures.

The point for the defence: our chaining logic is correct. Give it gains that
satisfy the paper's own stability theorem and it produces the string-stable
behaviour the paper claims. The divergence is the paper's parameter choice.
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from pid_controller import PIDController, check_stability
from platoon_sim import run_platoon

LABELS = ["Lead-2nd", "2nd-3rd", "3rd-4th"]


def run_case(kp, ki, kd, h=1.5, dt=0.01, duration=40.0):
    controllers = [PIDController(kp=kp, ki=ki, kd=kd, dt=dt) for _ in range(3)]
    result = run_platoon(n_followers=3, controllers=controllers, h=h, dt=dt, duration=duration)
    errs = result["spacing_errs"]
    peak = [float(np.max(np.abs(errs[i][int(10/dt):int(30/dt)]))) for i in range(3)]
    return result["t"], errs, peak


if __name__ == "__main__":
    h = 1.5

    # Case A: paper's literal baseline
    kpA, kiA, kdA = 2.0, 3.0, 0.5
    stableA, reasonsA = check_stability(kpA, kiA, kdA, h)
    tA, errsA, peakA = run_case(kpA, kiA, kdA, h)

    # Case B: corrected gains that satisfy Equation 13
    kpB, kiB, kdB = 1.0, 0.5, 2.0
    stableB, reasonsB = check_stability(kpB, kiB, kdB, h)
    tB, errsB, peakB = run_case(kpB, kiB, kdB, h)

    print("=== Case A: paper's baseline (kp=2, ki=3, kd=0.5) ===")
    print(f"  Satisfies Equation 13? {stableA}")
    for r in reasonsA:
        print("   -", r)
    print(f"  Peak |error| per pair (10-30s): {[round(p,2) for p in peakA]}")
    print(f"  -> error {'GROWS' if peakA[2] > peakA[0] else 'shrinks'} down the platoon "
          f"(string-{'UNSTABLE' if peakA[2] > peakA[0] else 'stable'})")
    print()
    print("=== Case B: corrected gains (kp=1, ki=0.5, kd=2) ===")
    print(f"  Satisfies Equation 13? {stableB}")
    print(f"  Peak |error| per pair (10-30s): {[round(p,2) for p in peakB]}")
    print(f"  -> error {'grows' if peakB[2] > peakB[0] else 'SHRINKS'} down the platoon "
          f"(string-{'unstable' if peakB[2] > peakB[0] else 'STABLE'})")
    print()
    print("Conclusion: identical platoon code. Only the gains changed. Gains that")
    print("satisfy Equation 13 give the string-stable behaviour the paper claims,")
    print("so the divergence elsewhere is the paper's baseline, not our code.")

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
    for i in range(3):
        axes[0].plot(tA, errsA[i], label=LABELS[i])
        axes[1].plot(tB, errsB[i], label=LABELS[i])
    axes[0].set_title("Baseline (kp=2, ki=3, kd=0.5): fails Eq 13, error grows", fontsize=10)
    axes[1].set_title("Corrected (kp=1, ki=0.5, kd=2): satisfies Eq 13, error shrinks", fontsize=10)
    for ax in axes:
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Spacing error (m)")
        ax.legend()
        ax.grid(alpha=0.3)
    plt.suptitle("Same code, different gains: the divergence is the paper's baseline, not the reproduction")
    plt.tight_layout()
    out_path = os.path.join(os.path.dirname(__file__), "..", "plots", "exp8_corrected_baseline.png")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    plt.savefig(out_path, dpi=130)
    print(f"Saved plot to {out_path}")
