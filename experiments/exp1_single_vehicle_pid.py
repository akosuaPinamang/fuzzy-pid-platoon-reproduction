"""
exp1_single_vehicle_pid.py

Single follower vehicle tracking the lead vehicle's speed profile with
plain PID control, no communication delay. Reproduces Figure 5.

Lead vehicle speed profile (Equation 21, corrected form, see project
document Section 2.4 for why the corrected ramp formula is used):
    v0(t) = 0            for t <= 10s
    v0(t) = 2*(t-10)     for 10 < t <= 20s
    v0(t) = 20           for t > 20s
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from vehicle_model import Vehicle
from spacing import spacing_error
from pid_controller import PIDController, check_stability


def lead_speed(t):
    if t <= 10.0:
        return 0.0
    elif t <= 20.0:
        return 2.0 * (t - 10.0)
    else:
        return 20.0


def run(kp=2.0, ki=3.0, kd=0.5, h=1.5, tau=0.5, dt=0.01, duration=40.0):
    n = int(duration / dt)
    t_arr = np.arange(n) * dt

    lead_v = np.array([lead_speed(t) for t in t_arr])
    lead_pos = np.cumsum(lead_v) * dt  # integrate lead position

    follower = Vehicle(tau=tau, dt=dt, length=4.5)
    # Initialize the follower so the initial gap equals the initial desired
    # spacing (zero, since v=0 at t=0). Without this, both vehicles start
    # at the same coordinate and the "gap" formula reads a nonsensical
    # negative overlap equal to minus the vehicle length. The paper states
    # "the initial location and velocity are together equivalent to zero",
    # which we interpret as zero initial error, not zero raw position.
    follower.position = -follower.length
    pid = PIDController(kp=kp, ki=ki, kd=kd, dt=dt)

    spacing_errs = np.zeros(n)
    velocities = np.zeros(n)
    prev_gap = None

    for k in range(n):
        gap = lead_pos[k] - follower.position - follower.length
        gap_rate = 0.0 if prev_gap is None else (gap - prev_gap) / dt
        prev_gap = gap

        delta, _, d_des = spacing_error(
            pos_front=lead_pos[k], pos_rear=follower.position,
            length_front=follower.length, h=h, velocity_rear=follower.velocity
        )
        u = pid.compute(delta, gap_rate)
        follower.step(u)

        spacing_errs[k] = delta
        velocities[k] = follower.velocity

    return t_arr, spacing_errs, velocities, lead_v


if __name__ == "__main__":
    stable, reasons = check_stability(2.0, 3.0, 0.5, 1.5)
    print(f"Stability check on paper's baseline: stable={stable}")
    for r in reasons:
        print("  -", r)
    print("Proceeding to simulate anyway with the paper's literal baseline,")
    print("since the real test is whether the simulated behaviour is bounded")
    print("and matches Figure 5, not whether it satisfies Eq 13 on paper.\n")

    t, err, vel, lead_v = run()

    print(f"Spacing error range: [{err.min():.3f}, {err.max():.3f}] m "
          f"(paper's Figure 5a shows roughly [-0.6, 0.6] m)")
    print(f"Final velocity: {vel[-1]:.3f} m/s (target 20 m/s)")
    print(f"Time to reach 19.9 m/s: "
          f"{t[np.argmax(vel >= 19.9)]:.2f}s" if np.any(vel >= 19.9) else "never reached 19.9 m/s")

    fig, axes = plt.subplots(2, 1, figsize=(7, 7))
    axes[0].plot(t, err)
    axes[0].set_title("Spacing error (target: resembles Figure 5a)")
    axes[0].set_xlabel("Time (s)")
    axes[0].set_ylabel("Spacing error (m)")
    axes[0].axhline(0, color="grey", linewidth=0.5)
    axes[0].grid(alpha=0.3)

    axes[1].plot(t, lead_v, "--", label="Reference (lead) velocity")
    axes[1].plot(t, vel, label="PID controlled velocity")
    axes[1].set_title("Velocity tracking (target: resembles Figure 5b)")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Velocity (m/s)")
    axes[1].legend()
    axes[1].grid(alpha=0.3)

    plt.tight_layout()
    out_path = os.path.join(os.path.dirname(__file__), "..", "plots", "exp1_single_vehicle_pid.png")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    plt.savefig(out_path, dpi=130)
    print(f"\nSaved plot to {out_path}")
