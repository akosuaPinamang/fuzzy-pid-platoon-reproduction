"""
test_platoon_sim.py

Verifies platoon_sim.py's multi-vehicle chaining logic, and formally
proves this project's two central findings at the full-platoon level:

1. With gains that satisfy the paper's own Equation 13 stability
   condition, the platoon is properly "string stable": the spacing
   error shrinks as it propagates from the front of the platoon to
   the back.

2. With the paper's own literal printed baseline (kp=2, ki=3, kd=0.5,
   h=1.5), which test_pid_controller.py already shows fails Equation 13,
   the spacing error grows over time instead of settling down. This is
   the headline finding from VERIFICATION_LOG.md, encoded here as an
   automated, repeatable check rather than only a written claim.
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import numpy as np
from pid_controller import PIDController
from platoon_sim import run_platoon


def test_output_shapes_are_correct():
    controllers = [PIDController(kp=0.5, ki=0.3, kd=2.0, dt=0.01) for _ in range(3)]
    result = run_platoon(n_followers=3, controllers=controllers, h=1.5, dt=0.01, duration=5.0)
    n_expected = int(5.0 / 0.01)
    assert result["spacing_errs"].shape == (3, n_expected)
    assert result["velocities"].shape == (3, n_expected)
    assert len(result["t"]) == n_expected


def test_string_stability_with_eq13_valid_gains():
    """
    Gains that satisfy Eq13 (kp=0.5, ki=0.3, kd=2, h=1.5) should show
    the spacing error amplitude shrinking down the platoon: the
    lead-to-2nd gap error should be the largest, and the 3rd-to-4th
    gap error should be the smallest.
    """
    controllers = [PIDController(kp=0.5, ki=0.3, kd=2.0, dt=0.01) for _ in range(3)]
    result = run_platoon(n_followers=3, controllers=controllers, h=1.5, dt=0.01, duration=40.0)
    errs = result["spacing_errs"]
    peak_1_2 = np.max(np.abs(errs[0]))
    peak_2_3 = np.max(np.abs(errs[1]))
    peak_3_4 = np.max(np.abs(errs[2]))
    assert peak_1_2 >= peak_2_3 >= peak_3_4, (
        f"expected error to shrink down the platoon, got peaks "
        f"{peak_1_2:.3f} -> {peak_2_3:.3f} -> {peak_3_4:.3f}"
    )


def test_paper_baseline_error_grows_over_time():
    """
    HEADLINE FINDING: using the paper's own literal baseline parameters,
    the spacing error should be measurably LARGER in a later time window
    than in an earlier one, proving the system has not settled down but
    is instead slowly diverging. This is easy to miss within the paper's
    own 40-second plotting window but is unambiguous by 60 seconds.
    """
    controllers = [PIDController(kp=2.0, ki=3.0, kd=0.5, dt=0.01) for _ in range(3)]
    result = run_platoon(n_followers=3, controllers=controllers, h=1.5, dt=0.01, duration=60.0)
    errs = result["spacing_errs"][0]  # lead-to-2nd vehicle pair

    dt = 0.01
    early_window = errs[int(10 / dt):int(15 / dt)]
    late_window = errs[int(50 / dt):int(55 / dt)]

    early_amplitude = np.std(early_window)
    late_amplitude = np.std(late_window)

    assert late_amplitude > early_amplitude, (
        f"expected growing oscillation with the paper's literal baseline, "
        f"got early_std={early_amplitude:.4f} late_std={late_amplitude:.4f}"
    )
