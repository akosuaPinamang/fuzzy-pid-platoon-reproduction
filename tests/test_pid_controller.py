"""
test_pid_controller.py

Verifies pid_controller.py implements Equation 16 (PID law) and
Equation 13 (Routh-Hurwitz stability conditions) correctly.

The second test here formally proves this project's headline finding:
the paper's own printed baseline parameters (kp=2, ki=3, kd=0.5, h=1.5)
do not satisfy the paper's own derived stability theorem. This is
documented at length in VERIFICATION_LOG.md; this test exists so that
finding is backed by an automated, repeatable check rather than only a
written claim.
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from pid_controller import PIDController, check_stability


def test_valid_gains_pass_stability_check():
    # kp=0.5, ki=0.3, kd=2, h=1.5 satisfies all three Eq13 conditions:
    # kp>0, kd>kp, and 0 < ki < kp*(kd-kp)*(1+h) = 0.5*1.5*2.5 = 1.875
    stable, reasons = check_stability(kp=0.5, ki=0.3, kd=2.0, h=1.5)
    assert stable is True
    assert reasons == []


def test_paper_baseline_fails_its_own_stability_condition():
    """
    HEADLINE FINDING: the paper's own baseline (kp=2, ki=3, kd=0.5, h=1.5)
    fails Equation 13's requirement that kd > kp. See VERIFICATION_LOG.md
    for the full derivation and three independent confirmations of this.
    """
    stable, reasons = check_stability(kp=2.0, ki=3.0, kd=0.5, h=1.5)
    assert stable is False
    assert any("kd must be > kp" in r for r in reasons)


def test_kp_must_be_positive():
    stable, reasons = check_stability(kp=-1.0, ki=1.0, kd=2.0, h=1.5)
    assert stable is False
    assert any("kp must be > 0" in r for r in reasons)


def test_pid_accelerates_when_trailing_too_far_behind():
    """
    A positive spacing error means the actual gap is larger than desired
    (the follower is trailing too far behind). A correctly implemented
    controller must respond with a POSITIVE control signal (accelerate)
    to close the gap, not a negative one. See pid_controller.py's
    module-level note on the sign-convention fix found during
    development for the full explanation.
    """
    pid = PIDController(kp=2.0, ki=3.0, kd=0.5, dt=0.01)
    u = pid.compute(delta=1.0, gap_rate=0.0)
    assert u > 0, "controller should accelerate (positive u) when trailing too far behind"


def test_pid_decelerates_when_too_close():
    pid = PIDController(kp=2.0, ki=3.0, kd=0.5, dt=0.01)
    u = pid.compute(delta=-1.0, gap_rate=0.0)
    assert u < 0, "controller should decelerate (negative u) when following too closely"
