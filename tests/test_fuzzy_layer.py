"""
test_fuzzy_layer.py

Verifies fuzzy_layer.py's rule table produces outputs in the expected
direction (Equation 23's gain-correction mechanism). The rule table is
now a direct, cell-by-cell transcription of the paper's own Table 2 (see
the module docstring in fuzzy_layer.py and VERIFICATION_LOG.md for the
transcription and cross-check against the classic literature table it is
based on). These tests check the sign/direction of the correction at the
table's extreme corners, which holds regardless of the exact rule table
used, so they remain valid as regression tests.
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from fuzzy_layer import FuzzyGainCorrector, FuzzyPIDController


def test_fuzzy_corrector_builds_without_error():
    fz = FuzzyGainCorrector(grid_size=11)  # small grid for a fast test
    assert fz is not None


def test_large_negative_error_increases_kp():
    fz = FuzzyGainCorrector(grid_size=11)
    dkp, dki, dkd = fz.compute(e_raw=-1.0, ec_raw=-1.0)
    assert dkp > 0, "a large negative error should push kp up (row NB/NB in RULES_DKP)"


def test_large_positive_error_decreases_kp():
    fz = FuzzyGainCorrector(grid_size=11)
    dkp, dki, dkd = fz.compute(e_raw=1.0, ec_raw=1.0)
    assert dkp < 0, "a large positive error should pull kp down (row PB/PB in RULES_DKP)"


def test_zero_error_gives_near_zero_correction():
    fz = FuzzyGainCorrector(grid_size=11)
    dkp, dki, dkd = fz.compute(e_raw=0.0, ec_raw=0.0)
    assert abs(dkp) < 0.5 and abs(dki) < 0.5


def test_fuzzy_pid_controller_runs_end_to_end():
    shared = FuzzyGainCorrector(grid_size=11)
    ctrl = FuzzyPIDController(kp_base=2.0, ki_base=3.0, kd_base=0.5, dt=0.01, shared_fuzzy=shared)
    u = ctrl.compute(delta=0.5, gap_rate=0.1)
    assert isinstance(u, float)
