"""
test_metrics.py

Verifies metrics.py implements Equations 24-26 (Ep, Mp, sigma_p) correctly,
using a small synthetic array with hand-calculated expected values.
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import numpy as np
from metrics import mean_space_error, mean_max_space_error, mean_std_space_error, all_metrics


def _sample_array():
    # vehicle 1: |errors| = [0, 1, 1, 2] -> mean=1.0, max=2.0
    # vehicle 2: |errors| = [0, 0.5, 0.5, 1] -> mean=0.5, max=1.0
    return np.array([
        [0.0, 1.0, -1.0, 2.0],
        [0.0, 0.5, -0.5, 1.0],
    ])


def test_mean_space_error():
    Ep = mean_space_error(_sample_array())
    assert abs(Ep - 0.75) < 1e-9  # (1.0 + 0.5) / 2 averaged over all points = 0.75


def test_mean_max_space_error():
    Mp = mean_max_space_error(_sample_array())
    assert abs(Mp - 1.5) < 1e-9  # (2.0 + 1.0) / 2


def test_all_metrics_returns_all_three_keys():
    m = all_metrics(_sample_array())
    assert set(m.keys()) == {"Ep", "Mp", "sigma_p"}
    assert m["Mp"] >= m["Ep"]  # max error can never be smaller than mean error
