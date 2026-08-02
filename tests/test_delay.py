"""
test_delay.py

Verifies delay.py's constant and random delay models (Equation 27).
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import numpy as np
from delay import constant_delay, random_delay


def test_constant_delay_returns_fixed_step_count():
    dt = 0.01
    fn = constant_delay(0.5, dt)
    steps = [fn(k, dt) for k in range(50)]
    assert all(s == 50 for s in steps)


def test_random_delay_averages_to_expected_mean():
    dt = 0.01
    fn = random_delay(0.5, dt, noise_low=-0.5, noise_high=0.5, seed=1)
    samples = [fn(k, dt) for k in range(3000)]
    mean_steps = np.mean(samples)
    # Tc=0.5s -> 50 steps, symmetric noise around 0 -> mean should stay near 50
    assert 45 <= mean_steps <= 55


def test_random_delay_never_negative():
    dt = 0.01
    fn = random_delay(0.0, dt, noise_low=-1.0, noise_high=1.0, seed=2)
    samples = [fn(k, dt) for k in range(500)]
    assert all(s >= 0 for s in samples)
