"""
test_vehicle_model.py

Verifies vehicle_model.Vehicle follows the first-order lag behaviour
described by Equation 6: tau*a_dot + a = u.

A first-order lag system, given a constant step input, is expected to:
  - reach about 63% of the final value after 1 tau has passed
  - reach about 99% of the final value after 5 tau have passed
This is a textbook control theory result, independent of this paper,
used here purely to confirm the vehicle model itself is implemented
correctly before anything else is built on top of it.
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import numpy as np
from vehicle_model import Vehicle


def test_first_order_lag_reaches_63_percent_after_one_tau():
    tau, dt = 0.5, 0.01
    veh = Vehicle(tau=tau, dt=dt)
    n_steps = int(tau / dt)
    for _ in range(n_steps):
        _, _, a = veh.step(1.0)
    assert 0.55 < a < 0.70, f"expected ~0.632 after 1 tau, got {a}"


def test_first_order_lag_reaches_99_percent_after_five_tau():
    tau, dt = 0.5, 0.01
    veh = Vehicle(tau=tau, dt=dt)
    n_steps = int(5 * tau / dt)
    for _ in range(n_steps):
        _, _, a = veh.step(1.0)
    assert a > 0.98, f"expected ~0.993 after 5 tau, got {a}"


def test_zero_input_keeps_vehicle_at_rest():
    veh = Vehicle(tau=0.5, dt=0.01)
    for _ in range(100):
        pos, vel, acc = veh.step(0.0)
    assert pos == 0.0 and vel == 0.0 and acc == 0.0
