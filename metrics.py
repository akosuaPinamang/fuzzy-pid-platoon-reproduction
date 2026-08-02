"""
metrics.py

Performance metrics, Equations 24-26 in the paper.

Ep(T)    : mean spacing error across all vehicles and all time steps
Mp(T)    : mean of each vehicle's own maximum absolute spacing error
sigma_p(T): mean standard deviation of spacing error, per vehicle,
            averaged across vehicles

Input `spacing_errs` is expected to be a 2D array shaped
(n_vehicles, n_timesteps), matching what platoon_sim.run_platoon returns.
"""
import numpy as np


def mean_space_error(spacing_errs):
    """Equation 24: Ep(T)"""
    return np.mean(np.abs(spacing_errs))


def mean_max_space_error(spacing_errs):
    """Equation 25: Mp(T)"""
    per_vehicle_max = np.max(np.abs(spacing_errs), axis=1)
    return np.mean(per_vehicle_max)


def mean_std_space_error(spacing_errs):
    """Equation 26: sigma_p(T)"""
    per_vehicle_std = np.std(np.abs(spacing_errs), axis=1)
    return np.mean(per_vehicle_std)


def all_metrics(spacing_errs):
    return {
        "Ep": mean_space_error(spacing_errs),
        "Mp": mean_max_space_error(spacing_errs),
        "sigma_p": mean_std_space_error(spacing_errs),
    }


if __name__ == "__main__":
    # Checkpoint: sanity check on a synthetic known array
    test = np.array([
        [0.0, 1.0, -1.0, 2.0],   # vehicle 1: mean|.|=1.0, max|.|=2.0
        [0.0, 0.5, -0.5, 1.0],   # vehicle 2: mean|.|=0.5, max|.|=1.0
    ])
    m = all_metrics(test)
    print(m)
    assert abs(m["Mp"] - 1.5) < 1e-9   # (2.0+1.0)/2
    print("metrics.py checkpoint passed")
