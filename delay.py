"""
delay.py

Communication delay models passed as delay_fn to platoon_sim.run_platoon:
  - constant_delay(seconds) : fixed delay every step
  - random_delay(...)       : Equation 27, Td = Tc + n(t)

Equation 27's noise n(t) is read as Uniform(-0.5, 0.5): the paper calls it
"uniformly distributed" but writes it with Gaussian-looking notation. The
separate Gaussian velocity noise for the heterogeneous platoon is handled
in platoon_sim. Full reasoning is in VERIFICATION_LOG.md.
"""
import numpy as np


def constant_delay(seconds, dt):
    steps = int(round(seconds / dt))
    def fn(k, dt_):
        return steps
    return fn


def random_delay(tc_seconds, dt, noise_low=-0.5, noise_high=0.5, seed=0, sample_period=0.1):
    """
    Equation 27: Td = Tc + n(t), n(t) ~ Uniform(noise_low, noise_high).
    sample_period : draws a new n(t) every 0.1s and holds it (zero-order
        hold), matching Table 3's Ts=0.1s rather than resampling every step.
    """
    rng = np.random.default_rng(seed)
    steps_per_sample = max(1, int(round(sample_period / dt)))
    state = {"k_last_sample": None, "steps": None}

    def fn(k, dt_):
        sample_idx = k // steps_per_sample
        if state["k_last_sample"] != sample_idx:
            td = tc_seconds + rng.uniform(noise_low, noise_high)
            state["steps"] = max(0, int(round(td / dt_)))
            state["k_last_sample"] = sample_idx
        return state["steps"]
    return fn


if __name__ == "__main__":
    dt = 0.01
    cd = constant_delay(0.5, dt)
    print(f"constant_delay(0.5s): steps={cd(0, dt)}  (expect 50)")
    assert cd(0, dt) == 50

    rd = random_delay(0.5, dt, seed=1)
    samples = [rd(k, dt) for k in range(2000)]
    print(f"random_delay: mean_steps={np.mean(samples):.2f} "
          f"(expect ~50, since noise is symmetric around 0), "
          f"min={min(samples)}, max={max(samples)}")
    print("delay.py checkpoint passed")
