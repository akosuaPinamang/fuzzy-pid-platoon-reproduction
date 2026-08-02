"""
platoon_sim.py

Chains N vehicles together: vehicle i's controller reads vehicle (i-1)'s
position and velocity as its reference (each vehicle only "sees" the one
immediately ahead of it, matching the paper's V2V assumption of no
multi-hop / infrastructure communication).

Supports:
  - plain PID or fuzzy-PID controllers per vehicle (anything with a
    .compute(delta, gap_rate) -> u method works)
  - per-vehicle tau (heterogeneous platoon, Table 3)
  - an optional delay function applied to the "signal" each follower
    receives from the vehicle ahead of it (constant / random / none)
"""
import numpy as np
from vehicle_model import Vehicle
from spacing import spacing_error


def lead_speed_profile(t):
    """
    Equation 21 lead-vehicle speed profile. Printed as "2t" for the 10-20s
    ramp, but that is discontinuous; the offset form 2*(t-10) is intended
    (the paper's own 2 m/s^2 acceleration and Figure 5 both require it).
    See VERIFICATION_LOG.md.
    """
    if t <= 10.0:
        return 0.0
    elif t <= 20.0:
        return 2.0 * (t - 10.0)
    else:
        return 20.0


def run_platoon(n_followers, controllers, h=1.5, taus=None, dt=0.01,
                 duration=40.0, delay_fn=None, speed_noise_std=0.0, seed=0):
    """
    n_followers : number of following vehicles (paper uses 3, for a 4-car
                  platoon including the lead vehicle)
    controllers : list of controller objects, one per follower, each with
                  .compute(delta, gap_rate) -> u  and .integral (for reset)
    taus        : list of per-vehicle tau values (defaults to 0.5 for all)
    delay_fn    : optional function delay_fn(k, dt) -> integer number of
                  timesteps to delay the signal from the vehicle ahead.
                  None means no delay (0 timesteps).
    speed_noise_std : if > 0, adds Gaussian noise to the lead vehicle's
                  speed signal (used for the heterogeneous platoon test).
    """
    rng = np.random.default_rng(seed)
    if taus is None:
        taus = [0.5] * n_followers
    assert len(taus) == n_followers
    assert len(controllers) == n_followers

    n = int(duration / dt)
    t_arr = np.arange(n) * dt

    lead_v = np.array([lead_speed_profile(t) for t in t_arr])
    if speed_noise_std > 0:
        lead_v = lead_v + rng.normal(0, speed_noise_std, size=n)
    lead_pos = np.cumsum(lead_v) * dt

    length = 4.5
    vehicles = [Vehicle(tau=taus[i], dt=dt, length=length) for i in range(n_followers)]
    for i, v in enumerate(vehicles):
        v.position = -(i + 1) * length  # zero initial spacing error for all

    # per-vehicle delay buffers: store history of (position, velocity) of
    # the vehicle ahead so we can look back `delay_steps` steps
    pos_history = [list() for _ in range(n_followers)]  # ahead-vehicle position history per follower
    vel_history = [list() for _ in range(n_followers)]

    spacing_errs = np.zeros((n_followers, n))
    velocities = np.zeros((n_followers, n))
    positions = np.zeros((n_followers, n))
    prev_gaps = [None] * n_followers

    for k in range(n):
        t = t_arr[k]
        for i in range(n_followers):
            ahead_pos = lead_pos[k] if i == 0 else vehicles[i - 1].position
            ahead_len = length

            # record current "true" position of the vehicle ahead for delay buffers
            pos_history[i].append(ahead_pos)

            delay_steps = 0 if delay_fn is None else delay_fn(k, dt)
            idx = max(0, len(pos_history[i]) - 1 - delay_steps)
            delayed_ahead_pos = pos_history[i][idx]

            gap = delayed_ahead_pos - vehicles[i].position - ahead_len
            gap_rate = 0.0 if prev_gaps[i] is None else (gap - prev_gaps[i]) / dt
            prev_gaps[i] = gap

            delta, _, _ = spacing_error(
                pos_front=delayed_ahead_pos, pos_rear=vehicles[i].position,
                length_front=ahead_len, h=h, velocity_rear=vehicles[i].velocity
            )
            u = controllers[i].compute(delta, gap_rate)
            vehicles[i].step(u)

            spacing_errs[i, k] = delta
            velocities[i, k] = vehicles[i].velocity
            positions[i, k] = vehicles[i].position

    return {
        "t": t_arr,
        "lead_v": lead_v,
        "lead_pos": lead_pos,
        "spacing_errs": spacing_errs,
        "velocities": velocities,
        "positions": positions,
    }
