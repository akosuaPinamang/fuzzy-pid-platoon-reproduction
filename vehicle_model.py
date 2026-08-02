"""
vehicle_model.py

Vehicle dynamics model, Equation 6 in the paper:
    tau * a_dot_i(t) + a_i(t) = u_i(t)

This is a first-order lag: the vehicle's acceleration chases the control
signal u_i(t) with time constant tau, integrated with simple Euler steps.
The paper never states a solver or step size, so dt=0.01s is an assumption
(see VERIFICATION_LOG.md).
"""

import numpy as np


class Vehicle:
    def __init__(self, tau, dt, length=4.5):
        """
        tau    : lag time constant (seconds). Larger tau = slower/laggier vehicle.
        dt     : simulation time step (seconds).
        length : vehicle length in metres, used only for spacing bookkeeping.
        """
        self.tau = tau
        self.dt = dt
        self.length = length
        self.position = 0.0
        self.velocity = 0.0
        self.acceleration = 0.0

    def reset(self, position=0.0, velocity=0.0, acceleration=0.0):
        self.position = position
        self.velocity = velocity
        self.acceleration = acceleration

    def step(self, u):
        """
        Advance one time step given control input u (the commanded
        acceleration-ish signal from the controller).

        Discretized form of tau*a_dot + a = u:
            a_dot = (u - a) / tau
            a_new = a + a_dot * dt
            v_new = v + a * dt          (use pre-update accel, standard Euler)
            x_new = x + v * dt
        """
        a_dot = (u - self.acceleration) / self.tau
        # integrate velocity and position using the CURRENT acceleration
        # (semi-implicit / symplectic Euler -> more stable than full explicit)
        self.velocity += self.acceleration * self.dt
        self.position += self.velocity * self.dt
        self.acceleration += a_dot * self.dt
        return self.position, self.velocity, self.acceleration


def simulate_step_response(tau=0.5, dt=0.01, duration=10.0, u_value=1.0):
    """Quick sanity check: feed a constant step input, confirm acceleration
    approaches u_value with the expected first-order lag time constant tau."""
    n = int(duration / dt)
    veh = Vehicle(tau=tau, dt=dt)
    accs = np.zeros(n)
    for k in range(n):
        _, _, a = veh.step(u_value)
        accs[k] = a
    t = np.arange(n) * dt
    return t, accs


if __name__ == "__main__":
    t, accs = simulate_step_response()
    # After ~1 tau, first-order lag should reach ~63% of final value;
    # after ~5 tau, should be within ~1% of final value.
    idx_1tau = int(0.5 / 0.01)
    idx_5tau = int(2.5 / 0.01)
    print(f"a(tau={0.5}s)  = {accs[idx_1tau]:.4f}  (expect ~0.632)")
    print(f"a(5*tau={2.5}s) = {accs[idx_5tau]:.4f}  (expect ~0.993)")
    print(f"a(end)         = {accs[-1]:.4f}  (expect ~1.000)")
