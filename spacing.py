"""
spacing.py

Constant Time Headway (CTH) spacing policy, Equations 1-5 in the paper.

d_des(t) = D_min + h * v_i(t)          (Eq 1)
gap(t)   = x_{i-1}(t) - x_i(t) - l_{i-1}   (Eq 2, rearranged to be positive)
delta_i(t) = gap(t) - d_des(t)          (Eq 3)

D_min is 0 throughout the paper, so d_des(t) = h * v_i(t).

measured_gap uses the standard road convention (pos_front > pos_rear),
which resolves an internal sign inconsistency in the paper's Equation 2
and gives physically correct closed-loop behaviour (see VERIFICATION_LOG.md).
"""


def desired_spacing(h, velocity, d_min=0.0):
    return d_min + h * velocity


def measured_gap(pos_front, pos_rear, length_front):
    """Positive gap between the rear vehicle and the vehicle in front of it."""
    return pos_front - pos_rear - length_front


def spacing_error(pos_front, pos_rear, length_front, h, velocity_rear, d_min=0.0):
    gap = measured_gap(pos_front, pos_rear, length_front)
    d_des = desired_spacing(h, velocity_rear, d_min)
    return gap - d_des, gap, d_des


if __name__ == "__main__":
    # Checkpoint: error should be exactly zero when gap == h * velocity
    h = 1.5
    v = 20.0
    length = 4.5
    pos_rear = 0.0
    pos_front = pos_rear + length + h * v  # gap exactly matches desired spacing
    err, gap, d_des = spacing_error(pos_front, pos_rear, length, h, v)
    print(f"gap={gap:.4f}  d_des={d_des:.4f}  error={err:.6f}  (expect 0.000000)")
    assert abs(err) < 1e-9, "spacing error checkpoint failed"
    print("spacing.py checkpoint passed")
