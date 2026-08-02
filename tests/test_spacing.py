"""
test_spacing.py

Verifies spacing.py implements Equations 1-3 (constant time headway
spacing policy and spacing error) correctly.
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from spacing import desired_spacing, measured_gap, spacing_error


def test_desired_spacing_grows_with_velocity():
    h = 1.5
    assert desired_spacing(h, velocity=0.0) == 0.0
    assert desired_spacing(h, velocity=20.0) == 30.0  # 1.5 * 20


def test_zero_error_when_gap_matches_desired_spacing():
    h, v, length = 1.5, 20.0, 4.5
    pos_rear = 0.0
    pos_front = pos_rear + length + h * v  # gap exactly equals desired spacing
    err, gap, d_des = spacing_error(pos_front, pos_rear, length, h, v)
    assert abs(err) < 1e-9


def test_positive_error_when_gap_too_large():
    # gap is bigger than desired spacing -> follower is trailing too far behind
    h, v, length = 1.5, 20.0, 4.5
    pos_rear = 0.0
    pos_front = pos_rear + length + h * v + 5.0  # 5m extra gap
    err, gap, d_des = spacing_error(pos_front, pos_rear, length, h, v)
    assert err > 0


def test_negative_error_when_gap_too_small():
    h, v, length = 1.5, 20.0, 4.5
    pos_rear = 0.0
    pos_front = pos_rear + length + h * v - 5.0  # 5m less gap than desired
    err, gap, d_des = spacing_error(pos_front, pos_rear, length, h, v)
    assert err < 0
