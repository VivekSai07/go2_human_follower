import math

import numpy as np

from go2_follower_control.controller import FollowController
from go2_follower_control.planner import choose_velocity


def scan(distance=5.0):
    return np.full(181, distance)


def test_stale_scan_stops():
    c = FollowController()
    c.set_enabled(True)
    c.update_target(2.0, 0.0, 0.0)
    c.update_scan(scan(), 0.0)
    assert c.step(0.6) == (0.0, 0.0)


def test_empty_scan_stops_even_when_fresh():
    c = FollowController()
    c.set_enabled(True)
    c.update_target(2.0, 0.0, 0.0)
    c.update_scan(np.full(181, np.inf), 0.0)
    assert c.step(0.1) == (0.0, 0.0)


def test_scan_with_only_side_returns_stops():
    c = FollowController()
    c.set_enabled(True)
    c.update_target(2.0, 0.0, 0.0)
    ranges = np.full(181, np.inf)
    ranges[-4:] = 2.0
    c.update_scan(ranges, 0.0)
    assert c.step(0.1) == (0.0, 0.0)


def test_safety_bubble_stops_immediately():
    c = FollowController()
    c.set_enabled(True)
    c.update_target(2.0, 0.0, 0.0)
    ranges = scan()
    ranges[90] = 0.3
    c.update_scan(ranges, 0.0)
    assert c.step(0.1) == (0.0, 0.0)


def test_obstacle_inside_braking_distance_stops_immediately():
    c = FollowController()
    c.set_enabled(True)
    c.update_target(3.0, 0.0, 0.0)
    c.update_scan(scan(), 0.0)
    assert c.step(0.1)[0] > 0
    c.update_scan(scan(), 0.2)
    assert c.step(0.2)[0] > 0
    near = scan()
    near[90] = 0.5
    c.update_scan(near, 0.3)
    assert c.step(0.3) == (0.0, 0.0)


def test_target_loss_decelerates_and_then_unlocks():
    c = FollowController()
    c.set_enabled(True)
    c.update_target(3.0, 0.0, 0.0)
    c.update_scan(scan(), 0.0)
    v, _ = c.step(0.1)
    assert v > 0
    c.update_target(None, None, 0.2)
    c.update_scan(scan(), 0.2)
    v2, _ = c.step(0.3)
    assert 0 <= v2 < v
    assert c.step(5.3) == (0.0, 0.0)


def test_planner_avoids_obstacle_in_forward_path():
    obstacles = np.array([[1.0, 0.0]])
    v, w = choose_velocity(2.0, 0.0, obstacles, 0.4, 0.8)
    assert v < 0.2 or abs(w) > 0.01
