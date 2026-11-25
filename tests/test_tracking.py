import numpy as np

from go2_follower_perception.fusion import range_at_bearing
from go2_follower_perception.tracker import Candidate, TargetTracker


def person(x, color, raised=False):
    return Candidate((x - 20, 10, x + 20, 100), np.array(color, dtype=float), raised)


def test_center_dwell_locks_only_after_two_seconds():
    tracker = TargetTracker(lock_dwell_s=2.0, image_width=640)
    assert tracker.update([person(320, [1, 0])], 0.0) is None
    assert tracker.update([person(320, [1, 0])], 1.9) is None
    assert tracker.update([person(320, [1, 0])], 2.0) is not None


def test_crossing_people_keep_appearance_identity():
    tracker = TargetTracker(lock_dwell_s=0.0, image_width=640)
    tracker.update([person(300, [1, 0], True)], 0.0)
    selected = tracker.update([person(320, [0, 1]), person(330, [1, 0])], 0.1)
    assert selected is not None and selected.appearance[0] == 1


def test_lost_target_never_relocks_to_bystander_and_expires():
    tracker = TargetTracker(lock_dwell_s=0.0, loss_timeout_s=5.0, image_width=640)
    tracker.update([person(320, [1, 0], True)], 0.0)
    assert tracker.update([person(330, [0, 1])], 1.0) is None
    assert tracker.locked
    assert tracker.update([], 5.1) is None
    assert not tracker.locked


def test_center_dwell_restarts_when_centered_person_changes():
    tracker = TargetTracker(lock_dwell_s=2.0, image_width=640)
    tracker.update([person(320, [1, 0])], 0.0)
    assert tracker.update([person(320, [0, 1])], 1.5) is None
    assert tracker.update([person(320, [0, 1])], 2.1) is None
    assert tracker.update([person(320, [0, 1])], 3.5) is not None


def test_range_fusion_selects_local_median():
    ranges = np.full(181, np.inf)
    ranges[88:93] = [2.1, 2.0, 2.0, 1.9, 2.0]
    assert range_at_bearing(ranges, -np.pi / 2, np.pi / 180, 0.0, 0.05) == 2.0
