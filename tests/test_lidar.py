import numpy as np

from go2_follower_perception.lidar_adapter import project_points


def test_slice_excludes_high_and_behind_points():
    points = np.array([[1.0, 0, 0.2], [0.2, 0, 1.5], [-0.2, 0, 0.2]])
    ranges = project_points(points, bins=181, min_z=0.05, max_z=0.7, min_range=0.1)
    assert np.isclose(ranges[90], 1.0)
    assert np.isinf(ranges[0])
