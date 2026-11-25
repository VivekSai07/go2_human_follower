import numpy as np

from go2_follower_perception.fusion import associate_bbox_cloud


def test_cloud_association_chooses_nearest_person_cluster():
    points = np.array([[2.0, 0.02, 1.1], [2.1, 0, 1.2], [2.0, -0.03, 1.0],
                       [4.0, 0, 1.1], [4.1, 0.02, 1.2], [4.0, -0.03, 1.0]])
    result = associate_bbox_cloud(points, (250, 100, 390, 450), 640, 480,
                                  100.0, 75.0, (0, 0, 0.5), 0.0, 0.0)
    assert result is not None
    assert 1.9 <= result[0] <= 2.2


def test_cloud_association_rejects_unmatched_points():
    points = np.array([[2.0, 2.0, 1.0], [2.1, 2.0, 1.1], [2.0, 2.1, 1.2]])
    assert associate_bbox_cloud(points, (250, 100, 390, 450), 640, 480,
                                100.0, 75.0, (0, 0, 0.5), 0.0, 0.0) is None
