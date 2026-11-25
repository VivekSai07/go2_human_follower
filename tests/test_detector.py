import numpy as np

from go2_follower_perception.detector import decode_pose_output


def test_pose_decoder_detects_raised_wrist_and_scales_box():
    frame = np.full((480, 640, 3), 80, dtype=np.uint8)
    raw = np.zeros((1, 56, 1), dtype=np.float32)
    raw[0, :5, 0] = [320, 320, 100, 200, 0.9]
    raw[0, 5 + 3 * 5:5 + 3 * 5 + 3, 0] = [300, 350, 0.9]
    raw[0, 5 + 3 * 9:5 + 3 * 9 + 3, 0] = [300, 200, 0.9]
    result = decode_pose_output(raw, frame, 640, 0.5)
    assert len(result) == 1
    assert result[0].raised_hand
    assert result[0].box == (270.0, 165.0, 370.0, 315.0)
