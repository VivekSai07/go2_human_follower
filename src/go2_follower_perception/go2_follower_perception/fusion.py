"""Conservative bearing association between a camera detection and a planar scan."""

import math

import numpy as np


def pixel_bearing(pixel_x: float, image_width: int, horizontal_fov_rad: float, camera_yaw_rad: float = 0.0) -> float:
    if image_width <= 0 or not 0 < horizontal_fov_rad < math.pi:
        raise ValueError("invalid camera calibration")
    return camera_yaw_rad - math.atan((pixel_x - image_width / 2) * 2 * math.tan(horizontal_fov_rad / 2) / image_width)


def range_at_bearing(ranges: np.ndarray, angle_min: float, angle_increment: float, bearing: float, half_window: float) -> float | None:
    if angle_increment <= 0 or half_window < 0:
        raise ValueError("invalid scan geometry")
    center = round((bearing - angle_min) / angle_increment)
    radius = max(1, math.ceil(half_window / angle_increment))
    samples = ranges[max(0, center - radius): min(len(ranges), center + radius + 1)]
    good = samples[np.isfinite(samples) & (samples > 0)]
    return float(np.median(good)) if len(good) >= 3 else None


def associate_bbox_cloud(points_base: np.ndarray, box: tuple[float, float, float, float],
                         image_width: int, image_height: int, hfov_deg: float, vfov_deg: float,
                         camera_xyz: tuple[float, float, float], camera_yaw_rad: float,
                         camera_pitch_rad: float, min_points: int = 3) -> tuple[float, float] | None:
    """Project base_link points into a camera box and take its nearest depth cluster.

    This requires measured camera/LiDAR extrinsics. No match means no target distance.
    """
    xyz = np.asarray(points_base, dtype=float).reshape(-1, 3)
    if len(xyz) < min_points or image_width <= 0 or image_height <= 0:
        return None
    if not 0 < hfov_deg < 180 or not 0 < vfov_deg < 180:
        raise ValueError("invalid camera field of view")
    relative = xyz - np.asarray(camera_xyz)
    cy, sy = math.cos(camera_yaw_rad), math.sin(camera_yaw_rad)
    cp, sp = math.cos(camera_pitch_rad), math.sin(camera_pitch_rad)
    forward = cy * relative[:, 0] + sy * relative[:, 1]
    left = -sy * relative[:, 0] + cy * relative[:, 1]
    depth = cp * forward - sp * relative[:, 2]
    up = sp * forward + cp * relative[:, 2]
    valid = np.isfinite(xyz).all(axis=1) & (depth > 0.15) & (xyz[:, 2] > 0.3) & (xyz[:, 2] < 2.2)
    fx = image_width / (2 * math.tan(math.radians(hfov_deg) / 2))
    fy = image_height / (2 * math.tan(math.radians(vfov_deg) / 2))
    with np.errstate(divide="ignore", invalid="ignore"):
        u = image_width / 2 - fx * left / depth
        v = image_height / 2 - fy * up / depth
    x1, y1, x2, y2 = box
    margin_x, margin_y = (x2 - x1) * 0.08, (y2 - y1) * 0.08
    valid &= (u >= x1 + margin_x) & (u <= x2 - margin_x)
    valid &= (v >= y1 + margin_y) & (v <= y2 - margin_y)
    matched = xyz[valid]
    if len(matched) < min_points:
        return None
    distances = np.linalg.norm(matched[:, :2], axis=1)
    order = np.argsort(distances)
    for start in range(len(order) - min_points + 1):
        group = order[start:start + min_points]
        if distances[group[-1]] - distances[group[0]] <= 0.35:
            cluster = matched[group]
            return float(np.median(cluster[:, 0])), float(np.median(cluster[:, 1]))
    return None
