"""Short-horizon sampled local velocity planner."""

import math

import numpy as np


def choose_velocity(target_x: float, target_y: float, obstacles: np.ndarray,
                    max_linear: float, max_angular: float, desired_distance: float = 1.35,
                    robot_radius: float = 0.32) -> tuple[float, float]:
    if not np.isfinite([target_x, target_y]).all() or target_x <= 0:
        return 0.0, 0.0
    obstacles = np.asarray(obstacles, dtype=float).reshape(-1, 2)
    goal_bearing = math.atan2(target_y, target_x)
    goal_speed = min(max_linear, max(0.0, target_x - desired_distance) * 0.6)
    best = (-float("inf"), 0.0, 0.0)
    for v in np.linspace(0.0, goal_speed, 5):
        for w in np.linspace(-max_angular, max_angular, 9):
            x = y = theta = 0.0
            clearance = float("inf")
            safe = True
            for _ in range(25):
                theta += float(w) * 0.1
                x += float(v) * math.cos(theta) * 0.1
                y += float(v) * math.sin(theta) * 0.1
                if len(obstacles):
                    clearance = min(clearance, float(np.min(np.hypot(obstacles[:, 0] - x, obstacles[:, 1] - y))))
                    if clearance < robot_radius + 0.2:
                        safe = False
                        break
            if not safe:
                continue
            heading_error = abs(math.atan2(math.sin(goal_bearing - theta), math.cos(goal_bearing - theta)))
            score = 2.5 * float(v) - 0.7 * heading_error - 0.07 * abs(float(w))
            score += 0.1 * min(clearance, 2.0) if math.isfinite(clearance) else 0.2
            if score > best[0]:
                best = (score, float(v), float(w))
    return best[1], best[2]
