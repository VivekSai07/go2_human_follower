"""Fail-closed controller state machine independent of ROS."""

import math

import numpy as np

from .planner import choose_velocity


class FollowController:
    def __init__(self, safety_radius: float = 0.35, max_linear: float = 0.4, max_angular: float = 0.8,
                 follow_distance: float = 1.35, sensor_timeout: float = 0.5, target_timeout: float = 5.0,
                 max_accel: float = 0.5):
        self.safety_radius = safety_radius
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.follow_distance = follow_distance
        self.sensor_timeout = sensor_timeout
        self.target_timeout = target_timeout
        self.max_accel = max_accel
        self.enabled = False
        self.target = None
        self.target_at = None
        self.scan = None
        self.scan_at = None
        self.angle_min = -math.pi / 2
        self.angle_increment = math.pi / 180
        self.velocity = (0.0, 0.0)
        self.last_step = None
        self.state = "DISABLED"

    def set_enabled(self, enabled: bool) -> None:
        self.enabled = enabled
        if not enabled:
            self.velocity = (0.0, 0.0)
            self.state = "DISABLED"

    def update_target(self, x: float | None, y: float | None, now: float) -> None:
        if x is not None and y is not None and np.isfinite([x, y]).all():
            self.target = (x, y)
            self.target_at = now
        else:
            self.target = None

    def update_scan(self, ranges: np.ndarray, now: float, angle_min: float = -math.pi / 2,
                    angle_increment: float = math.pi / 180) -> None:
        self.scan = np.asarray(ranges, dtype=float)
        self.scan_at = now
        self.angle_min = angle_min
        self.angle_increment = angle_increment

    def step(self, now: float) -> tuple[float, float]:
        dt = min(0.2, max(0.0, now - self.last_step)) if self.last_step is not None else 0.1
        self.last_step = now
        if not self.enabled:
            self.state = "DISABLED"
            self.velocity = (0.0, 0.0)
            return self.velocity
        if self.scan is None or self.scan_at is None or now - self.scan_at > self.sensor_timeout:
            self.state = "SENSOR_STALE"
            self.velocity = (0.0, 0.0)
            return self.velocity
        valid = np.isfinite(self.scan) & (self.scan > 0)
        if np.count_nonzero(valid) < 3:
            self.state = "SCAN_EMPTY"
            self.velocity = (0.0, 0.0)
            return self.velocity
        angles = self.angle_min + np.arange(len(self.scan)) * self.angle_increment
        if np.count_nonzero(valid & (np.abs(angles) < math.pi / 3)) < 3:
            self.state = "FRONT_SCAN_EMPTY"
            self.velocity = (0.0, 0.0)
            return self.velocity
        braking_distance = self.safety_radius + self.velocity[0] ** 2 / (2 * max(self.max_accel, 1e-6)) + 0.2
        if np.any(valid & (np.abs(angles) < math.pi / 4) & (self.scan < braking_distance)):
            self.state = "BRAKING_STOP"
            self.velocity = (0.0, 0.0)
            return self.velocity
        if np.any(valid & (self.scan < self.safety_radius)):
            self.state = "BUBBLE_STOP"
            self.velocity = (0.0, 0.0)
            return self.velocity
        if self.target is None or self.target_at is None:
            self.state = "WAITING_TARGET"
            self.velocity = (max(0.0, self.velocity[0] - self.max_accel * dt), 0.0)
            return self.velocity
        if now - self.target_at > self.target_timeout:
            self.state = "TARGET_LOST"
            self.velocity = (0.0, 0.0)
            return self.velocity
        if now - self.target_at > self.sensor_timeout:
            self.state = "TARGET_STALE"
            self.velocity = (max(0.0, self.velocity[0] - self.max_accel * dt), 0.0)
            return self.velocity
        obstacles = np.column_stack((self.scan[valid] * np.cos(angles[valid]),
                                     self.scan[valid] * np.sin(angles[valid])))
        tx, ty = self.target
        desired = choose_velocity(tx, ty, obstacles, self.max_linear, self.max_angular, self.follow_distance)
        v = min(desired[0], self.velocity[0] + self.max_accel * dt)
        v = max(desired[0], self.velocity[0] - self.max_accel * dt, 0.0) if desired[0] < self.velocity[0] else v
        self.velocity = (v, desired[1])
        self.state = "FOLLOWING" if v > 0 else "HOLDING"
        return self.velocity
