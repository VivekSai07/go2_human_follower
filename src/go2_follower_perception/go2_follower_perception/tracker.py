"""Single-person tracker with explicit lock, appearance gating, and loss timeout."""

from dataclasses import dataclass

import numpy as np


@dataclass
class Candidate:
    box: tuple[float, float, float, float]
    appearance: np.ndarray
    raised_hand: bool = False
    confidence: float = 1.0

    @property
    def center(self) -> tuple[float, float]:
        x1, y1, x2, y2 = self.box
        return ((x1 + x2) / 2, (y1 + y2) / 2)


class TargetTracker:
    def __init__(self, lock_dwell_s: float = 2.0, loss_timeout_s: float = 5.0, image_width: int = 640):
        self.lock_dwell_s = lock_dwell_s
        self.loss_timeout_s = loss_timeout_s
        self.image_width = image_width
        self.locked = False
        self._reference: np.ndarray | None = None
        self._center: tuple[float, float] | None = None
        self._velocity = np.zeros(2)
        self._last_seen: float | None = None
        self._dwell_start: float | None = None
        self._dwell_reference: np.ndarray | None = None
        self._force = False

    def request_lock(self) -> None:
        self._force = True

    def reset(self) -> None:
        self.locked = False
        self._reference = None
        self._center = None
        self._last_seen = None
        self._dwell_start = None
        self._dwell_reference = None
        self._force = False
        self._velocity[:] = 0

    @staticmethod
    def _similarity(a: np.ndarray, b: np.ndarray) -> float:
        a = a.astype(float)
        b = b.astype(float)
        denom = np.linalg.norm(a) * np.linalg.norm(b)
        return float(np.dot(a, b) / denom) if denom > 0 else 0.0

    def update(self, candidates: list[Candidate], now: float) -> Candidate | None:
        if self.locked:
            assert self._reference is not None and self._center is not None and self._last_seen is not None
            dt = max(0.0, now - self._last_seen)
            predicted = np.array(self._center) + self._velocity * min(dt, 1.0)
            ranked = []
            for c in candidates:
                sim = self._similarity(self._reference, c.appearance)
                distance = float(np.linalg.norm(np.array(c.center) - predicted))
                if sim >= 0.72 and distance <= max(100.0, self.image_width * 0.3):
                    ranked.append((sim - 0.15 * distance / self.image_width, c))
            if ranked:
                chosen = max(ranked, key=lambda pair: pair[0])[1]
                interval = max(now - self._last_seen, 0.01)
                self._velocity = 0.5 * self._velocity + 0.5 * (np.array(chosen.center) - self._center) / interval
                self._center = chosen.center
                self._reference = 0.9 * self._reference + 0.1 * chosen.appearance
                self._last_seen = now
                return chosen
            if now - self._last_seen >= self.loss_timeout_s:
                self.reset()
            return None

        if not candidates:
            self._dwell_start = None
            self._dwell_reference = None
            return None
        centered = [c for c in candidates if abs(c.center[0] - self.image_width / 2) <= self.image_width * 0.15]
        if not centered:
            self._dwell_start = None
            self._dwell_reference = None
            return None
        candidate = max(centered, key=lambda c: c.confidence)
        if self._force or candidate.raised_hand:
            ready = True
        else:
            if self._dwell_start is None or self._dwell_reference is None or self._similarity(self._dwell_reference, candidate.appearance) < 0.72:
                self._dwell_start = now
                self._dwell_reference = candidate.appearance.astype(float).copy()
            ready = now - self._dwell_start >= self.lock_dwell_s
        if ready:
            self.locked = True
            self._force = False
            self._reference = candidate.appearance.astype(float).copy()
            self._center = candidate.center
            self._last_seen = now
            return candidate
        return None
