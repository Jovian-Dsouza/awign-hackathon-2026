"""Dead reckoning from wheel encoders (distance) and the base gyro (heading).

This is the robot's `odom -> base_link` estimate: smooth and high-rate, but it
drifts. SLAM supplies the `map -> odom` correction on top of it.
"""

from __future__ import annotations

import math
from collections import OrderedDict
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from qc_warehouse.autonomy.frames import IDENTITY, BasePose, wrap_angle


@dataclass(frozen=True)
class OdometryNoise:
    """Sensor imperfections; radius_scale models a mis-calibrated wheel radius."""

    encoder_cpr: int | None = 4096      # encoder counts per wheel revolution (None = ideal)
    radius_scale: float = 1.003         # believed / true wheel radius
    gyro_sigma: float = 0.002           # rad/s white noise per sample
    gyro_bias: float = 5e-4             # rad/s constant bias


EXACT = OdometryNoise(encoder_cpr=None, radius_scale=1.0, gyro_sigma=0.0, gyro_bias=0.0)


class WheelGyroOdometry:
    """Integrates encoder + gyro samples into an odom-frame pose, keeping a stamped history.

    Stamps are integer nanoseconds of simulation time.
    """

    def __init__(
        self,
        wheel_radius: float,
        noise: OdometryNoise = OdometryNoise(),
        rng: np.random.Generator | None = None,
        history_seconds: float = 60.0,
    ):
        self._wheel_radius = wheel_radius
        self._noise = noise
        self._rng = rng or np.random.default_rng(0)
        self._history_ns = int(history_seconds * 1e9)
        self._history: OrderedDict[int, BasePose] = OrderedDict()
        self._pose = IDENTITY
        self._last: tuple[int, float, float] | None = None  # (stamp, left, right) quantised angles

    @property
    def pose(self) -> BasePose:
        return self._pose

    def _quantise(self, angle: float) -> float:
        cpr = self._noise.encoder_cpr
        if cpr is None:
            return angle
        step = 2 * math.pi / cpr
        return math.floor(angle / step) * step

    def reset(self, stamp_ns: int, left_angle: float, right_angle: float, pose: BasePose = IDENTITY) -> None:
        self._pose = pose
        self._last = (stamp_ns, self._quantise(left_angle), self._quantise(right_angle))
        self._history = OrderedDict({stamp_ns: pose})

    def update(self, stamp_ns: int, left_angle: float, right_angle: float, gyro_delta_yaw: float) -> BasePose:
        """Advance with new encoder angles and the gyro's integrated yaw change since the last update."""
        if self._last is None:
            raise RuntimeError("odometry used before reset()")
        last_stamp, last_left, last_right = self._last
        dt = (stamp_ns - last_stamp) * 1e-9
        if dt <= 0:
            return self._pose
        left, right = self._quantise(left_angle), self._quantise(right_angle)
        radius = self._wheel_radius * self._noise.radius_scale
        distance = radius * ((left - last_left) + (right - last_right)) / 2
        dyaw = gyro_delta_yaw + self._noise.gyro_bias * dt
        if self._noise.gyro_sigma:
            dyaw += float(self._rng.normal(0.0, self._noise.gyro_sigma)) * dt
        mid = self._pose.yaw + dyaw / 2
        self._pose = BasePose(
            self._pose.x + distance * math.cos(mid),
            self._pose.y + distance * math.sin(mid),
            wrap_angle(self._pose.yaw + dyaw),
        )
        self._last = (stamp_ns, left, right)
        self._remember(stamp_ns)
        return self._pose

    def _remember(self, stamp_ns: int) -> None:
        self._history[stamp_ns] = self._pose
        cutoff = stamp_ns - self._history_ns
        while self._history and next(iter(self._history)) < cutoff:
            self._history.popitem(last=False)

    def pose_at(self, stamp_ns: int) -> BasePose:
        """Odom pose recorded at exactly `stamp_ns` (KeyError if unknown or expired)."""
        if stamp_ns not in self._history:
            raise KeyError(f"no odometry sample at t={stamp_ns * 1e-9:.3f}s")
        return self._history[stamp_ns]


class GyroIntegrator:
    """Integrates the gyro at the physics rate, like an IMU's internal delta-angle output.

    Sampling the rate only once per 20 ms control tick aliases base wobble into
    a few percent of heading error.
    """

    def __init__(self, rate_fn: Callable[[], float], dt: float):
        self._rate_fn, self._dt = rate_fn, dt
        self._accumulated = 0.0

    def __call__(self) -> None:
        self._accumulated += self._rate_fn() * self._dt

    def take(self) -> float:
        delta, self._accumulated = self._accumulated, 0.0
        return delta
