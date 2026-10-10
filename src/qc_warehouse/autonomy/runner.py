"""Steps the simulation at a fixed control rate, optionally syncing a passive viewer."""

from __future__ import annotations

import time
from collections.abc import Callable
from typing import Any

import mujoco

CONTROL_DT = 0.02  # 50 Hz control loop


class SimRunner:
    """Fixed-rate control loop.

    `background` (if set) runs every tick that has no foreground control, e.g. a
    base-hold controller while the arm works.
    """

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, viewer: Any | None = None, realtime: bool = False):
        self.model, self.data = model, data
        self.viewer = viewer
        self.realtime = realtime
        self.background: Callable[[], None] | None = None
        self.post_step: list[Callable[[], None]] = []  # per control tick: odometry, SLAM, monitors
        self.substep_hooks: list[Callable[[], None]] = []  # per physics step: high-rate sensors (IMU)
        self._substeps = max(1, round(CONTROL_DT / model.opt.timestep))

    @property
    def time(self) -> float:
        return float(self.data.time)

    def _tick(self) -> None:
        start = time.perf_counter()
        if self.substep_hooks:
            for _ in range(self._substeps):
                mujoco.mj_step(self.model, self.data)
                for hook in self.substep_hooks:
                    hook()
        else:
            mujoco.mj_step(self.model, self.data, nstep=self._substeps)
        for hook in self.post_step:
            hook()
        if self.viewer is not None:
            if not self.viewer.is_running():
                raise KeyboardInterrupt("viewer closed")
            self.viewer.sync()
        if self.realtime:
            remaining = CONTROL_DT - (time.perf_counter() - start)
            if remaining > 0:
                time.sleep(remaining)

    def wait(self, seconds: float) -> None:
        end = self.data.time + seconds
        while self.data.time < end:
            if self.background is not None:
                self.background()
            self._tick()

    def run_until(self, done: Callable[[], bool], timeout: float, control: Callable[[], None] | None = None) -> bool:
        """Call `control` (else `background`) each tick until `done()` is true; False on timeout."""
        end = self.data.time + timeout
        while self.data.time < end:
            if done():
                return True
            step = control if control is not None else self.background
            if step is not None:
                step()
            self._tick()
        return done()
