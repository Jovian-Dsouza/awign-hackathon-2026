"""Per-tick sensor pipeline: encoders + gyro -> odometry; lidar -> SLAM; SLAM pose -> map correction.

Runs as a SimRunner post-step hook. The simulator is lockstepped to slam_toolbox:
after publishing a scan that slam_toolbox will process (see ScanGate), it waits
for the matching /pose before stepping on, so results do not depend on how fast
the host is.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass

from qc_warehouse.autonomy.frames import BasePose, compose, inverse
from qc_warehouse.autonomy.lidar import Lidar2D
from qc_warehouse.autonomy.localization import SlamLocalizer
from qc_warehouse.autonomy.odometry import GyroIntegrator, WheelGyroOdometry
from qc_warehouse.autonomy.scan_gate import ScanGate
from qc_warehouse.autonomy.slam_bridge import SlamBackend
from qc_warehouse.autonomy.stretch import Stretch

log = logging.getLogger(__name__)

ACK_TIMEOUT = 5.0  # wall seconds to wait for slam_toolbox to process one scan
MIN_TRAVEL_DISTANCE = 0.15  # must match docker/slam/config/slam_toolbox.yaml
MIN_TRAVEL_HEADING = 0.15


@dataclass
class FrontendStats:
    scans: int = 0
    acks: int = 0
    unexpected_acks: int = 0
    missed_acks: int = 0
    wait_seconds: float = 0.0


def _ns(seconds: float) -> int:
    return round(seconds * 1e9)


class SlamFrontend:
    def __init__(
        self,
        robot: Stretch,
        backend: SlamBackend,
        lidar: Lidar2D,
        odometry: WheelGyroOdometry,
        localizer: SlamLocalizer,
        ack_timeout: float = ACK_TIMEOUT,
    ):
        self.robot, self.backend, self.lidar, self.odometry, self.localizer = robot, backend, lidar, odometry, localizer
        self.gyro = GyroIntegrator(robot.yaw_rate, robot.model.opt.timestep)  # register as a SimRunner substep hook
        self._gate = ScanGate(MIN_TRAVEL_DISTANCE, MIN_TRAVEL_HEADING)
        self._period_ns = _ns(lidar.spec.scan_period)
        self._ack_timeout = ack_timeout
        self._next_scan_ns = 0
        self._odom_ns = -1
        self.last_processed_ns = -1
        self.stats = FrontendStats()

    def start(self) -> None:
        """Zero odometry at the current pose (map origin = here) and send the first scan."""
        now = _ns(self.robot.data.time)
        self.odometry.reset(now, *self.robot.wheel_angles())
        self.gyro.take()
        self._odom_ns = now
        self._scan(now)

    def __call__(self) -> None:
        now = _ns(self.robot.data.time)
        if now <= self._odom_ns:
            return  # time did not advance; keep accumulating the gyro
        self.odometry.update(now, *self.robot.wheel_angles(), self.gyro.take())
        self._odom_ns = now
        if now >= self._next_scan_ns:
            self._scan(now)

    def _scan(self, now: int) -> None:
        self._next_scan_ns = now + self._period_ns
        odom_pose = self.odometry.pose
        self.backend.publish(now, odom_pose, self.lidar.scan(self.robot.data))
        self.stats.scans += 1
        previous_reference = self._gate.reference
        if not self._gate.offer(odom_pose, now):
            self._absorb_unexpected(self.backend.poll_acks())
            return
        started = time.monotonic()
        pose, others = self.backend.wait_ack(now, self._ack_timeout)
        self.stats.wait_seconds += time.monotonic() - started
        self._absorb_unexpected(others)
        if pose is None:
            self.stats.missed_acks += 1
            self._gate.withdraw(now, previous_reference)
            log.warning("slam_toolbox did not process the scan at t=%.2fs; continuing on odometry", now * 1e-9)
            return
        self.stats.acks += 1
        self._correct(now, pose)

    def _absorb_unexpected(self, acks: list[tuple[int, BasePose]]) -> None:
        """Late or unpredicted acks: adopt them (newest wins) and resync the gate."""
        for stamp, pose in sorted(acks, key=lambda a: a[0]):
            self.stats.unexpected_acks += 1
            try:
                self._gate.resync(self.odometry.pose_at(stamp), stamp)
            except KeyError:
                continue
            self._correct(stamp, pose)

    def _correct(self, stamp: int, map_T_base: BasePose) -> None:
        if stamp < self.last_processed_ns:
            return
        self.last_processed_ns = stamp
        self.localizer.apply_correction(compose(map_T_base, inverse(self.odometry.pose_at(stamp))))
