"""Predicts which scans slam_toolbox will process, so the simulator can wait for exactly those.

Mirrors `SlamToolbox::shouldProcessScan` (jazzy) with throttle_scans=1,
minimum_time_interval=0 and check_min_dist_and_heading_precisely=true:
  - the first scan is always processed;
  - scans 2-4 are dropped "for initial stabilization";
  - afterwards a scan is processed when the odom pose moved >= min_distance OR
    turned >= min_heading since the last processed scan.
"""

from __future__ import annotations

from qc_warehouse.autonomy.frames import BasePose, wrap_angle

WARMUP_SCANS = 5


class ScanGate:
    """Tracks slam_toolbox's reference pose (the odom pose of the last processed scan) and its stamp."""

    def __init__(self, min_distance: float, min_heading: float):
        self._min_dist2 = min_distance * min_distance
        self._min_heading = min_heading
        self._count = 0
        self._last: BasePose | None = None
        self._last_ns = -1

    @property
    def reference(self) -> tuple[BasePose | None, int]:
        return self._last, self._last_ns

    def offer(self, odom_pose: BasePose, stamp_ns: int) -> bool:
        """Register one published scan; True if slam_toolbox will process (and ack) it."""
        self._count += 1
        if self._last is None:
            self._last, self._last_ns = odom_pose, stamp_ns
            return True
        if self._count < WARMUP_SCANS:
            return False
        dist2 = float(((odom_pose.xy - self._last.xy) ** 2).sum())
        heading = abs(wrap_angle(odom_pose.yaw - self._last.yaw))
        if dist2 < self._min_dist2 and heading < self._min_heading:
            return False
        self._last, self._last_ns = odom_pose, stamp_ns
        return True

    def resync(self, odom_pose: BasePose, stamp_ns: int) -> None:
        """slam_toolbox processed the scan at stamp_ns: adopt it, unless we already track a newer one."""
        if stamp_ns > self._last_ns:
            self._last, self._last_ns = odom_pose, stamp_ns

    def withdraw(self, stamp_ns: int, previous: tuple[BasePose | None, int]) -> None:
        """A predicted scan was never acked: restore `previous`, unless a newer ack already moved us on."""
        if self._last_ns == stamp_ns:
            self._last, self._last_ns = previous
