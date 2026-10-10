"""Transport between the simulator and slam_toolbox.

`RosbridgeSlamBackend` talks to the dockerised slam_toolbox through rosbridge
(websocket, via roslibpy). Every backend implements `SlamBackend`, so the rest
of the stack can also run against `FakeSlamBackend` in tests.
"""

from __future__ import annotations

import logging
import queue
import threading
import time
from collections.abc import Callable
from typing import Any, Protocol
from urllib.parse import urlparse

import numpy as np

from qc_warehouse.autonomy import ros_messages as rm
from qc_warehouse.autonomy.frames import BasePose, compose, inverse, yaw_to_quat
from qc_warehouse.autonomy.lidar import LidarSpec
from qc_warehouse.autonomy.localization import LocalizationLostError
from qc_warehouse.autonomy.navigation import OccupancyGrid

log = logging.getLogger(__name__)

DEFAULT_ROSBRIDGE_URL = "ws://localhost:9090"
CONNECT_TIMEOUT = 10.0
MAX_PENDING_ACKS = 1000  # slam_toolbox acks at most a few per waited scan; more means a flood


class SlamUnavailableError(LocalizationLostError):
    """The SLAM backend could not be reached."""


class SlamBackend(Protocol):
    def publish(self, sim_ns: int, odom_pose: BasePose, ranges: np.ndarray) -> None:
        """Publish /clock, the odom -> base_link transform and the scan, all stamped sim_ns."""

    def wait_ack(self, sim_ns: int, timeout: float) -> tuple[BasePose | None, list[tuple[int, BasePose]]]:
        """Block until slam_toolbox reports the map pose for the scan at sim_ns.

        Returns (pose or None on timeout, any other acks that arrived meanwhile).
        """

    def poll_acks(self) -> list[tuple[int, BasePose]]:
        """Acks that arrived without being waited for."""

    def latest_map(self, min_stamp_ns: int, timeout: float) -> dict[str, Any] | None:
        """The newest nav_msgs/OccupancyGrid built from scans up to at least min_stamp_ns."""

    def close(self) -> None: ...


class RosbridgeSlamBackend:
    def __init__(
        self,
        url: str = DEFAULT_ROSBRIDGE_URL,
        spec: LidarSpec = LidarSpec(),
        laser_frame: str = "laser",
        connect_timeout: float = CONNECT_TIMEOUT,
    ):
        import roslibpy  # deferred: only needed when SLAM is actually used

        self._roslibpy = roslibpy
        parsed = urlparse(url)
        if parsed.scheme not in ("ws", "wss") or not parsed.hostname:
            raise ValueError(f"rosbridge URL must look like ws://host:port, got {url!r}")
        self._spec, self._laser_frame = spec, laser_frame
        try:
            port = parsed.port or 9090
        except ValueError as exc:
            raise ValueError(f"invalid port in rosbridge URL {url!r}") from exc
        if parsed.hostname not in ("localhost", "127.0.0.1", "::1") and parsed.scheme == "ws":
            log.warning("rosbridge at %s is not local and the connection is unencrypted", parsed.hostname)
        self._acks: queue.Queue[tuple[int, BasePose]] = queue.Queue(maxsize=MAX_PENDING_ACKS)
        self._map_lock = threading.Lock()
        self._map: dict[str, Any] | None = None
        self._topics: list[tuple[Any, bool]] = []  # (topic, advertised-by-us)
        self._closed = False
        self._ros = roslibpy.Ros(host=parsed.hostname, port=port, is_secure=parsed.scheme == "wss")
        try:
            self._ros.run(timeout=connect_timeout)
        except Exception as exc:  # roslibpy raises a bare Exception on timeout
            self.close()
            raise SlamUnavailableError(f"cannot reach rosbridge at {url} ({exc}). Start it with `uv run qcw-slam up`.") from exc
        try:
            self._setup_topics()
        except Exception as exc:
            self.close()
            raise SlamUnavailableError(f"rosbridge at {url} rejected the SLAM topics ({exc})") from exc

    def _setup_topics(self) -> None:
        roslibpy = self._roslibpy
        self._clock = self._advertise("/clock", "rosgraph_msgs/msg/Clock")
        self._tf = self._advertise("/tf", "tf2_msgs/msg/TFMessage")
        self._scan = self._advertise("/scan", "sensor_msgs/msg/LaserScan")
        for name, msg_type, callback in (
            ("/pose", "geometry_msgs/msg/PoseWithCovarianceStamped", self._on_pose),
            ("/map", "nav_msgs/msg/OccupancyGrid", self._on_map),
        ):
            topic = roslibpy.Topic(self._ros, name, msg_type, queue_length=1 if name == "/map" else 0)
            topic.subscribe(callback)
            self._topics.append((topic, False))

    def _advertise(self, name: str, msg_type: str):
        topic = self._roslibpy.Topic(self._ros, name, msg_type, queue_size=100)
        topic.advertise()
        self._topics.append((topic, True))
        return topic

    # ---- callbacks (rosbridge reactor thread) ----------------------------------
    def _on_pose(self, msg: dict[str, Any]) -> None:
        try:
            ack = rm.parse_pose(msg)
        except (KeyError, TypeError, ValueError, OverflowError, AttributeError):
            log.warning("ignoring malformed /pose message")
            return
        try:
            self._acks.put_nowait(ack)
        except queue.Full:
            log.warning("dropping /pose: too many unprocessed acks")

    def _on_map(self, msg: dict[str, Any]) -> None:
        try:
            rm.validate_map(msg)
        except ValueError as exc:
            log.warning("ignoring /map: %s", exc)
            return
        with self._map_lock:
            self._map = msg

    # ---- SlamBackend -------------------------------------------------------------
    def publish(self, sim_ns: int, odom_pose: BasePose, ranges: np.ndarray) -> None:
        if not self._ros.is_connected:
            raise SlamUnavailableError("lost connection to rosbridge")
        msg = self._roslibpy.Message
        self._clock.publish(msg(rm.clock_msg(sim_ns)))
        self._tf.publish(msg(rm.tf_msg(sim_ns, "odom", "base_link", odom_pose)))
        self._scan.publish(msg(rm.scan_msg(sim_ns, ranges, self._spec, self._laser_frame)))

    def wait_ack(self, sim_ns: int, timeout: float) -> tuple[BasePose | None, list[tuple[int, BasePose]]]:
        others: list[tuple[int, BasePose]] = []
        deadline = time.monotonic() + timeout
        while (remaining := deadline - time.monotonic()) > 0:
            if not self._ros.is_connected:
                raise SlamUnavailableError("lost connection to rosbridge")
            try:
                stamp, pose = self._acks.get(timeout=min(remaining, 0.5))
            except queue.Empty:
                break
            if stamp == sim_ns:
                return pose, others
            others.append((stamp, pose))
        return None, others

    def poll_acks(self) -> list[tuple[int, BasePose]]:
        acks = []
        while True:
            try:
                acks.append(self._acks.get_nowait())
            except queue.Empty:
                return acks

    def latest_map(self, min_stamp_ns: int, timeout: float) -> dict[str, Any] | None:
        deadline = time.monotonic() + timeout
        while True:
            with self._map_lock:
                current = self._map
            if current is not None and rm.parse_stamp(current["header"]["stamp"]) >= min_stamp_ns:
                return current
            if time.monotonic() >= deadline:
                return current  # stale is better than nothing; caller decides
            time.sleep(0.01)

    def close(self) -> None:
        """Release topics and the websocket (idempotent). The roslibpy reactor thread lives on
        until process exit by design: Twisted reactors cannot be restarted."""
        if self._closed:
            return
        self._closed = True
        for topic, advertised in self._topics:
            try:
                topic.unadvertise() if advertised else topic.unsubscribe()
            except Exception:  # best effort on shutdown
                log.debug("ignoring error while closing %s", topic.name)
        try:
            self._ros.close()
        except Exception:
            log.debug("ignoring error while closing the rosbridge connection")


class FakeSlamBackend:
    """Offline stand-in for slam_toolbox: acks every scan with the true pose (plus noise) and
    serves a fixed map. Lets the full localization/mapping stack run in tests without ROS."""

    def __init__(
        self,
        truth: Callable[[], BasePose],
        world_T_map: BasePose,
        map_grid: OccupancyGrid,
        xy_noise: float = 0.0,
        rng: np.random.Generator | None = None,
    ):
        self._truth, self._map_T_world = truth, inverse(world_T_map)
        self._xy_noise, self._rng = xy_noise, rng or np.random.default_rng(0)
        self._pending: dict[int, BasePose] = {}
        self._map = _ros_map_msg(map_grid, self._map_T_world)
        self.published = 0

    def publish(self, sim_ns: int, odom_pose: BasePose, ranges: np.ndarray) -> None:
        self.published += 1
        pose = compose(self._map_T_world, self._truth())
        noise = self._rng.normal(0.0, self._xy_noise, 2) if self._xy_noise else np.zeros(2)
        self._pending = {sim_ns: BasePose(pose.x + float(noise[0]), pose.y + float(noise[1]), pose.yaw)}
        self._map = {**self._map, "header": {**self._map["header"], "stamp": rm.stamp(sim_ns)}}

    def wait_ack(self, sim_ns: int, timeout: float) -> tuple[BasePose | None, list[tuple[int, BasePose]]]:
        return self._pending.pop(sim_ns, None), []

    def poll_acks(self) -> list[tuple[int, BasePose]]:
        return []

    def latest_map(self, min_stamp_ns: int, timeout: float) -> dict[str, Any] | None:
        return self._map

    def close(self) -> None:
        pass


def _ros_map_msg(grid: OccupancyGrid, map_T_world: BasePose) -> dict[str, Any]:
    """nav_msgs/OccupancyGrid (map frame) for a layout-frame boolean grid."""
    half = grid.resolution / 2
    origin = compose(map_T_world, BasePose(grid.origin[0] - half, grid.origin[1] - half, 0.0))
    nx, ny = grid.occupied.shape
    return {
        "header": {"stamp": rm.stamp(0), "frame_id": "map"},
        "info": {
            "resolution": grid.resolution, "width": nx, "height": ny,
            "origin": {"position": {"x": origin.x, "y": origin.y, "z": 0.0}, "orientation": yaw_to_quat(origin.yaw)},
        },
        "data": np.where(grid.occupied.T, 100, 0).astype(int).ravel().tolist(),
    }
