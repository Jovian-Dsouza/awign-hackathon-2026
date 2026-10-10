"""Robot control boundary.

`RobotAdapter` is the only thing the workflow engine knows about a robot. Today
the sole implementation is `SimulatedRobotAdapter`, which just waits out
plausible step durations; nothing in this module talks to hardware.

To integrate a real robot (Stretch SDK, ROS 2 action servers, or the MuJoCo
`qc_warehouse` simulator in this repo), implement `RobotAdapter`, set
`simulated = False`, and select it via the ROBOT_ADAPTER env var in `main.py`.
`stop()` on a hardware adapter must only be wired to a vetted, tested stop path;
the dashboard button is never a substitute for the robot's physical E-stop.
"""

from __future__ import annotations

import asyncio
import random
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

STEP_TYPES = ("navigate", "identify", "pick", "transport", "place")


class RobotStopped(Exception):
    """Raised from `execute` when a stop is requested mid-step."""


@dataclass
class StepCommand:
    type: str                 # one of STEP_TYPES
    title: str
    line_key: str             # "<order_id>:<line_index>"; lets sim scenarios correlate steps
    sku: str
    location: str             # shelf slot, e.g. "A-L1-S0"
    destination: str          # dispatch bay
    attempt: int = 1


@dataclass
class StepResult:
    ok: bool
    outcome: str              # machine-readable, e.g. "arrived", "grasped", "grasp_failed", "placed"
    detail: str               # human-readable
    data: dict = field(default_factory=dict)


@dataclass
class Telemetry:
    connected: bool
    zone: str | None
    battery_pct: float | None  # None when the adapter has no real battery telemetry


class RobotAdapter(ABC):
    robot_id: str
    name: str
    model: str
    adapter_name: str
    simulated: bool

    @abstractmethod
    async def execute(self, cmd: StepCommand) -> StepResult:
        """Run one step to completion. Honors pause; raises RobotStopped on stop()."""

    @abstractmethod
    async def wait_until_ready(self) -> None:
        """Block while paused; raise RobotStopped if stopped."""

    @abstractmethod
    def pause(self) -> None: ...

    @abstractmethod
    def resume(self) -> None: ...

    @abstractmethod
    def stop(self) -> None: ...

    @abstractmethod
    def reset(self) -> None: ...

    @property
    @abstractmethod
    def paused(self) -> bool: ...

    @property
    @abstractmethod
    def stopped(self) -> bool: ...

    @abstractmethod
    def telemetry(self) -> Telemetry: ...

    def park(self) -> None:
        """Called when the robot has no mission. Optional."""


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------

ANOMALIES = ("weight_mismatch", "nfc_missing", "sku_mismatch", "grasp_failure")


class SimulationScenario:
    """Decides, once per order line, whether something goes wrong in the simulation.

    Shared by the simulated robot (grasp failures) and simulated sensors (bad
    readings) so one line's story is consistent across both.
    """

    def __init__(self, anomaly_rate: float, seed: int | None = None):
        self.anomaly_rate = anomaly_rate
        self.rng = random.Random(seed)
        self._by_line: dict[str, str | None] = {}
        self._injected: set[str] = set()
        self.pending: str | None = None  # injected from the dashboard; applies to the next line

    def inject(self, kind: str) -> None:
        self.pending = kind

    def anomaly_for(self, line_key: str) -> str | None:
        if line_key not in self._by_line:
            if self.pending:
                self._by_line[line_key], self.pending = self.pending, None
                self._injected.add(line_key)
            elif self.rng.random() < self.anomaly_rate:
                self._by_line[line_key] = self.rng.choices(ANOMALIES, weights=(4, 3, 1, 3))[0]
            else:
                self._by_line[line_key] = None
        return self._by_line[line_key]

    def is_injected(self, line_key: str) -> bool:
        return line_key in self._injected


_STEP_SECONDS = {"navigate": 5.0, "identify": 2.5, "pick": 4.0, "transport": 5.0, "place": 2.5}


class SimulatedRobotAdapter(RobotAdapter):
    adapter_name = "simulated"
    simulated = True

    def __init__(self, scenario: SimulationScenario, speed: float = 1.0,
                 robot_id: str = "SIM-STRETCH-01"):
        self.robot_id = robot_id
        self.name = "Stretch 3 (simulated)"
        self.model = "Hello Robot Stretch 3"
        self.scenario = scenario
        self.speed = max(speed, 0.05)
        self._paused = False
        self._stopped = False
        self._zone: str | None = "Dock"
        self._tick = 0.1

    # ---- control ----
    def pause(self) -> None:
        self._paused = True

    def resume(self) -> None:
        self._paused = False

    def stop(self) -> None:
        self._stopped = True

    def reset(self) -> None:
        self._stopped = False
        self._paused = False

    @property
    def paused(self) -> bool:
        return self._paused

    @property
    def stopped(self) -> bool:
        return self._stopped

    async def wait_until_ready(self) -> None:
        while self._paused or self._stopped:
            if self._stopped:
                raise RobotStopped()
            await asyncio.sleep(self._tick)

    def telemetry(self) -> Telemetry:
        return Telemetry(connected=True, zone=self._zone, battery_pct=None)

    def park(self) -> None:
        self._zone = "Dock"

    # ---- execution ----
    async def _wait(self, seconds: float) -> None:
        remaining = seconds / self.speed
        while remaining > 0:
            if self._stopped:
                raise RobotStopped()
            if not self._paused:
                remaining -= self._tick
            await asyncio.sleep(self._tick)

    async def execute(self, cmd: StepCommand) -> StepResult:
        shelf = cmd.location.split("-")[0]
        self._zone = {
            "navigate": f"Aisle → Shelf {shelf}",
            "identify": f"Shelf {shelf} · {cmd.location}",
            "pick": f"Shelf {shelf} · {cmd.location}",
            "transport": "Walkway → Packing station",
            "place": f"Packing station · {cmd.destination}",
        }[cmd.type]
        await self._wait(_STEP_SECONDS[cmd.type])

        anomaly = self.scenario.anomaly_for(cmd.line_key)
        if cmd.type == "navigate":
            return StepResult(True, "arrived", f"Reached staging point for {cmd.location}")
        if cmd.type == "identify":
            return StepResult(True, "identified", f"Item {cmd.sku} detected in slot {cmd.location}")
        if cmd.type == "pick":
            # Injected grasp failures persist across retries; random ones recover on retry.
            if anomaly == "grasp_failure" and (cmd.attempt == 1 or self.scenario.is_injected(cmd.line_key)):
                return StepResult(False, "grasp_failed", "Gripper closed but no item held (proprioception check)")
            return StepResult(True, "grasped", "Gripper closed; item held")
        if cmd.type == "transport":
            return StepResult(True, "arrived", "Reached packing station")
        return StepResult(True, "placed", f"Released item over {cmd.destination} tote",
                          data={"destination": cmd.destination})
