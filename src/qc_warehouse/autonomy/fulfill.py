"""Order fulfilment: for each ordered item, navigate -> identify -> grasp -> carry -> pack."""

from __future__ import annotations

import logging
from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np

from qc_warehouse.autonomy.grasping import GraspError, plan_grasp
from qc_warehouse.autonomy.manipulation import HeldItem, SkillError, pick, place, raise_for_carry, travel_posture
from qc_warehouse.autonomy.localization import LocalizationLostError
from qc_warehouse.autonomy.mapping import MapProvider
from qc_warehouse.autonomy.motion import drive_line, look_at, navigate_to, rotate_to, step_back_from
from qc_warehouse.autonomy.orders import Order
from qc_warehouse.autonomy.perception import Detection, HeadCamera, identify_items
from qc_warehouse.autonomy.poses import SHELF_STANDOFF, SIDE_GRASP_EXTRA_STANDOFF, STATION_STANDOFF, reach_pose, stage_pose
from qc_warehouse.autonomy.runner import SimRunner
from qc_warehouse.autonomy.stretch import BasePose, Stretch
from qc_warehouse.autonomy.warehouse import Tote, Warehouse
from qc_warehouse.config import Item

log = logging.getLogger(__name__)

SHELF_STAGE_DISTANCE = 1.0
STATION_STAGE_DISTANCE = 1.3
LOOK_ABOVE_SLOT = 0.08   # aim the head camera slightly above the slot's board
SURVEY_TURNS = 3


@dataclass(frozen=True)
class ItemResult:
    order_id: str
    item: str
    success: bool
    detail: str
    grasp: str = ""
    sim_seconds: float = 0.0


@dataclass(frozen=True)
class OrderResult:
    order_id: str
    tote: int
    items: tuple[ItemResult, ...] = field(default_factory=tuple)

    @property
    def success(self) -> bool:
        return all(r.success for r in self.items)


class OrderPicker:
    def __init__(
        self,
        robot: Stretch,
        runner: SimRunner,
        warehouse: Warehouse,
        maps: MapProvider,
        camera: HeadCamera,
        on_event: Callable[[str], None] | None = None,
        survey: bool = False,
    ):
        self.robot, self.runner, self.wh, self.maps, self.camera = robot, runner, warehouse, maps, camera
        self._emit = on_event or (lambda msg: log.info(msg))
        self._survey = survey
        self._lost: str | None = None  # set when localization is lost: stop all motion

    # ---- top level -----------------------------------------------------------
    def fulfil(self, orders: tuple[Order, ...]) -> tuple[OrderResult, ...]:
        try:
            travel_posture(self.robot, self.runner)
        except SkillError as exc:
            self._emit(f"could not stow the arm before starting: {exc}")
        if self._survey:
            self._survey_surroundings()
        results = []
        for index, order in enumerate(orders):
            tote = self.wh.tote(index)
            self._emit(f"order {order.order_id}: {', '.join(order.items)} -> tote_{index}")
            items = tuple(self._fetch(order.order_id, name, tote) for name in order.items)
            results.append(OrderResult(order.order_id, index, items))
            self._emit(f"order {order.order_id}: {'PACKED' if results[-1].success else 'INCOMPLETE'}")
        if self._lost is None:
            self._return_to_dock()
        else:
            self.robot.stop()
            self._emit(f"stopped in place: {self._lost}")
        return tuple(results)

    def _fetch(self, order_id: str, name: str, tote: Tote) -> ItemResult:
        start = self.runner.time
        progress = {"grasp": ""}
        if self._lost is not None:
            return ItemResult(order_id, name, False, f"not attempted: {self._lost}")
        try:
            self._pick_and_pack(name, tote, progress)
        except KeyboardInterrupt:
            raise
        except LocalizationLostError as exc:
            self._lost = f"localization lost ({exc})"
            self.robot.stop()
            self._emit(f"  {name}: FAILED - {self._lost}")
            return ItemResult(order_id, name, False, self._lost, progress["grasp"], self.runner.time - start)
        except (SkillError, GraspError, KeyError) as exc:
            return self._failed(order_id, name, str(exc), progress["grasp"], start)
        except Exception as exc:  # keep earlier results even if something unexpected breaks
            log.exception("unexpected error while fetching %s", name)
            return self._failed(order_id, name, f"unexpected {type(exc).__name__}: {exc}", progress["grasp"], start)
        if not self.wh.item_in_tote(name, tote):
            where = np.round(self.wh.item_position(name), 2).tolist()
            self._emit(f"  {name}: FAILED - released but not inside tote_{tote.index} (item at {where})")
            return ItemResult(order_id, name, False, f"not inside tote after release (item at {where})", progress["grasp"], self.runner.time - start)
        grasp = progress["grasp"]
        self._emit(f"  {name}: packed in tote_{tote.index} ({grasp} grasp, {self.runner.time - start:.0f}s)")
        return ItemResult(order_id, name, True, "packed", grasp, self.runner.time - start)

    def _pick_and_pack(self, name: str, tote: Tote, progress: dict[str, str]) -> None:
        item = self.wh.item(name)
        held = self._pick_from_shelf(item, progress)
        self._pack(held, tote)

    def _failed(self, order_id: str, name: str, reason: str, grasp: str, start: float) -> ItemResult:
        self._emit(f"  {name}: FAILED - {reason}")
        self._recover()
        return ItemResult(order_id, name, False, reason, grasp, self.runner.time - start)

    # ---- phases --------------------------------------------------------------
    def _survey_surroundings(self) -> None:
        """Turn a full circle on the dock so SLAM sees past the mast's lidar shadow."""
        self._emit("surveying: one turn on the dock to seed the SLAM map")
        start = self.robot.base_pose().yaw
        try:
            for k in range(1, SURVEY_TURNS + 1):
                rotate_to(self.robot, self.runner, start + 2 * np.pi * k / SURVEY_TURNS)
            self.maps.refresh()
        except SkillError as exc:
            self._emit(f"survey incomplete: {exc}")
        except LocalizationLostError as exc:
            self._lost = f"localization lost ({exc})"
            self._emit(f"survey aborted: {self._lost}")

    def _pick_from_shelf(self, item: Item, progress: dict[str, str]) -> HeldItem:
        slot = self.wh.slot_position(item)
        normal = self.wh.shelf_normal(item)
        goal = reach_pose(slot[:2], normal, SHELF_STANDOFF)
        self._emit(f"  {item.name}: navigating to shelf {item.shelf} level {item.level} slot {item.slot}")
        navigate_to(self.robot, self.runner, self.maps, goal, SHELF_STAGE_DISTANCE)

        # One frame from seeing the item to closing on it: a SLAM correction in between
        # would shift the gripper relative to the detection.
        with self.robot.freeze_localization():
            detection = self._identify(item, slot)
            plan = plan_grasp(detection, -normal, self.wh.board_z(item), self.wh.ceiling_z(item))
            progress["grasp"] = plan.strategy
            self._emit(f"  {item.name}: identified at {np.round(detection.center, 3).tolist()}, {plan.strategy} grasp")
            if not plan.is_top:
                step_back_from(self.robot, self.runner, normal, SIDE_GRASP_EXTRA_STANDOFF)
            held = pick(self.robot, self.runner, plan, -normal)
        pose = self.robot.base_pose()
        drive_line(self.robot, self.runner, stage_pose(BasePose(pose.x, pose.y, goal.yaw), SHELF_STAGE_DISTANCE))  # back out
        return held

    def _identify(self, item: Item, slot: np.ndarray) -> Detection:
        look_at(self.robot, self.runner, slot + np.array([0.0, 0.0, LOOK_ABOVE_SLOT]))
        frame = self.camera.capture(self.robot.data, self.robot.estimated_data())
        detections = identify_items(self.robot.model, frame, self.wh.item_bodies)
        if item.name not in detections:
            seen = ", ".join(sorted(detections)) or "nothing"
            raise SkillError(f"{item.name} not found at shelf {item.shelf} slot {item.slot} (camera sees: {seen})")
        return detections[item.name]

    def _pack(self, held: HeldItem, tote: Tote) -> None:
        raise_for_carry(self.robot, self.runner, held, tote.rim_z)
        goal = reach_pose(tote.center, self.wh.station_normal, STATION_STANDOFF)
        self._emit(f"  {held.plan.item}: carrying to packing station tote_{tote.index}")
        navigate_to(self.robot, self.runner, self.maps, goal, STATION_STAGE_DISTANCE)
        place(self.robot, self.runner, held, tote.center, tote.floor_z, tote.rim_z, -self.wh.station_normal)
        drive_line(self.robot, self.runner, stage_pose(goal, STATION_STAGE_DISTANCE))

    def _recover(self) -> None:
        """Best effort: open the gripper, stow the arm, and back away from whatever we were at."""
        self.robot.stop()
        self.robot.command(gripper=0.04)
        self.runner.wait(1.0)
        try:
            travel_posture(self.robot, self.runner)
            pose = self.robot.base_pose()
            drive_line(self.robot, self.runner, stage_pose(pose, 0.8), timeout=20.0)
        except SkillError as exc:
            self._emit(f"  recovery incomplete: {exc}")

    def _return_to_dock(self) -> None:
        dock = self.wh.layout.robot_dock
        goal = BasePose(dock.pos[0], dock.pos[1], np.radians(dock.yaw_deg))
        self._emit("returning to dock")
        try:
            navigate_to(self.robot, self.runner, self.maps, goal, 1.0)
        except SkillError as exc:
            self._emit(f"could not return to dock: {exc}")
