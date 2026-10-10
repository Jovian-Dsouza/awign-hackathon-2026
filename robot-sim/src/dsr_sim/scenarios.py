"""The simulations you can choose from. Each scenario is a scene setup plus a scripted run, both driven
by the same `Params`:

  shelf_to_tray      pick the item out of the rack you choose and carry it to the drop tray
  restock_misplaced  the item has been left anywhere on the table: pick it up and put it back in a rack
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import mujoco
import numpy as np

from dsr_sim import world as W
from dsr_sim.pick import (
    ARM_JOINTS,
    GRASP_ROLL,
    GRIPPER_CLOSED_DEG,
    GRIPPER_OPEN_DEG,
    GRIPPER_RELEASE_DEG,
    PINCH_Z,
    SHOULDER_FORWARD,
    TRAY_REACH,
    Task,
)
from dsr_sim.world import ITEM, Params

TABLE_CLEARANCE = 0.002
GRASP_Z_OFF = 0.015  # jaw middle above the item's centre
GRASP_Y_OFF = 0.03  # jaw middle past the item's centre (item sits mid-finger)
MOUTH_GAP = 0.095  # tool stops this far short of the item before driving in
ADVANCE = MOUTH_GAP + GRASP_Y_OFF
RACK_RELEASE_DEG = 60.0  # jaw opening when letting go inside a rack (limited by the divider walls)
CARRY_POSE = (0, -80, 85, 55, GRASP_ROLL, GRIPPER_CLOSED_DEG)
REST_POSE = (0, -100, 95, 65, 0, 0)
RECENTER_GAIN = 1.0  # +1 moves the jaws toward the item's centre line, -1 away, 0 off (calibration knob)
GRIP_CLOSE_S = 0.7  # seconds to close the jaws (slower = the item centres itself instead of being shoved)
CARRY_TUCK_S = 1.5  # slow, so the held item is not flung out of the jaws while the arm moves
TRAY_ROLL = 0.0  # wrist roll over the tray: jaws open up/down, so the item simply drops out
CARRY_REACH = 0.24  # shoulder-to-jaws distance while carrying (jaws level, in front of the rover)
CARRY_HEIGHT = 0.2  # m above the table while carrying
UP = np.array([0.0, 0.0, 1.0])
ITEM_HALF_HEIGHT = 0.0245  # soup can half height (scaled), m
LATERAL_BIAS = 0.0  # m, shifts the grasp sideways (left of the approach direction); calibration knob


class Unreachable(Exception):
    """The arm / rover cannot do what was asked from anywhere; the message is for the user."""


def _fwd(yaw: float) -> np.ndarray:
    return np.array([np.cos(yaw), np.sin(yaw), 0.0])


def _rot(yaw: float, v: np.ndarray) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]])


@dataclass(frozen=True)
class Scenario:
    name: str
    title: str
    description: str
    run: Callable[[Task, Params], bool]
    setup: Callable[[mujoco.MjModel, mujoco.MjData, Params], None] | None = None


# ------------------------------------------------------------------ scene setup
def _joint_slice(model: mujoco.MjModel, name: str) -> slice:
    adr = model.jnt_qposadr[model.joint(f"item_{name}_joint").id]
    return slice(adr, adr + 7)


def _shift(model: mujoco.MjModel, data: mujoco.MjData, name: str, delta: np.ndarray) -> None:
    data.qpos[_joint_slice(model, name)][:3] += delta


def setup_shelf_to_tray(model: mujoco.MjModel, data: mujoco.MjData, p: Params) -> None:
    """Put the item in the chosen rack (the rack's usual occupant swaps into the item's home rack)."""
    level, slot = p.rack
    home = W.home_rack()
    if (level, slot) == home:
        return
    occupant = W.occupied_racks().get((level, slot))
    if occupant:
        _shift(model, data, occupant, W.rack_pos(*home) - W.rack_pos(level, slot))
    _shift(model, data, ITEM, W.rack_pos(level, slot) - W.rack_pos(*home))
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)


def setup_misplaced(model: mujoco.MjModel, data: mujoco.MjData, p: Params) -> None:
    """Take the item out of its rack and leave it standing on the table at p.item_xy."""
    mujoco.mj_forward(model, data)
    centre = data.xpos[model.body(f"item_{ITEM}").id].copy()
    board_top = W.rack_pos(*W.home_rack())[2]
    target = np.array([p.item_xy[0], p.item_xy[1], centre[2] - board_top + TABLE_CLEARANCE])
    _shift(model, data, ITEM, target - centre)
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)


# ------------------------------------------------------------------ skills
def _reach_or_fail(task: Task, mouth: np.ndarray, heading: float, what: str) -> float:
    reach = task.plan_reach(mouth, heading)
    if reach is None:
        raise Unreachable(f"the arm cannot hold the jaws level at the {what} from any stand-off distance")
    return reach


def _drive_route(task: Task, goal: tuple[float, float], yaw: float, speed: float = 0.35, precise: bool = False) -> None:
    px, py, _ = task.robot.pose
    route = W.find_route((px, py), goal)
    if route is None:
        raise Unreachable("no clear path for the rover to get there")
    for wx, wy in route[:-1]:
        task.drive_to(wx, wy, yaw, speed=speed)
    task.drive_to(goal[0], goal[1], yaw, speed=speed, precise=precise)


def _approach_axis(task: Task) -> np.ndarray:
    """Unit (x, y) direction the jaws actually point (the IK trades a few degrees of heading for position).
    The rover is holonomic, so we drive in along this axis rather than along its own heading."""
    a = task.data.site_xmat[task.site].reshape(3, 3)[:, 0][:2]
    return a / (np.linalg.norm(a) + 1e-12)


def _recenter(task: Task, name: str, heading: float) -> None:
    """With the jaws open, slide the rover sideways until the item is centred between them, so that backing
    away does not drag it along a finger. (Uses the true item position; on hardware, the wrist camera.)"""
    d = task.data
    axis = _approach_axis(task)
    side = np.array([-axis[1], axis[0]])
    lateral = RECENTER_GAIN * float(np.dot(side, (task.item_pos(name) - task.jaw_centre())[:2]))
    task.notes["release_recenter_m"] = round(lateral, 4)
    if abs(lateral) > 0.001:
        px, py, _ = task.robot.pose
        task.drive_to(px + side[0] * lateral, py + side[1] * lateral, heading, speed=0.08)


def _grasp(task: Task, name: str, heading: float, stage_xy=None, reach: float | None = None) -> np.ndarray:
    """Approach `name` along `heading`, grip it and back out. Returns the jaw-centre offset in the heading frame."""
    fwd = _fwd(heading)
    item0 = task.item_pos(name)
    mouth_xy = item0[:2] - fwd[:2] * MOUTH_GAP
    left = np.array([-np.sin(heading), np.cos(heading), 0.0])
    pre = np.array([mouth_xy[0], mouth_xy[1], item0[2] + GRASP_Z_OFF])
    if reach is None:
        reach = _reach_or_fail(task, pre, heading, "item")
    stage = mouth_xy - fwd[:2] * (reach + SHOULDER_FORWARD)
    task.notes.update(pick_heading_deg=round(float(np.degrees(heading)), 1), pick_reach=reach, pick_stage=[round(float(v), 3) for v in stage])
    grasp_pt = item0 + fwd * GRASP_Y_OFF + UP * GRASP_Z_OFF

    task.stage = "drive_to_item"
    _drive_route(task, (stage[0], stage[1]), heading, precise=True)
    task.stage = "pre_grasp"
    task.reach(pre + UP * 0.02, fwd, 1.3, GRIPPER_OPEN_DEG, GRASP_ROLL, PINCH_Z)
    task.reach(pre, fwd, 0.5, GRIPPER_OPEN_DEG, GRASP_ROLL, PINCH_Z)
    # Sideways correction: measure where the jaw centre really is next to the item (after the arm has settled)
    # and slide the holonomic rover sideways to cancel it; repeat once to refine. On the real robot this is the
    # wrist camera's job; here we read the truth.
    d = task.data
    axis = _approach_axis(task)
    side = np.array([-axis[1], axis[0]])  # left of the approach axis
    for _ in range(2):
        task.run(0.5)
        zaxis0 = d.site_xmat[task.site].reshape(3, 3)[:, 2]
        jaw_mid0 = d.site_xpos[task.site] + zaxis0 * PINCH_Z
        axis = _approach_axis(task)
        side = np.array([-axis[1], axis[0]])
        lateral = float(np.dot(side, (task.item_pos(name) - jaw_mid0)[:2])) + LATERAL_BIAS
        task.notes["lateral_correction_m"] = round(lateral, 4)
        if abs(lateral) < 0.0015:
            break
        px, py, _ = task.robot.pose
        task.drive_to(px + side[0] * lateral, py + side[1] * lateral, heading, speed=0.1, precise=True)
        stage = stage + side * lateral
    axis = _approach_axis(task)
    px, py, _ = task.robot.pose
    task.stage = "drive_in"
    task.drive_to(px + axis[0] * ADVANCE, py + axis[1] * ADVANCE, heading, speed=0.15, precise=True)
    task.stage = "grasp"
    task.move_arm({"gripper": GRIPPER_CLOSED_DEG}, GRIP_CLOSE_S)
    task.run(0.6)  # let the squeeze build before anything moves
    if task.assist:
        held = task.attach(name)
        task.notes["assist_attached"] = held
        if not held:
            task.say("  (grasp assist: the jaws are not around the item, nothing attached)")
    task.stage = "lift_back_out"
    task.reach(grasp_pt + UP * 0.02, fwd, 0.5, GRIPPER_CLOSED_DEG, GRASP_ROLL, PINCH_Z)
    # Where the jaw centre sits relative to the item, in the approach frame (used to place it precisely).
    d = task.data
    zaxis = d.site_xmat[task.site].reshape(3, 3)[:, 2]
    jaw_centre = d.site_xpos[task.site] + zaxis * PINCH_Z
    hold = _rot(-heading, jaw_centre - task.item_pos(name))
    task.notes["hold_offset_m"] = [round(float(v), 4) for v in hold]
    task.drive_to(px, py, heading, speed=0.15)
    task.notes["item_lifted_z"] = round(float(task.item_pos(name)[2]), 4)
    return hold


def _place(task: Task, name: str, target: np.ndarray, heading: float, hold: np.ndarray) -> None:
    """Carry-pose to the `heading` stand-off, slide the held item to `target` (item centre), release, back out."""
    fwd = _fwd(heading)
    jaw_target = target + _rot(heading, hold)
    pre = jaw_target - fwd * ADVANCE
    reach = _reach_or_fail(task, pre, heading, "rack")
    stage = pre[:2] - fwd[:2] * (reach + SHOULDER_FORWARD)
    task.notes.update(place_reach=reach, place_stage=[round(float(v), 3) for v in stage])

    task.stage = "drive_to_rack"
    _drive_route(task, (stage[0], stage[1]), heading, speed=0.3, precise=True)
    task.stage = "slide_in"
    task.reach(pre, fwd, 1.5, GRIPPER_CLOSED_DEG, GRASP_ROLL, PINCH_Z)
    task.run(0.4)
    axis = _approach_axis(task)
    px, py, _ = task.robot.pose
    task.drive_to(px + axis[0] * ADVANCE, py + axis[1] * ADVANCE, heading, speed=0.15, precise=True)
    task.run(0.2)
    task.stage = "release"
    task.detach()  # grasp assist off: from here on the item is just resting in the rack
    task.move_arm({"gripper": RACK_RELEASE_DEG}, 0.5)
    task.run(0.4)
    task.stage = "back_out"
    _recenter(task, name, heading)
    ax = _approach_axis(task)
    cx, cy, _ = task.robot.pose
    task.drive_to(cx - ax[0] * ADVANCE * 1.2, cy - ax[1] * ADVANCE * 1.2, heading, speed=0.15)
    task.move_arm(dict(zip(ARM_JOINTS, REST_POSE)), 1.0)


def _carry_pose(task: Task, heading: float) -> None:
    """Draw the held item in toward the body and a little up, jaws still level, for the drive.

    Keeping the gripper's orientation unchanged (only translating it) is what stops the item working loose;
    folding the wrist while holding it flung it out of the jaws.
    """
    fwd = _fwd(heading)
    px, py, _ = task.robot.pose
    base = np.array([px, py, 0.0]) + fwd * (SHOULDER_FORWARD + CARRY_REACH)
    point = np.array([base[0], base[1], CARRY_HEIGHT])
    task.stage = "carry"
    task.reach(point, fwd, CARRY_TUCK_S, GRIPPER_CLOSED_DEG, GRASP_ROLL, PINCH_Z)


def _go_home(task: Task) -> None:
    task.say("  returning to the start position", "return_home")
    task.move_arm(dict(zip(ARM_JOINTS, REST_POSE)), 1.0)
    try:
        _drive_route(task, (task.start_pose[0], task.start_pose[1]), task.start_pose[2])
    except Unreachable:
        task.drive_to(task.start_pose[0], task.start_pose[1], task.start_pose[2])  # open table: go straight


# ------------------------------------------------------------------ shelf_to_tray
def run_shelf_to_tray(task: Task, p: Params) -> bool:
    m, d = task.model, task.data
    task.set_assist(p.assist)
    level, slot = p.rack
    tray_c = d.site_xpos[m.site("drop_zone").id].copy()
    item0 = task.item_pos(ITEM)
    task.say(f"shelf -> tray: taking the {ITEM} from rack level {level}, slot {slot} at {np.round(item0, 3)}", "start")
    try:
        task.say("  1/4 drive to the rack and grip the item", "grasp")
        hold = _grasp(task, ITEM, np.pi / 2)
        _carry_pose(task, np.pi / 2)

        task.say("  2/4 carry it to the tray", "carry")
        carry = (tray_c[0] - SHOULDER_FORWARD - TRAY_REACH, tray_c[1])
        _drive_route(task, carry, 0.0, speed=0.3)

        task.say("  3/4 set it down in the tray and let go", "release")
        floor_z = float(m.body("drop_tray").pos[2]) + 0.006  # inside floor of the tray
        centre = np.array([tray_c[0], tray_c[1], floor_z + ITEM_HALF_HEIGHT + 0.004])
        fwd = np.array([1.0, 0.0, 0.0])
        jaw_target = centre + _rot(0.0, hold)
        task.reach(jaw_target + UP * 0.07, fwd, 1.2, GRIPPER_CLOSED_DEG, GRASP_ROLL, PINCH_Z)
        task.reach(jaw_target, fwd, 0.7, GRIPPER_CLOSED_DEG, GRASP_ROLL, PINCH_Z)
        task.detach()
        task.move_arm({"gripper": GRIPPER_RELEASE_DEG}, 0.6)
        task.run(0.4)
        # Withdraw straight back along the jaw axis (like backing out of a rack) before lifting: lifting while
        # the item is still between the open fingers drags it along.
        _recenter(task, ITEM, 0.0)
        ax = _approach_axis(task)
        cx, cy, _ = task.robot.pose
        task.drive_to(cx - ax[0] * 0.15, cy - ax[1] * 0.15, 0.0, speed=0.15)  # arm stays put; the rover backs away
        task.move_arm(dict(zip(ARM_JOINTS, REST_POSE)), 1.0)
    except Unreachable as exc:
        task.say(f"  cannot do this one: {exc}", "failed")
        return False

    final = task.item_pos(ITEM)
    # resting in the tray (not still hanging in the gripper above it)
    inside = abs(final[0] - tray_c[0]) < 0.13 and abs(final[1] - tray_c[1]) < 0.09 and final[2] < tray_c[2] + 0.05
    task.notes.update(final_item=[round(float(v), 3) for v in final], target=[round(float(v), 3) for v in tray_c])
    task.say(f"  4/4 check: item ended at {np.round(final, 3)} -> {'IN TRAY' if inside else 'NOT in tray'}", "check")
    _go_home(task)
    return bool(inside)


# ------------------------------------------------------------------ restock_misplaced
def _plan_table_pick(task: Task, item: np.ndarray) -> tuple[float, float]:
    """Pick the approach heading (and the stand-off reach) that is reachable, on the table and clear of obstacles."""
    start = np.array(task.robot.pose[:2])
    best = None
    for k in range(8):
        heading = k * np.pi / 4
        fwd = _fwd(heading)
        mouth_xy = item[:2] - fwd[:2] * MOUTH_GAP
        pre = np.array([mouth_xy[0], mouth_xy[1], item[2] + GRASP_Z_OFF])
        reach = task.plan_reach(pre, heading)
        if reach is None:
            continue
        stage = mouth_xy - fwd[:2] * (reach + SHOULDER_FORWARD)
        end = stage + fwd[:2] * ADVANCE
        if not (W.rover_pose_ok(stage[0], stage[1], heading) and W.rover_pose_ok(end[0], end[1], heading)):
            continue
        route = W.find_route(start, stage)
        if route is None:
            continue
        length = sum(np.linalg.norm(np.array(b) - np.array(a)) for a, b in zip([tuple(start)] + route[:-1], route))
        cost = length + (0.3 if k % 2 else 0.0)
        if best is None or cost < best[0]:
            best = (cost, heading, reach)
    if best is None:
        raise Unreachable("the rover cannot get to that spot from any direction (try moving it away from the edge or shelf)")
    return best[1], best[2]


def run_restock(task: Task, p: Params) -> bool:
    m = task.model
    task.set_assist(p.assist)
    dest = p.dest_rack or W.home_rack()
    ref = mujoco.MjData(m)
    ref.qpos[:] = m.qpos0
    mujoco.mj_forward(m, ref)
    home_centre = ref.xpos[m.body(f"item_{ITEM}").id].copy()
    target = home_centre + (W.rack_pos(*dest) - W.rack_pos(*W.home_rack()))
    drop_z = 0.006  # slide in just above the board, then let it drop the last few mm
    start = task.item_pos(ITEM)
    task.say(f"restock: the {ITEM} was left at {np.round(start, 3)}; it belongs in rack level {dest[0]}, slot {dest[1]}", "start")
    try:
        heading, reach = _plan_table_pick(task, start)
        task.say(f"  1/3 drive to the item (approach heading {np.degrees(heading):.0f} deg) and grip it", "grasp")
        hold = _grasp(task, ITEM, heading, reach=reach)
        _carry_pose(task, heading)

        task.say("  2/3 carry it to the shelf and slide it into its rack", "place")
        _place(task, ITEM, target + np.array([0.0, 0.0, drop_z]), np.pi / 2, hold)
    except Unreachable as exc:
        task.say(f"  cannot do this one: {exc}", "failed")
        return False

    final = task.item_pos(ITEM)
    delta = final - target
    ok = abs(delta[0]) < 0.05 and abs(delta[1]) < 0.06 and abs(delta[2]) < 0.03
    task.notes.update(final_item=[round(float(v), 3) for v in final], target=[round(float(v), 3) for v in target])
    task.say(f"  3/3 check: item ended at {np.round(final, 3)} (slot {np.round(target, 3)}) -> {'BACK IN RACK' if ok else 'NOT in rack'}", "check")
    _go_home(task)
    return bool(ok)


# ------------------------------------------------------------------ registry
SCENARIOS: dict[str, Scenario] = {
    "shelf_to_tray": Scenario(
        "shelf_to_tray",
        "Shelf to tray",
        "Pick the item out of the rack you choose and carry it across the table to the drop tray.",
        run=run_shelf_to_tray,
        setup=setup_shelf_to_tray,
    ),
    "restock_misplaced": Scenario(
        "restock_misplaced",
        "Restock a misplaced item",
        "The item was left somewhere on the table instead of its rack. Pick it up and put it back in a rack.",
        run=run_restock,
        setup=setup_misplaced,
    ),
}


def resolve(choice: str) -> Scenario:
    """Accept a name or a 1-based menu number."""
    names = list(SCENARIOS)
    if choice.isdigit() and 1 <= int(choice) <= len(names):
        return SCENARIOS[names[int(choice) - 1]]
    if choice in SCENARIOS:
        return SCENARIOS[choice]
    raise KeyError(f"unknown simulation {choice!r}; choose one of: {', '.join(names)}")
