"""Pick and place skills for Stretch 3 (these step the simulation).

All positioning is closed-loop on the measured pinch point (midpoint of the
finger pads), not on nominal kinematics: the lift sags under load and the base
twists slightly when the arm extends.
"""

from __future__ import annotations

import logging
import math
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass

import numpy as np

from qc_warehouse.autonomy.grasping import SIDE_PREGRASP_BACKOFF, TOP_PREGRASP_CLEARANCE, GraspPlan, lift_for
from qc_warehouse.autonomy.motion import BaseHold, SkillError, rotate_to, shift_along_heading
from qc_warehouse.autonomy.runner import CONTROL_DT, SimRunner
from qc_warehouse.autonomy.stretch import ARM_RANGE, GRIPPER_CLOSED, GRIPPER_OPEN, PITCH_DOWN, BasePose, Stretch

log = logging.getLogger(__name__)

TRAVEL_LIFT = 0.45
FOLD_YAW = 1.57                 # wrist yaw that tucks a side-held item ahead of the base
                                # (folding backward jams the gripper against the arm)
GRASP_OVERSHOOT = 0.01          # push slightly past the estimated centre
HELD_MARGIN = 0.003             # fingers blocked this far short of their target = holding something
SIDE_LIFT_AFTER_GRASP = 0.04
TOP_LIFT_AFTER_GRASP = 0.07
TOP_PRESS_WHILE_CLOSING = 0.015 # fingertips arc upward as they close; keep pressing down
TOP_PRESHAPE_MARGIN = 0.016     # open just wider than the item: less fingertip arc before contact
SIDE_PRESHAPE_MARGIN = 0.03
SQUEEZE_DEPTH = 0.07            # close to this much narrower than the item (a full squeeze
                                # makes the soft tips ride up / pops round items out)
GRIPPER_EMPTY_STOP = -0.0115    # where the fingers stop when closing on nothing (pads meet)
RIM_CLEARANCE = 0.06            # carry items this far above tote rims
PLACE_ABOVE_FLOOR = 0.02
RELEASE_CLEARANCE = 0.06        # gripper rises this far above a released item's top
PLACE_REACH_PAST = 0.05         # side grasps release past the tote centre (gripper body clears the wall)
STALL_GRACE = 0.4
ARM_SPEED = 0.25                # m/s arm extension rate (real Stretch: ~0.3)
WRIST_YAW_SPEED = 0.6           # rad/s; a fast fold flings heavier items out of the fingers
LIFT_SPEED = 0.15               # m/s lift rate; fast lifts shake thin packs out of the fingers
SETTLED_SPEED = 0.005           # m/s: gripper considered still below this
DESCEND_OVERDRIVE = 0.01        # command slightly below target to beat lift sag; the guard stops us
LATERAL_TOLERANCE = 0.004
HEIGHT_TOLERANCE = 0.004


@dataclass(frozen=True)
class HeldItem:
    plan: GraspPlan
    fold_yaw: float = 0.0


RAMPED_JOINTS = {"arm": ARM_SPEED, "lift": LIFT_SPEED, "wrist_yaw": WRIST_YAW_SPEED}


def _move(robot: Stretch, runner: SimRunner, timeout: float = 8.0, settle: float = 0.4, **targets: float) -> None:
    """Move joints; lift and arm targets are ramped at real-robot speeds (base hold stays active)."""
    # Clip ramp goals to actuator limits, otherwise the ramp could never "finish".
    ramped = {name: robot.clip_ctrl(name, float(targets.pop(name))) for name in list(targets) if name in RAMPED_JOINTS}
    if "arm" in ramped:
        ramped["arm"] = float(np.clip(ramped["arm"], *ARM_RANGE))
    robot.command(**targets)
    control = _ramp(robot, runner, ramped)

    def done() -> bool:
        ramps_finished = all(abs(robot.ctrl(name) - target) < 1e-6 for name, target in ramped.items())
        return ramps_finished and robot.joint_targets_reached()

    timeout += max((abs(t - robot.ctrl(n)) / RAMPED_JOINTS[n] for n, t in ramped.items()), default=0.0)
    if not runner.run_until(done, timeout, control):
        raise SkillError(f"joints did not reach {ramped | targets} (lift {robot.lift():.3f}, arm {robot.arm_extension():.3f})")
    runner.wait(settle)


def _ramp(robot: Stretch, runner: SimRunner, targets: dict[str, float]):
    """Control step moving each joint target toward its goal at its max speed, plus base hold."""

    def control() -> None:
        for name, target in targets.items():
            step = RAMPED_JOINTS[name] * CONTROL_DT
            current = robot.ctrl(name)
            robot.command(**{name: current + float(np.clip(target - current, -step, step))})
        if runner.background is not None:
            runner.background()

    return control


def _descend_to(robot: Stretch, runner: SimRunner, pinch_z: float, timeout: float = 6.0) -> None:
    """Guarded move: lower until the pinch point reaches `pinch_z` or something blocks the lift."""
    robot.command(lift=robot.ctrl("lift") + (pinch_z - robot.pinch_point()[2]) - DESCEND_OVERDRIVE)
    runner.wait(STALL_GRACE)  # let the lift start moving before judging a stall
    runner.run_until(lambda: robot.pinch_point()[2] <= pinch_z + HEIGHT_TOLERANCE or robot.lift_stalled(), timeout)
    robot.command(lift=robot.lift())  # hold here, don't keep pushing
    runner.wait(0.3)


def travel_posture(robot: Stretch, runner: SimRunner) -> None:
    """Arm in, wrist pointing down beside the base: nothing sticks out while driving."""
    robot.command(wrist_yaw=0.0, wrist_roll=0.0, gripper=0.0)
    _move(robot, runner, arm=0.0)
    _move(robot, runner, lift=TRAVEL_LIFT, wrist_pitch=PITCH_DOWN, settle=1.0)


def task_heading(arm_dir: np.ndarray) -> float:
    """Base heading that points the arm (base -y) along `arm_dir`."""
    return math.atan2(arm_dir[0], -arm_dir[1])


def _settle_gripper(robot: Stretch, runner: SimRunner, timeout: float = 4.0) -> None:
    """Wait until the extended gripper stops swaying, so measurements are not transients."""
    state = {"last": robot.pinch_point()}

    def still() -> bool:
        now = robot.pinch_point()
        moved = float(np.linalg.norm(now - state["last"]))
        state["last"] = now
        return moved < SETTLED_SPEED * 0.02  # per 20 ms control tick

    if not runner.run_until(still, timeout):
        log.warning("gripper still swaying after %.1fs; measuring anyway", timeout)


def _align_lateral(robot: Stretch, runner: SimRunner, target_xy: np.ndarray, arm_dir: np.ndarray) -> None:
    """Slide the base along the task heading so the pinch point lines up with the target."""
    heading = task_heading(arm_dir)
    forward = np.array([math.cos(heading), math.sin(heading)])
    _settle_gripper(robot, runner)
    error = float((np.asarray(target_xy) - robot.pinch_point()[:2]) @ forward)
    if abs(error) > LATERAL_TOLERANCE:
        shift_along_heading(robot, runner, error, heading)


def _reach_to(robot: Stretch, runner: SimRunner, target_xy: np.ndarray, arm_dir: np.ndarray, extra: float) -> None:
    """Extend/retract so the pinch point is `extra` m past `target_xy` along the arm axis."""
    _settle_gripper(robot, runner)
    along = float((target_xy - robot.pinch_point()[:2]) @ arm_dir)
    _move(robot, runner, arm=robot.arm_extension() + along + extra)


def _correct_height(robot: Stretch, runner: SimRunner, z: float) -> None:
    _settle_gripper(robot, runner)
    error = z - robot.pinch_point()[2]
    if abs(error) > HEIGHT_TOLERANCE:
        _move(robot, runner, lift=robot.ctrl("lift") + error)


def _approach(robot: Stretch, runner: SimRunner, plan: GraspPlan, arm_dir: np.ndarray) -> None:
    """Pre-shape, hover in front of / above the item, aligned on the pinch point."""
    target_xy = plan.target[:2]
    hover_z = plan.target[2] + (TOP_PREGRASP_CLEARANCE if plan.is_top else 0.0)
    # Change height with the wrist still pointing down (nothing sticks out toward the
    # shelf), then swing the wrist into the grasp orientation between the boards.
    robot.command(wrist_yaw=0.0, wrist_pitch=PITCH_DOWN)
    _move(robot, runner, lift=lift_for(plan.is_top, hover_z))
    robot.command(wrist_pitch=plan.pitch, wrist_roll=plan.roll, gripper=Stretch.preshape_for(plan.width, TOP_PRESHAPE_MARGIN if plan.is_top else SIDE_PRESHAPE_MARGIN))
    runner.wait(1.2)
    _correct_height(robot, runner, hover_z)
    rotate_to(robot, runner, task_heading(arm_dir))  # square up while the arm is still in
    _align_lateral(robot, runner, target_xy, arm_dir)
    pre_reach = 0.0 if plan.is_top else -SIDE_PREGRASP_BACKOFF
    # Extending the arm twists the base slightly, so alternate reach / re-align.
    for _ in range(2):
        _reach_to(robot, runner, target_xy, arm_dir, pre_reach)
        _align_lateral(robot, runner, target_xy, arm_dir)
    _correct_height(robot, runner, hover_z)


def _close_target(plan: GraspPlan) -> float:
    """Squeeze to ~7 cm narrower than the item: firm, but a full squeeze pops items out."""
    return Stretch.preshape_for(plan.width, -SQUEEZE_DEPTH, minimum=GRIPPER_CLOSED)


def _is_holding(robot: Stretch, close_target: float) -> bool:
    """Proprioceptive grasp check: an object stops the fingers short of where they'd stop empty."""
    empty_stop = max(close_target, GRIPPER_EMPTY_STOP)
    return robot.gripper() - empty_stop > HELD_MARGIN


@contextmanager
def _holding_base(robot: Stretch, runner: SimRunner, arm_dir: np.ndarray) -> Iterator[None]:
    """Servo the base in place (at the task heading) for the duration of an arm skill.

    SLAM corrections are held back meanwhile, so the target and the gripper stay in one frame.
    """
    with robot.freeze_localization():
        pose = robot.base_pose()
        previous = runner.background
        runner.background = BaseHold(robot, BasePose(pose.x, pose.y, task_heading(arm_dir)))
        try:
            yield
        finally:
            runner.background = previous
            robot.stop()


def pick(robot: Stretch, runner: SimRunner, plan: GraspPlan, arm_dir: np.ndarray) -> HeldItem:
    """Grasp the planned item and lift it clear; side-held items are folded ahead of the base."""
    with _holding_base(robot, runner, arm_dir):
        return _pick(robot, runner, plan, arm_dir)


def _pick(robot: Stretch, runner: SimRunner, plan: GraspPlan, arm_dir: np.ndarray) -> HeldItem:
    _approach(robot, runner, plan, arm_dir)
    if plan.is_top:
        _descend_to(robot, runner, plan.target[2])
        robot.command(lift=robot.lift() - TOP_PRESS_WHILE_CLOSING)
    else:
        # Reach past the centre by however far the pads will retract while closing.
        retract = Stretch.side_pad_retraction(robot.ctrl("gripper"), plan.width)
        _reach_to(robot, runner, plan.target[:2], arm_dir, GRASP_OVERSHOOT + retract)

    close_target = _close_target(plan)
    robot.command(gripper=close_target)
    runner.wait(1.5)
    if plan.is_top:
        robot.command(lift=robot.lift())
    if not _is_holding(robot, close_target):
        robot.command(gripper=GRIPPER_OPEN)
        try:
            _move(robot, runner, lift=robot.lift() + 0.03)
            _move(robot, runner, arm=0.0)
        except SkillError:
            pass  # report the miss, not the cleanup; fulfil() recovers further
        raise SkillError(f"grasp of {plan.item} missed (gripper closed fully)")

    lift_after = TOP_LIFT_AFTER_GRASP if plan.is_top else SIDE_LIFT_AFTER_GRASP
    _move(robot, runner, lift=robot.ctrl("lift") + lift_after)
    _move(robot, runner, arm=0.0)
    fold_yaw = 0.0
    if not plan.is_top:
        fold_yaw = FOLD_YAW
        _move(robot, runner, wrist_yaw=fold_yaw, settle=1.0)
    if not _is_holding(robot, close_target):
        raise SkillError(f"{plan.item} slipped out of the gripper while lifting")
    return HeldItem(plan, fold_yaw)


def carry_lift(held: HeldItem, rim_z: float) -> float:
    grasp_z = rim_z + RIM_CLEARANCE + held.plan.hold_offset
    return lift_for(held.plan.is_top, grasp_z)


def raise_for_carry(robot: Stretch, runner: SimRunner, held: HeldItem, rim_z: float) -> None:
    """Ramp the lift to the height that clears tote rims with the held item."""
    _move(robot, runner, lift=carry_lift(held, rim_z))


def place(robot: Stretch, runner: SimRunner, held: HeldItem, drop_xy: np.ndarray, floor_z: float, rim_z: float, arm_dir: np.ndarray) -> None:
    """Lower `held` into a tote whose floor is at floor_z and rim at rim_z."""
    with _holding_base(robot, runner, arm_dir):
        _place(robot, runner, held, drop_xy, floor_z, rim_z, arm_dir)


def _place(robot: Stretch, runner: SimRunner, held: HeldItem, drop_xy: np.ndarray, floor_z: float, rim_z: float, arm_dir: np.ndarray) -> None:
    plan = held.plan
    if not _is_holding(robot, _close_target(plan)):
        raise SkillError(f"{plan.item} was dropped in transit (gripper empty on arrival)")
    _move(robot, runner, lift=carry_lift(held, rim_z), settle=0.5)
    if not plan.is_top:
        _move(robot, runner, wrist_yaw=0.0, settle=1.0)
    reach_past = 0.0 if plan.is_top else PLACE_REACH_PAST
    for _ in range(2):  # align, reach, then re-align once the arm is out
        _align_lateral(robot, runner, drop_xy, arm_dir)
        _reach_to(robot, runner, drop_xy, arm_dir, reach_past)
    _descend_to(robot, runner, floor_z + plan.hold_offset + PLACE_ABOVE_FLOOR)
    release_pinch_z = robot.pinch_point()[2]
    robot.command(gripper=GRIPPER_OPEN)
    runner.wait(1.2)
    # Rise until the open gripper clears the released item's top before pulling the arm in.
    clear_z = max(release_pinch_z + plan.top_offset + RELEASE_CLEARANCE, rim_z + RIM_CLEARANCE)
    _move(robot, runner, lift=robot.ctrl("lift") + (clear_z - robot.pinch_point()[2]))
    travel_posture(robot, runner)
