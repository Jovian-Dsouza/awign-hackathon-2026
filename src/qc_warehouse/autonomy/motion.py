"""Closed-loop base and head motion skills (these step the simulation)."""

from __future__ import annotations

import math

import mujoco
import numpy as np

from qc_warehouse.autonomy.mapping import MapProvider
from qc_warehouse.autonomy.navigation import (
    PlanningError,
    PursuitParams,
    advance_path,
    astar,
    line_is_free,
    pure_pursuit,
    shortcut_path,
    wrap_angle,
)
from qc_warehouse.autonomy.poses import stage_pose
from qc_warehouse.autonomy.runner import SimRunner
from qc_warehouse.autonomy.stretch import BasePose, Stretch
from qc_warehouse.robot import ROBOT_PREFIX

GOAL_TOLERANCE = 0.06       # m, end of planned path
HEADING_TOLERANCE = 0.02    # rad
LINE_TOLERANCE = 0.004      # m, final along-line position error
LINE_MAX_SPEED = 0.12
LINE_GAIN = 1.5
CROSS_TRACK_GAIN = 2.5
TURN_GAIN = 2.5
MAX_TURN = 0.8
HOLD_GAIN = 2.0
HOLD_MAX_SPEED = 0.05
REPLAN_PERIOD = 2.0          # s of sim time between map checks while driving
MAX_REPLANS = 3


class SkillError(RuntimeError):
    """A skill could not complete; the message says why."""


def rotate_to(robot: Stretch, runner: SimRunner, yaw: float, timeout: float = 20.0) -> None:
    def err() -> float:
        return wrap_angle(yaw - robot.base_pose().yaw)

    ok = runner.run_until(lambda: abs(err()) < HEADING_TOLERANCE, timeout, lambda: robot.drive(0.0, float(np.clip(TURN_GAIN * err(), -MAX_TURN, MAX_TURN))))
    robot.stop()
    runner.wait(0.2)
    if not ok:
        raise SkillError(f"rotate_to {math.degrees(yaw):.0f} deg timed out")


def drive_line(
    robot: Stretch, runner: SimRunner, goal: BasePose, timeout: float = 30.0, tolerance: float = LINE_TOLERANCE, fix_heading: bool = True
) -> None:
    """Drive forward/backward along goal's heading line until reaching goal's position.

    fix_heading rotates in place at the end; disable it when the arm is extended.
    """
    heading = np.array([math.cos(goal.yaw), math.sin(goal.yaw)])
    normal = np.array([-heading[1], heading[0]])
    hold = runner.background if isinstance(runner.background, BaseHold) else None
    previous_hold = hold.pose if hold else None
    if hold:
        hold.pose = goal  # hold the new pose once we get there

    def errors() -> tuple[float, float]:
        d = robot.base_pose().xy - goal.xy
        return float(d @ heading), float(d @ normal)  # along (negative = behind goal), cross-track

    def control() -> None:
        along, cross = errors()
        v = float(np.clip(-LINE_GAIN * along, -LINE_MAX_SPEED, LINE_MAX_SPEED))
        desired = goal.yaw - math.atan(CROSS_TRACK_GAIN * cross) * (1.0 if v >= 0 else -1.0)
        w = float(np.clip(TURN_GAIN * wrap_angle(desired - robot.base_pose().yaw), -MAX_TURN, MAX_TURN))
        robot.drive(v, w)

    ok = runner.run_until(lambda: abs(errors()[0]) < tolerance, timeout, control)
    robot.stop()
    runner.wait(0.3)
    if not ok:
        if hold:
            hold.pose = previous_hold  # don't keep servoing toward a goal we never reached
        raise SkillError(f"drive_line to ({goal.x:.2f}, {goal.y:.2f}) timed out (along error {errors()[0]:.3f} m)")
    if fix_heading and abs(wrap_angle(goal.yaw - robot.base_pose().yaw)) > HEADING_TOLERANCE:
        rotate_to(robot, runner, goal.yaw)  # cross-track steering can leave a small heading error


def shift_along_heading(robot: Stretch, runner: SimRunner, distance: float, heading: float | None = None) -> None:
    """Move the base `distance` m along `heading` (default: current heading; negative = backwards).

    Passing the task heading keeps repeated small corrections from accumulating yaw drift.
    """
    pose = robot.base_pose()
    yaw = pose.yaw if heading is None else heading
    target = BasePose(pose.x + distance * math.cos(yaw), pose.y + distance * math.sin(yaw), yaw)
    drive_line(robot, runner, target, timeout=10.0 + abs(distance) * 15, fix_heading=False)


class BaseHold:
    """Background controller that keeps the base at `pose` (heading + along-track position).

    Arm motions push the base around; this mimics the real robot's base servo.
    """

    def __init__(self, robot: Stretch, pose: BasePose):
        self.robot = robot
        self.pose = pose

    def __call__(self) -> None:
        current = self.robot.base_pose()
        heading = np.array([math.cos(self.pose.yaw), math.sin(self.pose.yaw)])
        along = float((current.xy - self.pose.xy) @ heading)
        v = float(np.clip(-HOLD_GAIN * along, -HOLD_MAX_SPEED, HOLD_MAX_SPEED))
        w = float(np.clip(TURN_GAIN * wrap_angle(self.pose.yaw - current.yaw), -MAX_TURN, MAX_TURN))
        self.robot.drive(v, w)


def step_back_from(robot: Stretch, runner: SimRunner, normal: np.ndarray, distance: float) -> None:
    """Move the base `distance` m along a surface's outward `normal` and restore its heading.

    A diff-drive base can't strafe, so: turn to face along the normal, drive, turn back.
    Only call this with the arm retracted.
    """
    pose = robot.base_pose()
    rotate_to(robot, runner, math.atan2(normal[1], normal[0]))
    target = pose.xy + np.asarray(normal) * distance
    drive_line(robot, runner, BasePose(target[0], target[1], math.atan2(normal[1], normal[0])), fix_heading=False)
    rotate_to(robot, runner, pose.yaw)


def _plan(maps: MapProvider, start: np.ndarray, goal: np.ndarray) -> list[np.ndarray]:
    grid = maps.refresh()
    try:
        return shortcut_path(grid, astar(grid, start, goal))
    except PlanningError as exc:
        raise SkillError(str(exc)) from exc


def _path_length(path: list[np.ndarray]) -> float:
    return sum(float(np.linalg.norm(b - a)) for a, b in zip(path, path[1:]))


def follow_path(robot: Stretch, runner: SimRunner, maps: MapProvider, path: list[np.ndarray], timeout: float) -> None:
    """Pure-pursuit along `path`; every REPLAN_PERIOD the (live) map is refreshed and, if the
    rest of the path now crosses an obstacle, a new path is planned from where the robot is."""
    goal = path[-1]
    state = {"path": list(path), "next_check": runner.time + REPLAN_PERIOD, "replans": 0}
    params = PursuitParams()

    def remaining_blocked(grid) -> bool:
        points = state["path"]
        return any(not line_is_free(grid, a, b) for a, b in zip(points, points[1:])) or not grid.is_free_xy(goal)

    def control() -> None:
        if runner.time >= state["next_check"]:
            state["next_check"] = runner.time + REPLAN_PERIOD
            if remaining_blocked(maps.refresh()):
                if state["replans"] >= MAX_REPLANS:
                    raise SkillError(f"path kept getting blocked ({MAX_REPLANS} replans)")
                state["replans"] += 1
                state["path"] = _plan(maps, robot.base_pose().xy, goal)
        pose = robot.base_pose()
        state["path"] = advance_path(pose.xy, state["path"])
        robot.drive(*pure_pursuit((pose.x, pose.y, pose.yaw), state["path"], params))

    try:
        ok = runner.run_until(lambda: np.linalg.norm(robot.base_pose().xy - goal) < GOAL_TOLERANCE, timeout, control)
    finally:
        robot.stop()
    if not ok:
        raise SkillError(f"path following timed out {np.linalg.norm(robot.base_pose().xy - goal):.2f} m from goal")


def navigate_to(robot: Stretch, runner: SimRunner, maps: MapProvider, goal: BasePose, stage_distance: float) -> list[np.ndarray]:
    """Plan to a staging point behind `goal`, align heading, then drive the final straight line."""
    stage = stage_pose(goal, stage_distance)
    if not maps.refresh().is_free_xy(stage.xy):
        raise SkillError(f"staging point ({stage.x:.2f}, {stage.y:.2f}) is blocked")
    path = _plan(maps, robot.base_pose().xy, stage.xy)
    follow_path(robot, runner, maps, path, timeout=30.0 + _path_length(path) * 12)
    rotate_to(robot, runner, goal.yaw)
    drive_line(robot, runner, goal, timeout=15.0 + stage_distance * 15)
    return path


def head_angles_to_look_at(model: mujoco.MjModel, data: mujoco.MjData, camera: str, target: np.ndarray) -> tuple[float, float]:
    """Solve head pan/tilt that centres `target` in `camera` (Gauss-Newton on a scratch copy)."""
    scratch = mujoco.MjData(model)
    scratch.qpos[:] = data.qpos
    pan_adr = model.jnt_qposadr[model.joint(ROBOT_PREFIX + "joint_head_pan").id]
    tilt_adr = model.jnt_qposadr[model.joint(ROBOT_PREFIX + "joint_head_tilt").id]
    pan_rng = model.jnt_range[model.joint(ROBOT_PREFIX + "joint_head_pan").id]
    tilt_rng = model.jnt_range[model.joint(ROBOT_PREFIX + "joint_head_tilt").id]
    cam = model.camera(camera).id

    def residual(q: np.ndarray) -> np.ndarray:
        scratch.qpos[pan_adr], scratch.qpos[tilt_adr] = q
        mujoco.mj_kinematics(model, scratch)
        mujoco.mj_camlight(model, scratch)
        direction = target - scratch.cam_xpos[cam]
        direction = direction / np.linalg.norm(direction)
        mat = scratch.cam_xmat[cam].reshape(3, 3)
        # Camera looks along -z; centred when the direction has no x/y component.
        return np.array([direction @ mat[:, 0], direction @ mat[:, 1], min(0.0, -(direction @ mat[:, 2]))])

    q = np.array([scratch.qpos[pan_adr], scratch.qpos[tilt_adr]], dtype=float)
    for _ in range(40):
        r = residual(q)
        jac = np.stack([(residual(q + dq) - r) / 1e-4 for dq in (np.array([1e-4, 0.0]), np.array([0.0, 1e-4]))], axis=1)
        step = np.linalg.lstsq(jac, -r, rcond=None)[0]
        q = np.array([np.clip(q[0] + step[0], *pan_rng), np.clip(q[1] + step[1], *tilt_rng)])
        if np.linalg.norm(step) < 1e-5:
            break
    return float(q[0]), float(q[1])


def look_at(robot: Stretch, runner: SimRunner, target: np.ndarray, settle: float = 1.2) -> None:
    pan, tilt = head_angles_to_look_at(robot.model, robot.estimated_data(), robot.head_camera, np.asarray(target, dtype=float))
    robot.command(head_pan=pan, head_tilt=tilt)
    runner.wait(settle)
