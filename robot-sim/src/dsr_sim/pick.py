"""Scripted pick-and-carry: drive to the shelf, pick one item from its rack, carry it to the
drop tray and release it. Physics-based grasp (the item is held by the gripper's friction).

Usage:
  uv run dsr-pick                       # headless, prints a stage log and the result
  uv run mjpython -m dsr_sim.pick --viewer [--speed 1.0]    # live viewer (macOS needs mjpython)
Viewer keys: = / - zoom in/out, 1 overview, 2 robot close-up, 3 shelf, 4 tray.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from dsr_sim.constants import DEFAULT_SCENE, PROJECT_ROOT
from dsr_sim.control import Robot
from dsr_sim.render import write_png
from dsr_sim.robot import ARM_JOINTS, ARM_PREFIX

ARM_IK_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")
GRIPPER_OPEN_DEG = 60.0  # gap ~0.08 m at the finger middle: clears a 0.06 m can
GRIPPER_CLOSED_DEG = -5.0  # past closed: the item stops the jaw and the actuator squeezes
SHOULDER_FORWARD = 0.069  # arm base x offset from rover centre (mount 0.03 + shoulder 0.039)
PINCH_Z = 0.026  # jaw-middle offset from the gripperframe site, along the jaw-opening axis
GRASP_ROLL = 90.0  # wrist roll that makes the jaws open sideways (horizontal)
PRE_REACH = 0.37  # shoulder-to-tool distance at the rack mouth: level jaws this low need a long reach
TRAY_REACH = 0.28

# Camera presets for the viewer: (azimuth, elevation, distance, lookat)
VIEWS = {
    "1": (90, -30, 2.3, (0.0, 0.0, 0.1)),
    "2": (130, -22, 0.8, (-0.7, -0.1, 0.12)),
    "3": (90, -18, 0.8, (-0.5, 0.2, 0.2)),
    "4": (60, -30, 0.9, (0.8, 0.0, 0.05)),
}


@dataclass
class Pick:
    item: str = "alphabet_soup"
    tray: str = "drop_tray"
    z_off: float = 0.015  # grasp height above the item's centre
    y_off: float = 0.03  # how far past the item centre the gripperframe site goes (into the rack)


class Task:
    def __init__(self, model, data, viewer=None, speed: float = 1.0, frames_dir: Path | None = None):
        self.model, self.data, self.viewer, self.speed, self.frames_dir = model, data, viewer, speed, frames_dir
        self.robot = Robot(model, data)
        self.dt = model.opt.timestep
        self._wall0 = time.time()
        self._sim0 = data.time
        self._frame = 0
        self._ik = mujoco.MjData(model)
        self.site = model.site(ARM_PREFIX + "gripperframe").id
        self.ik_dofs = [model.jnt_dofadr[model.joint(ARM_PREFIX + j).id] for j in ARM_IK_JOINTS]
        self.ik_qadr = [model.jnt_qposadr[model.joint(ARM_PREFIX + j).id] for j in ARM_IK_JOINTS]
        self.roll_qadr = model.jnt_qposadr[model.joint(ARM_PREFIX + "wrist_roll").id]
        self.ik_range = np.array([model.jnt_range[model.joint(ARM_PREFIX + j).id] for j in ARM_IK_JOINTS])
        self.arm_cmd = {j: float(np.rad2deg(data.ctrl[model.actuator(ARM_PREFIX + j).id])) for j in ARM_JOINTS}

    # ------------------------------------------------------------ simulation stepping
    def _tick(self) -> None:
        mujoco.mj_step(self.model, self.data)
        if self.viewer is not None and self.data.time - getattr(self, "_last_sync", -1) >= 1 / 60:
            self._last_sync = self.data.time
            self.viewer.sync()
            ahead = (self.data.time - self._sim0) / self.speed - (time.time() - self._wall0)
            if ahead > 0:
                time.sleep(min(ahead, 0.05))

    def run(self, seconds: float) -> None:
        end = self.data.time + seconds
        while self.data.time < end:
            self._tick()

    def snapshot(self, tag: str) -> None:
        if self.frames_dir is None:
            return
        self.frames_dir.mkdir(parents=True, exist_ok=True)
        with mujoco.Renderer(self.model, 480, 640) as rd:
            for cam in ("overview", "wrist_cam"):
                rd.update_scene(self.data, camera=cam)
                write_png(self.frames_dir / f"{self._frame:02d}_{tag}_{cam}.png", rd.render())
        self._frame += 1

    # ------------------------------------------------------------ rover
    def drive_to(self, x: float, y: float, yaw: float, speed: float = 0.25, timeout: float = 25.0, accel: float = 0.35, ang_accel: float = 1.5) -> None:
        """Holonomic go-to with acceleration limits (so a held item is not jerked out of the gripper)."""
        end = self.data.time + timeout
        cur = np.zeros(3)  # commanded body twist
        while self.data.time < end:
            px, py, pyaw = self.robot.pose
            ex, ey = x - px, y - py
            eyaw = (yaw - pyaw + np.pi) % (2 * np.pi) - np.pi
            dist = float(np.hypot(ex, ey))
            if dist < 0.004 and abs(eyaw) < 0.01 and np.linalg.norm(cur) < 0.02:
                break
            # speed profile: cruise, then brake so that we can stop within the remaining distance
            v = min(speed, np.sqrt(2.0 * accel * 0.8 * dist) + 0.02)
            wx, wy = (ex / dist * v, ey / dist * v) if dist > 1e-6 else (0.0, 0.0)
            c, s_ = np.cos(pyaw), np.sin(pyaw)
            want = np.array([c * wx + s_ * wy, -s_ * wx + c * wy, float(np.clip(1.5 * eyaw, -0.8, 0.8))])
            step = np.array([accel, accel, ang_accel]) * self.dt
            cur = cur + np.clip(want - cur, -step, step)
            self.robot.set_twist(*cur)
            self._tick()
        self.robot.stop()
        self.run(0.3)

    # ------------------------------------------------------------ arm
    def _solve(self, target: np.ndarray, forward: np.ndarray, iters: int) -> None:
        d = self._ik
        jacp, jacr = np.zeros((3, self.model.nv)), np.zeros((3, self.model.nv))
        for _ in range(iters):
            mujoco.mj_kinematics(self.model, d)
            mujoco.mj_comPos(self.model, d)
            pos = d.site_xpos[self.site]
            axis = d.site_xmat[self.site].reshape(3, 3)[:, 0]  # SO-101 gripperframe: x is the jaw approach direction
            e_pos = target - pos
            e_rot = np.cross(axis, forward)
            if np.linalg.norm(e_pos) < 5e-4 and np.linalg.norm(e_rot) < 0.01:
                break
            mujoco.mj_jacSite(self.model, d, jacp, jacr, self.site)
            J = np.vstack([jacp[:, self.ik_dofs], jacr[:, self.ik_dofs]])
            e = np.concatenate([e_pos, e_rot])
            dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), e)
            for k, adr in enumerate(self.ik_qadr):
                d.qpos[adr] = np.clip(d.qpos[adr] + 0.5 * dq[k], *self.ik_range[k])

    def solve_ik(self, target: np.ndarray, forward: np.ndarray, roll_deg: float = 0.0, pinch: float = 0.0, iters: int = 300) -> dict[str, float]:
        """Joint angles (deg) that put the gripper at `target` (world) with the jaws pointing along `forward`.

        `pinch` is the distance from the gripperframe site to the middle of the open jaws, along the site's
        z axis (the jaw-opening direction); the target is the point that should sit between the jaws.
        """
        d = self._ik
        d.qpos[:] = self.data.qpos
        d.qpos[self.roll_qadr] = np.deg2rad(roll_deg)
        forward = forward / np.linalg.norm(forward)
        self._solve(target, forward, iters)
        if pinch:
            zaxis = d.site_xmat[self.site].reshape(3, 3)[:, 2]
            self._solve(target - zaxis * pinch, forward, iters)
        mujoco.mj_kinematics(self.model, d)
        goal = target - (d.site_xmat[self.site].reshape(3, 3)[:, 2] * pinch)
        err = float(np.linalg.norm(goal - d.site_xpos[self.site]))
        tilt = float(np.linalg.norm(np.cross(d.site_xmat[self.site].reshape(3, 3)[:, 0], forward)))
        if err > 0.01 or tilt > 0.12:
            print(f"    [ik] residual {err * 1000:.1f} mm, tool tilt {np.rad2deg(np.arcsin(min(tilt, 1))):.0f} deg for target {np.round(target, 3)}")
        out = {j: float(np.rad2deg(d.qpos[a])) for j, a in zip(ARM_IK_JOINTS, self.ik_qadr)}
        out["wrist_roll"] = roll_deg
        return out

    def move_arm(self, targets: dict[str, float], seconds: float) -> None:
        """Smoothly move the commanded joint angles (degrees) to `targets` over `seconds`."""
        start = dict(self.arm_cmd)
        goal = {**start, **targets}
        t0 = self.data.time
        while self.data.time - t0 < seconds:
            u = (self.data.time - t0) / seconds
            u = u * u * (3 - 2 * u)
            for j in ARM_JOINTS:
                self.arm_cmd[j] = start[j] + (goal[j] - start[j]) * u
            self.robot.set_arm_deg(tuple(self.arm_cmd[j] for j in ARM_JOINTS))
            self._tick()
        self.arm_cmd = dict(goal)
        self.robot.set_arm_deg(tuple(goal[j] for j in ARM_JOINTS))

    def reach(self, target: np.ndarray, forward: np.ndarray, seconds: float, gripper: float, roll_deg: float = 0.0, pinch: float = 0.0) -> None:
        angles = self.solve_ik(target, forward, roll_deg, pinch)
        self.move_arm({**angles, "gripper": gripper}, seconds)

    def item_pos(self, name: str) -> np.ndarray:
        return self.data.xpos[self.model.body(f"item_{name}").id].copy()


def run_pick(task: Task, pick: Pick) -> bool:
    m, d = task.model, task.data
    print(f"pick {pick.item!r} -> {pick.tray!r}")
    item0 = task.item_pos(pick.item)
    tray_c = d.site_xpos[m.site("drop_zone").id].copy()
    print(f"  item at {np.round(item0, 3)}, tray at {np.round(tray_c, 3)}")
    task.snapshot("start")

    def where(tag: str) -> None:
        print(f"      [{tag}] item {np.round(task.item_pos(pick.item), 3)}")

    fwd = np.array([0.0, 1.0, 0.0])
    grasp_z = item0[2] + pick.z_off
    mouth_y = item0[1] - 0.095  # just outside the open front of the rack
    grasp_y = item0[1] + pick.y_off  # item centre ends up this far behind the gripperframe, mid-finger
    # Stage far enough back that the folded arm clears the shelf lip, then reach in by driving forward.
    # With the jaws level and this low, the SO-101 only reaches >= ~0.28 m out from its shoulder, so
    # stage far back; driving forward then carries the (fixed) arm pose in to the item.
    stage = (item0[0], mouth_y - PRE_REACH - SHOULDER_FORWARD, np.pi / 2)
    advance = grasp_y - mouth_y

    print("  1/6 drive to the shelf")
    task.drive_to(*stage)
    task.snapshot("staged")

    print("  2/6 pre-grasp at the rack mouth, drive in")
    task.reach(np.array([item0[0], mouth_y, grasp_z + 0.02]), fwd, 2.0, GRIPPER_OPEN_DEG, GRASP_ROLL, PINCH_Z)
    task.reach(np.array([item0[0], mouth_y, grasp_z]), fwd, 0.8, GRIPPER_OPEN_DEG, GRASP_ROLL, PINCH_Z)
    task.drive_to(stage[0], stage[1] + advance, stage[2], speed=0.08)
    task.snapshot("reached")
    where("in rack")

    print("  3/6 grasp")
    task.move_arm({"gripper": GRIPPER_CLOSED_DEG}, 1.0)
    task.run(0.6)
    where("gripped")

    print("  4/6 lift and back out")
    task.reach(np.array([item0[0], grasp_y, grasp_z + 0.02]), fwd, 0.8, GRIPPER_CLOSED_DEG, GRASP_ROLL, PINCH_Z)
    task.drive_to(*stage, speed=0.08)
    task.snapshot("grasped")
    where("backed out")
    task.move_arm(dict(zip(ARM_JOINTS, (0, -80, 85, 55, GRASP_ROLL, GRIPPER_CLOSED_DEG))), 1.5)  # tucked carry pose
    where("tucked")

    print("  5/6 carry to the tray")
    carry = (tray_c[0] - SHOULDER_FORWARD - TRAY_REACH, tray_c[1], 0.0)
    task.drive_to(*carry)
    task.snapshot("at_tray")
    where("at tray")

    print("  6/6 release over the tray")
    over = np.array([tray_c[0], tray_c[1], tray_c[2] + 0.09])
    task.reach(over, np.array([1.0, 0.0, 0.0]), 2.0, GRIPPER_CLOSED_DEG)
    task.move_arm({"gripper": GRIPPER_OPEN_DEG}, 0.8)
    task.run(1.5)
    task.move_arm(dict(zip(ARM_JOINTS, (0, -100, 95, 65, 0, 0))), 1.5)
    task.snapshot("done")

    final = task.item_pos(pick.item)
    inside = abs(final[0] - tray_c[0]) < 0.13 and abs(final[1] - tray_c[1]) < 0.09 and final[2] > -0.01
    print(f"  item ended at {np.round(final, 3)} -> {'IN TRAY' if inside else 'NOT in tray'}")
    return bool(inside)


def _viewer_keys(holder: dict):
    def on_key(keycode: int) -> None:
        viewer = holder.get("viewer")
        if viewer is None:
            return
        ch = chr(keycode) if 0 < keycode < 128 else ""
        cam = viewer.cam
        if ch in ("=", "+"):
            cam.distance = max(0.15, cam.distance * 0.85)
        elif ch == "-":
            cam.distance = min(6.0, cam.distance / 0.85)
        elif ch in VIEWS:
            az, el, dist, look = VIEWS[ch]
            cam.azimuth, cam.elevation, cam.distance = az, el, dist
            cam.lookat[:] = look

    return on_key


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--item", default="alphabet_soup")
    parser.add_argument("--viewer", action="store_true", help="live viewer (run with mjpython on macOS)")
    parser.add_argument("--speed", type=float, default=1.0, help="viewer playback speed vs real time")
    parser.add_argument("--frames-dir", type=Path, default=None, help="save overview/wrist snapshots per stage")
    args = parser.parse_args(argv)

    model = mujoco.MjModel.from_xml_path(str(args.scene))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    mujoco.mj_forward(model, data)

    if args.viewer:
        from mujoco import viewer as mj_viewer

        holder: dict = {}
        with mj_viewer.launch_passive(model, data, key_callback=_viewer_keys(holder)) as viewer:
            holder["viewer"] = viewer
            viewer.cam.azimuth, viewer.cam.elevation, viewer.cam.distance = VIEWS["1"][:3]
            viewer.cam.lookat[:] = VIEWS["1"][3]
            task = Task(model, data, viewer, args.speed, args.frames_dir)
            task.run(1.0)
            ok = run_pick(task, Pick(item=args.item))
            print("done: close the window to exit" if viewer.is_running() else "viewer closed")
            while viewer.is_running():
                viewer.sync()
                time.sleep(0.03)
        return 0 if ok else 1
    task = Task(model, data, None, 1.0, args.frames_dir)
    task.run(0.5)
    return 0 if run_pick(task, Pick(item=args.item)) else 1


if __name__ == "__main__":
    sys.exit(main())
