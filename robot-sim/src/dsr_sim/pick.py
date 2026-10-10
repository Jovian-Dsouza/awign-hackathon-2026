"""Task machinery shared by all simulations: stepping with pause/reset, rover drive, arm IK and motion,
reach planning. The simulations themselves live in scenarios.py (and skills.py).

`dsr-pick` still works and runs the shelf_to_tray simulation headless.
Viewer keys (the stock viewer shortcuts do not drive a scripted run, so these are ours):
  Space      pause / resume        R or Backspace   reset to the start (then Space to play)
  N          next simulation       ] / [            faster / slower
  = / -      zoom in / out
  1 2 3 4    camera: overview / robot / shelf / tray
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

from dsr_sim.constants import DEFAULT_SCENE, PROJECT_ROOT
from dsr_sim.control import Robot
from dsr_sim.render import write_png
from dsr_sim.robot import ARM_JOINTS, ARM_PREFIX

ARM_IK_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")
GRIPPER_OPEN_DEG = 52.0  # tuned in the simulator: passes a 0.06 m can without shoving it, narrow enough for the racks
GRIPPER_RELEASE_DEG = 70.0  # wide enough that a 0.06 m can is free of both jaws
GRIPPER_CLOSED_DEG = -5.0  # past closed: the item stops the jaw and the actuator squeezes
SHOULDER_FORWARD = 0.069  # arm base x offset from rover centre (mount 0.03 + shoulder 0.039)
PINCH_Z = 0.036  # jaw-middle offset from the gripperframe site along the jaw-opening axis (tuned: centres a 0.06 m can between the fingers)
GRASP_ROLL = 90.0  # wrist roll that makes the jaws open sideways (horizontal)
PRE_REACH = 0.37  # preferred shoulder-to-tool distance at the rack mouth (longest first; see plan_reach)
REACH_CANDIDATES = (0.37, 0.34, 0.31, 0.28, 0.25, 0.22)
TRAY_REACH = 0.28
IK_ROT_WEIGHT = 0.25  # weight of the jaw-axis (heading) error relative to position error
IK_OK_ERR = 0.025  # m: a plan is "reachable" if the IK lands within this of the target
IK_OK_TILT = 0.12  # rad of jaw-axis error

# Camera presets for the viewer: (azimuth, elevation, distance, lookat)
VIEWS = {
    "1": (90, -30, 2.3, (0.0, 0.0, 0.1)),
    "2": (130, -22, 0.8, (-0.7, -0.1, 0.12)),
    "3": (90, -18, 0.8, (-0.5, 0.2, 0.2)),
    "4": (60, -30, 0.9, (0.8, 0.0, 0.05)),
}


_JAW_GEOMS: dict[int, list[tuple[int, int, int]]] = {}  # model id -> jaw collision geoms and their original filters


class Restart(Exception):
    """Raised inside the running task when the user asks for a reset."""


class Closed(Exception):
    """Raised when the viewer window was closed."""


class RunControl:
    """Playback state shared between the viewer's key callback and the scripted run."""

    def __init__(self, paused: bool = False, speed: float = 1.0) -> None:
        self.paused = paused
        self.speed = speed
        self.reset_requested = False
        self.next_requested = False

    def on_key(self, keycode: int) -> None:
        if keycode == 32:  # space
            self.paused = not self.paused
            print("[paused]" if self.paused else "[playing]")
        elif keycode in (259, 82, 8):  # backspace / R
            self.reset_requested = True
        elif keycode == 78:  # N: next simulation
            self.next_requested = True
            self.reset_requested = True
        elif keycode == 93:  # ]
            self.speed = min(8.0, self.speed * 1.5)
            print(f"[speed x{self.speed:.2f}]")
        elif keycode == 91:  # [
            self.speed = max(0.1, self.speed / 1.5)
            print(f"[speed x{self.speed:.2f}]")


class Task:
    def __init__(self, model, data, viewer=None, speed: float = 1.0, frames_dir: Path | None = None, control: RunControl | None = None):
        self.model, self.data, self.viewer, self.speed, self.frames_dir = model, data, viewer, speed, frames_dir
        self.control = control
        self._last_time = data.time
        self.robot = Robot(model, data)
        self.start_pose = self.robot.pose  # where the rover began (the dock): runs end back here
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
        self.base_q = [model.jnt_qposadr[model.joint(n).id] for n in ("robot/base_x", "robot/base_y", "robot/base_yaw")]
        self.observers: list = []  # called after every physics step (data recording hooks in here)
        self.stage = ""  # current stage label, for recordings
        self.notes: dict = {}  # measured/chosen values worth logging (headings, reaches, offsets...)
        self.last_ik = {"err": 0.0, "tilt": 0.0}
        self.attached: str | None = None
        self.assist = True  # grasp assist (weld) on/off; set from the run parameters

    def say(self, message: str, stage: str | None = None) -> None:
        """Print a progress line and (optionally) mark the current stage for recordings."""
        print(message)
        if stage is not None:
            self.stage = stage

    # ------------------------------------------------------------ simulation stepping
    def _rebase(self) -> None:
        self._wall0, self._sim0 = time.time(), self.data.time

    def _poll(self) -> None:
        """Handle pause / reset / close. Blocks while paused; raises Restart or Closed."""
        ctl, viewer = self.control, self.viewer
        if viewer is None or ctl is None:
            return
        if self.data.time < self._last_time - 1e-9:  # the viewer's own Reset button rewound the clock
            ctl.reset_requested = True
        if ctl.paused and not ctl.reset_requested:
            while ctl.paused and not ctl.reset_requested and viewer.is_running():
                viewer.sync()
                time.sleep(0.03)
            self._rebase()
        if not viewer.is_running():
            raise Closed
        if ctl.reset_requested:
            raise Restart
        if ctl.speed != self.speed:
            self.speed = ctl.speed
            self._rebase()

    def _tick(self) -> None:
        self._poll()
        mujoco.mj_step(self.model, self.data)
        self._last_time = self.data.time
        for observer in self.observers:
            observer(self)
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
    def drive_to(
        self,
        x: float,
        y: float,
        yaw: float,
        speed: float = 0.35,
        timeout: float = 20.0,
        accel: float = 0.8,
        ang_accel: float = 3.0,
        precise: bool = False,
    ) -> None:
        """Holonomic go-to with acceleration limits (so a held item is not jerked out of the gripper).

        `precise` waits for the heading to settle (0.3 deg): at arm's length a degree of yaw is a centimetre
        at the jaws, so stand-offs in front of an item use it.
        """
        tol_pos, tol_yaw = (0.004, 0.006) if precise else (0.006, 0.035)
        end = self.data.time + timeout
        cur = np.zeros(3)  # commanded body twist
        while self.data.time < end:
            px, py, pyaw = self.robot.pose
            ex, ey = x - px, y - py
            eyaw = (yaw - pyaw + np.pi) % (2 * np.pi) - np.pi
            dist = float(np.hypot(ex, ey))
            if dist < tol_pos and abs(eyaw) < tol_yaw:
                break
            # cruise, then brake so we can stop within the remaining distance (never creep below 3 cm/s)
            v = min(speed, max(0.03, float(np.sqrt(2.0 * accel * 0.7 * dist))))
            wx, wy = (ex / dist * v, ey / dist * v) if dist > 1e-6 else (0.0, 0.0)
            c, s_ = np.cos(pyaw), np.sin(pyaw)
            want = np.array([c * wx + s_ * wy, -s_ * wx + c * wy, float(np.clip(4.0 * eyaw, -1.5, 1.5))])
            step = np.array([accel, accel, ang_accel]) * self.dt
            cur = cur + np.clip(want - cur, -step, step)
            self.robot.set_twist(*cur)
            self._tick()
        self.robot.stop()
        self.run(0.15)

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
            # Position matters more than heading: a few degrees of jaw-axis error costs only millimetres
            # sideways over the short drive-in, while position error would miss the item entirely.
            J = np.vstack([jacp[:, self.ik_dofs], IK_ROT_WEIGHT * jacr[:, self.ik_dofs]])
            e = np.concatenate([e_pos, IK_ROT_WEIGHT * e_rot])
            dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), e)
            for k, adr in enumerate(self.ik_qadr):
                d.qpos[adr] = np.clip(d.qpos[adr] + 0.5 * dq[k], *self.ik_range[k])

    def solve_ik(
        self,
        target: np.ndarray,
        forward: np.ndarray,
        roll_deg: float = 0.0,
        pinch: float = 0.0,
        iters: int = 300,
        base_pose: tuple[float, float, float] | None = None,
        quiet: bool = False,
    ) -> dict[str, float]:
        """Joint angles (deg) that put the gripper at `target` (world) with the jaws pointing along `forward`.

        `pinch` is the distance from the gripperframe site to the middle of the open jaws, along the site's
        z axis (the jaw-opening direction); the target is the point that should sit between the jaws.
        """
        d = self._ik
        d.qpos[:] = self.data.qpos  # rover pose etc. from the live state
        if base_pose is not None:  # plan from a hypothetical rover pose (world x, y, yaw)
            px, py, pyaw = self.robot.pose
            d.qpos[self.base_q[0]] += base_pose[0] - px
            d.qpos[self.base_q[1]] += base_pose[1] - py
            d.qpos[self.base_q[2]] = base_pose[2]
        # Warm-start from the *commanded* arm angles (not the lagging measured ones) so the solution
        # does not depend on how settled the arm happens to be.
        for j, adr in zip(ARM_IK_JOINTS, self.ik_qadr):
            d.qpos[adr] = np.deg2rad(self.arm_cmd[j])
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
        self.last_ik = {"err": err, "tilt": tilt}
        if not quiet and (err > 0.01 or tilt > 0.12):
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

    def plan_reach(self, mouth: np.ndarray, heading: float, candidates=REACH_CANDIDATES, margin_z: float = 0.02) -> float | None:
        """Longest shoulder-to-tool distance (from `candidates`) for which the arm can hold the jaws level at
        `mouth` (the point just outside the item, at grasp height) with the rover standing that far back,
        facing `heading`. Returns None if nothing is reachable. Driving forward later carries the same joint
        solution in to the item, so feasibility at the mouth is feasibility of the grasp."""
        fwd = np.array([np.cos(heading), np.sin(heading), 0.0])
        for reach in candidates:
            stage = mouth[:2] - fwd[:2] * (reach + SHOULDER_FORWARD)
            ok = True
            for dz in (margin_z, 0.0):
                tgt = np.array([mouth[0], mouth[1], mouth[2] + dz])
                self.solve_ik(tgt, fwd, GRASP_ROLL, PINCH_Z, base_pose=(stage[0], stage[1], heading), quiet=True)
                if self.last_ik["err"] > IK_OK_ERR or self.last_ik["tilt"] > IK_OK_TILT:
                    ok = False
                    break
            if ok:
                return reach
        return None

    def set_assist(self, on: bool) -> None:
        """Grasp assist on/off.

        On: items are carried by a weld constraint (see attach) and the jaws are 'ghosts' to items - they no
        longer collide with them - so a released item is never dragged out by a finger. This models a good,
        clean grasp; it says nothing about friction grip quality.
        Off: full contact physics between jaws and items (fragile with this gripper and a 5 cm can).
        """
        self.assist = on
        m = self.model
        geoms = _JAW_GEOMS.get(id(m))
        if geoms is None:  # remember the original collision filters the first time
            geoms = []
            for body in (ARM_PREFIX + "gripper", ARM_PREFIX + "moving_jaw_so101_v1"):
                bid = m.body(body).id
                geoms += [(g, int(m.geom_contype[g]), int(m.geom_conaffinity[g])) for g in range(m.ngeom) if m.geom_bodyid[g] == bid and m.geom_contype[g]]
            _JAW_GEOMS[id(m)] = geoms
        for g, ct, ca in geoms:
            m.geom_contype[g], m.geom_conaffinity[g] = (2, 2) if on else (ct, ca)

    def jaw_centre(self, pinch: float = PINCH_Z) -> np.ndarray:
        d = self.data
        return d.site_xpos[self.site] + d.site_xmat[self.site].reshape(3, 3)[:, 2] * pinch

    def attach(self, name: str, max_distance: float = 0.06) -> bool:
        """Grasp assist: weld the item to the gripper, but only if the jaws are really around it.

        Returns False (and attaches nothing) if the item is not within `max_distance` of the jaw centre.
        """
        m, d = self.model, self.data
        if np.linalg.norm(self.item_pos(name) - self.jaw_centre()) > max_distance:
            return False
        eq = m.equality(f"grip_weld_{name}").id
        g, it = m.body(ARM_PREFIX + "gripper").id, m.body(f"item_{name}").id
        r1 = d.xmat[g].reshape(3, 3)
        rel_pos = r1.T @ (d.xpos[it] - d.xpos[g])
        q_inv = np.zeros(4)
        mujoco.mju_negQuat(q_inv, d.xquat[g])
        rel_quat = np.zeros(4)
        mujoco.mju_mulQuat(rel_quat, q_inv, d.xquat[it])
        m.eq_data[eq, :3] = 0.0  # anchor: origin of body2
        m.eq_data[eq, 3:6] = rel_pos
        m.eq_data[eq, 6:10] = rel_quat
        d.eq_active[eq] = 1
        self.attached = name
        return True

    def detach(self) -> None:
        name = getattr(self, "attached", None)
        if name:
            self.data.eq_active[self.model.equality(f"grip_weld_{name}").id] = 0
            self.attached = None

    def return_to_start(self) -> None:
        """Fold the arm and drive back to the starting pose."""
        self.move_arm(dict(zip(ARM_JOINTS, (0, -100, 95, 65, 0, 0))), 1.0)
        self.drive_to(*self.start_pose, speed=0.35)

    def item_pos(self, name: str) -> np.ndarray:
        return self.data.xpos[self.model.body(f"item_{name}").id].copy()


def _viewer_keys(holder: dict, control: RunControl):
    def on_key(keycode: int) -> None:
        control.on_key(keycode)
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
    """Kept for `dsr-pick`: runs the shelf_to_tray simulation. See `dsr_sim.run` for the selector."""
    from dsr_sim import run

    argv = list(sys.argv[1:] if argv is None else argv)
    if not any(a == "--task" or a.startswith("--task=") for a in argv):
        argv = ["--task", "shelf_to_tray", *argv]
    if "--viewer" in argv:
        argv.remove("--viewer")
    elif "--headless" not in argv:
        argv.append("--headless")
    return run.main(argv)


if __name__ == "__main__":
    sys.exit(main())
