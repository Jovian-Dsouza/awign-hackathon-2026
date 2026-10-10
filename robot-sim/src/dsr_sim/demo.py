"""Scripted check of the robot: drive the mecanum base in every direction, move the arm,
and save frames from the overview and wrist cameras.

Usage:  uv run dsr-demo [--out-dir renders]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import mujoco

from dsr_sim.constants import DEFAULT_SCENE, PROJECT_ROOT
from dsr_sim.control import Robot
from dsr_sim.render import write_png

CAMERAS = ("overview", "wrist_cam")
SIZE = (480, 640)  # height, width


def run(model: mujoco.MjModel, data: mujoco.MjData, robot: Robot, seconds: float) -> None:
    end = data.time + seconds
    while data.time < end:
        mujoco.mj_step(model, data)


def snapshot(model, data, renderer, out_dir: Path, tag: str) -> None:
    for cam in CAMERAS:
        renderer.update_scene(data, camera=cam)
        write_png(out_dir / f"{tag}_{cam}.png", renderer.render())


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--out-dir", type=Path, default=PROJECT_ROOT / "renders")
    args = parser.parse_args(argv)
    args.out_dir.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(args.scene))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    robot = Robot(model, data)
    robot.set_arm_deg("rest")

    moves = [
        ("forward", (0.2, 0.0, 0.0), 2.0),
        ("strafe_left", (0.0, 0.2, 0.0), 2.0),
        ("rotate", (0.0, 0.0, 0.8), 2.0),
    ]
    with mujoco.Renderer(model, *SIZE) as renderer:
        run(model, data, robot, 0.5)
        snapshot(model, data, renderer, args.out_dir, "00_start")
        for i, (name, twist, secs) in enumerate(moves, start=1):
            x0, y0, yaw0 = robot.pose
            robot.set_twist(*twist)
            run(model, data, robot, secs)
            robot.stop()
            run(model, data, robot, 0.3)
            x1, y1, yaw1 = robot.pose
            print(f"{name:12s} dx={x1 - x0:+.3f} dy={y1 - y0:+.3f} dyaw={yaw1 - yaw0:+.3f} rad")
            snapshot(model, data, renderer, args.out_dir, f"{i:02d}_{name}")
        for pose in ("ready", "reach_low"):
            robot.set_arm_deg(pose)
            run(model, data, robot, 2.0)
            print(f"arm {pose:10s}", {k: round(v) for k, v in robot.arm_deg().items()})
            snapshot(model, data, renderer, args.out_dir, f"arm_{pose}")
    print(f"[ok] frames in {args.out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
