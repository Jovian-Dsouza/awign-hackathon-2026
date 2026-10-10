"""Open the warehouse scene in the interactive MuJoCo viewer.

Usage:  uv run python -m dsr_sim.view [--scene S] [--rebuild]

Uses the blocking `mujoco.viewer.launch`, which works with plain `python` on
macOS (only `launch_passive` requires `mjpython`). Press Space to pause, use
the "Rendering > Camera" menu to jump to the named cameras, and toggle group 4
sites to see shelf-slot / drop-zone markers.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import mujoco
import mujoco.viewer

from dsr_sim import build_scene
from dsr_sim.constants import DEFAULT_SCENE


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--rebuild", action="store_true", help="regenerate the scene from config/ first")
    args = parser.parse_args(argv)

    if args.rebuild or not args.scene.exists():
        status = build_scene.main(["--out", str(args.scene)])
        if status != 0:
            return status

    try:
        model = mujoco.MjModel.from_xml_path(str(args.scene))
    except ValueError as exc:
        print(f"[error] failed to load {args.scene}: {exc}", file=sys.stderr)
        return 1
    data = mujoco.MjData(model)
    if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home") >= 0:
        mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    mujoco.viewer.launch(model, data)
    return 0


if __name__ == "__main__":
    sys.exit(main())
